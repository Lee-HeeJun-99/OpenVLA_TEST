"""ROS-independent policy_movej planning and settlement primitives.

Nothing in this module publishes or calls a service.  An action is either
accepted in full or rejected; it is never clamped or projected.
"""
from __future__ import annotations

from dataclasses import dataclass
import threading
import time
import numpy as np

from controller.geometry import apply_action
from controller.precheck import joint_envelope_status,joint_margin_reasons
from kinematics import DLSIKSolver,IKConvergenceError,IKJointChangeError


class PolicyMoveJPrecheckError(RuntimeError):
    def __init__(self, reasons, rejected_candidate=None):
        self.reasons=tuple(dict.fromkeys(reasons))
        self.rejected_candidate=dict(rejected_candidate or {})
        super().__init__("POLICY_MOVEJ_PRECHECK:"+",".join(self.reasons))


@dataclass(frozen=True)
class MoveJPlan:
    target_joint_rad: np.ndarray
    target_tool_pose: object
    target_link_pose: object
    path_minimum_singular_values: tuple
    path_minimum_tool_z_m: float
    envelope_log_only: dict


def policy_movej_action_reasons(action, target_tool_position, config, streak_counts):
    """Policy-MoveJ action checks without translation/rotation magnitude caps.

    The user disabled both Cartesian action magnitude caps on 2026-11-13.  This
    still rejects every non-finite action and retains workspace and configured
    endpoint-policy checks.  It does not clamp or modify the action.
    """
    values=np.concatenate((np.asarray(action.raw,dtype=float).reshape(-1),
      np.asarray(action.delta_position_m,dtype=float).reshape(-1),
      np.asarray(action.rotation_vector_rad,dtype=float).reshape(-1),
      np.asarray([action.gripper_target],dtype=float)))
    if not np.all(np.isfinite(values)):
        return ["NONFINITE_ACTION"]
    reasons=[];p=np.asarray(target_tool_position,dtype=float)
    if np.any(p<config.workspace_min_m) or np.any(p>config.workspace_max_m):
        reasons.append("WORKSPACE_CANDIDATE_EXCEEDED")
    if (config.endpoint_policy=="stop" and
            np.any(np.asarray(streak_counts)>=config.endpoint_streak_limit)):
        dims=np.argwhere(np.asarray(streak_counts)>=config.endpoint_streak_limit).tolist()
        reasons.append(f"ENDPOINT_TOKEN_STREAK:{dims}")
    return reasons


def validate_sigma_override(default_value,requested_value=None,reason=None):
    """Return an audited threshold; lowering requires a non-empty reason."""
    default=float(default_value)
    value=default if requested_value is None else float(requested_value)
    if not np.isfinite(value) or value<=0:raise ValueError("sigma-min must be finite and positive")
    lowered=value<default-1e-12
    if lowered and not (reason and str(reason).strip()):
        raise ValueError("LOWER_SIGMA_MIN_REQUIRES_REASON")
    return {"value":value,"default":default,"lowered":lowered,
            "reason":str(reason).strip() if lowered else None}


def descent_limit_reason(home_tool_z_m,path_tool_z_m,max_descent_mm,index=None):
    if max_descent_mm is None:return None
    descent=(float(home_tool_z_m)-float(path_tool_z_m))*1000
    if descent<float(max_descent_mm)-1e-9:return None
    return (f"PATH_{index}:" if index is not None else "")+"DESCENT_LIMIT"


class AbortSignal:
    """Thread-safe operator request timestamp used for response-time audit."""
    def __init__(self,clock=time.monotonic):
        self.clock=clock;self.event=threading.Event();self.lock=threading.Lock();self.requested_at=None
    def request(self):
        with self.lock:
            if self.requested_at is None:self.requested_at=self.clock()
        self.event.set()
    def is_set(self):return self.event.is_set()
    def response_s(self,stop_requested_at):
        with self.lock:requested=self.requested_at
        return None if requested is None else max(0.0,float(stop_requested_at)-requested)


class MoveJPolicyPlanner:
    def __init__(self,kinematics,tool_transform,controller_config,runtime_config,
                 *,sigma_min=None,max_descent_mm=None,home_tool_z_m=None):
        self.kin=kinematics;self.tool=tool_transform;self.cc=controller_config
        self.rc=runtime_config
        self.sigma_min=float(self.cc.jacobian_min_singular_value if sigma_min is None else sigma_min)
        self.max_descent_mm=None if max_descent_mm is None else float(max_descent_mm)
        self.home_tool_z_m=None if home_tool_z_m is None else float(home_tool_z_m)
        if self.max_descent_mm is not None:
            if self.max_descent_mm<0:raise ValueError("max_descent_mm must be non-negative")
            if self.home_tool_z_m is None:raise ValueError("home_tool_z_m required with max_descent_mm")
        self.ik=DLSIKSolver(kinematics,
            position_tolerance_m=self.cc.ik_position_tolerance_m,
            rotation_tolerance_rad=self.cc.ik_rotation_tolerance_rad)

    def plan(self,action,command_tool_pose,seed_joint_rad,streak_counts):
        seed=np.asarray(seed_joint_rad,dtype=float)
        finite_reasons=policy_movej_action_reasons(action,command_tool_pose.position_m,
                                                    self.cc,streak_counts)
        if "NONFINITE_ACTION" in finite_reasons:
            raise PolicyMoveJPrecheckError(finite_reasons)
        target_tool=apply_action(command_tool_pose,action.delta_position_m,
                                 action.rotation_vector_rad,"world_left")
        diagnostic={"target_tool_pose":{"position_m":target_tool.position_m.tolist(),
          "rotation_matrix":target_tool.rotation.tolist()}}
        reasons=policy_movej_action_reasons(action,target_tool.position_m,self.cc,streak_counts)
        if reasons:raise PolicyMoveJPrecheckError(reasons,diagnostic)
        target_link=self.tool.tool_to_link(target_tool)
        max_delta=np.deg2rad(float(self.rc["policy_movej"]["max_joint_delta_deg_candidate"]))
        try:
            target_q=self.ik.solve(target_link,seed,
                joint_lower_rad=self.cc.effective_lower_rad,
                joint_upper_rad=self.cc.effective_upper_rad,
                max_joint_change_rad=max_delta)
        except IKJointChangeError as error:
            diagnostic.update(exception_message=str(error))
            if error.candidate_joint_rad is not None:
                diagnostic["ik_result_joint_rad"]=error.candidate_joint_rad.tolist()
                diagnostic["joint_delta_rad"]=(error.candidate_joint_rad-seed).tolist()
                diagnostic["joint_delta_deg"]=np.rad2deg(error.candidate_joint_rad-seed).tolist()
            raise PolicyMoveJPrecheckError(["JOINT_STEP_CANDIDATE_EXCEEDED"],diagnostic) from error
        except IKConvergenceError as error:
            diagnostic.update(exception_message=str(error),iterations=error.iterations,
                              best_residual=error.best_residual)
            if error.best_joint_rad is not None:
                diagnostic["best_joint_rad"]=np.asarray(error.best_joint_rad).tolist()
                diagnostic["best_joint_deg"]=np.rad2deg(error.best_joint_rad).tolist()
                diagnostic["joint_delta_rad"]=(np.asarray(error.best_joint_rad)-seed).tolist()
                diagnostic["joint_delta_deg"]=np.rad2deg(np.asarray(error.best_joint_rad)-seed).tolist()
            raise PolicyMoveJPrecheckError(["IK_REJECTED:IKConvergenceError"],diagnostic) from error
        except Exception as error:
            diagnostic["exception_message"]=str(error)
            raise PolicyMoveJPrecheckError(["IK_REJECTED:"+type(error).__name__],diagnostic) from error
        diagnostic.update(ik_result_joint_rad=target_q.tolist(),
          ik_result_joint_deg=np.rad2deg(target_q).tolist(),
          joint_delta_rad=(target_q-seed).tolist(),
          joint_delta_deg=np.rad2deg(target_q-seed).tolist())
        delta=np.abs(target_q-seed)
        if np.any(delta>max_delta):
            raise PolicyMoveJPrecheckError([f"JOINT_STEP_CANDIDATE_EXCEEDED:J{i+1}"
                for i in np.flatnonzero(delta>max_delta)],diagnostic)
        sigmas=[];tool_z=[];path_reasons=[]
        samples=int(self.rc["policy_movej"]["joint_path_samples_candidate"])
        if samples<2:raise ValueError("joint_path_samples_candidate must be >= 2")
        for index,alpha in enumerate(np.linspace(0.0,1.0,samples)):
            q=seed+alpha*(target_q-seed)
            path_reasons.extend(f"PATH_{index}:{reason}" for reason in joint_margin_reasons(q,self.cc))
            sigma=float(np.linalg.svd(self.kin.jacobian(q),compute_uv=False)[-1]);sigmas.append(sigma)
            if sigma<self.sigma_min:path_reasons.append(f"PATH_{index}:SINGULARITY_APPROACH")
            z=float(self.tool.link_to_tool(self.kin.fk(q)).position_m[2]);tool_z.append(z)
            descent_reason=descent_limit_reason(self.home_tool_z_m,z,self.max_descent_mm,index)
            if descent_reason:path_reasons.append(descent_reason)
        if path_reasons:
            diagnostic.update(path_minimum_singular_value=min(sigmas),
                              path_minimum_tool_z_m=min(tool_z))
            raise PolicyMoveJPrecheckError(path_reasons,diagnostic)
        return MoveJPlan(target_q,target_tool,target_link,tuple(sigmas),min(tool_z),
                         joint_envelope_status(target_q,self.cc))


class SettlementTracker:
    def __init__(self,velocity_rad_s=.001,hold_s=.3,target_error_deg=.1):
        self.velocity=float(velocity_rad_s);self.hold=float(hold_s)
        self.error=np.deg2rad(float(target_error_deg));self.zero_since=None
    def update(self,joint_rad,velocity_rad_s,target_joint_rad,now=None):
        now=time.monotonic() if now is None else float(now)
        speed_ok=bool(np.all(np.abs(velocity_rad_s)<=self.velocity))
        target_ok=bool(np.all(np.abs(np.asarray(joint_rad)-target_joint_rad)<=self.error))
        if speed_ok and target_ok:
            self.zero_since=now if self.zero_since is None else self.zero_since
            return now-self.zero_since>=self.hold
        self.zero_since=None;return False


class MockableMoveJStep:
    """Small command boundary used to prove rejection and abort semantics."""
    def __init__(self,planner,move_transport,stop_callback,move_config):
        self.planner=planner;self.move=move_transport;self.stop=stop_callback;self.cfg=move_config
    def start(self,*plan_args):
        plan=self.planner.plan(*plan_args)  # failure occurs before any transport call
        self.move.move_joint(pos=np.rad2deg(plan.target_joint_rad).tolist(),
            vel=self.cfg["velocity_deg_s_candidate"],
            acc=self.cfg["acceleration_deg_s2_confirmed"],sync_type=1)
        return plan
    def operator_abort(self):
        return self.stop("OPERATOR_ABORT")
