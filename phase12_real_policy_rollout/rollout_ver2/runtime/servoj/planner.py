from __future__ import annotations
from dataclasses import dataclass
import time
import numpy as np
from scipy.spatial.transform import Rotation,Slerp
from controller.geometry import apply_action
from controller.precheck import joint_margin_reasons
from controller.trajectory import minimum_jerk
from kinematics import DLSIKSolver,Pose


class ServoJPrecheckError(RuntimeError):
    def __init__(self,reasons,diagnostics=None):
        self.reasons=tuple(dict.fromkeys(reasons));self.diagnostics=dict(diagnostics or {})
        super().__init__("SERVOJ_PRECHECK:"+",".join(self.reasons))


@dataclass(frozen=True)
class ServoJSegment:
    duration_s:float;times_s:np.ndarray;tool_poses:tuple;link_poses:tuple
    joints_rad:np.ndarray;velocity_rad_s:np.ndarray;acceleration_rad_s2:np.ndarray
    sigma_min:np.ndarray;ik_sample_s:np.ndarray;precompute_s:float


class FixedSegmentPlanner:
    """Precompute and validate one fixed-duration mode-A segment in full."""
    def __init__(self,kin,tool,config,*,rate_hz=100.0,sigma_min=None):
        self.kin=kin;self.tool=tool;self.config=config;self.rate_hz=float(rate_hz)
        self.sigma_min=float(config.jacobian_min_singular_value if sigma_min is None else sigma_min)
        # Tight primary solve prevents tolerance-sized joint chatter from being
        # misclassified as 100 Hz acceleration.  The configured candidate is
        # retained only as a fallback acceptance tolerance.
        self.ik=DLSIKSolver(kin,damping=1e-3,max_iterations=250,
          position_tolerance_m=1e-8,rotation_tolerance_rad=1e-8)
        self.fallback_ik=DLSIKSolver(kin,damping=1e-3,max_iterations=250,
          position_tolerance_m=config.ik_position_tolerance_m,
          rotation_tolerance_rad=config.ik_rotation_tolerance_rad)
    def target(self,start_tool,delta_position_m,rotation_vector_rad):
        values=np.r_[delta_position_m,rotation_vector_rad]
        if values.shape!=(6,) or not np.all(np.isfinite(values)):raise ServoJPrecheckError(["NONFINITE_ACTION"])
        return apply_action(start_tool,delta_position_m,rotation_vector_rad,"world_left")
    def plan_action(self,start_tool,delta_position_m,rotation_vector_rad,seed_joint_rad,duration_s):
        return self.plan_to_pose(start_tool,self.target(start_tool,delta_position_m,rotation_vector_rad),
                                 seed_joint_rad,duration_s)
    def plan_to_pose(self,start_tool,target_tool,seed_joint_rad,duration_s):
        started=time.perf_counter();duration=float(duration_s)
        if duration<=0 or not np.isfinite(duration):raise ValueError("duration must be finite and positive")
        count=int(round(duration*self.rate_hz))+1;times=np.linspace(0,duration,count)
        s=minimum_jerk(times/duration)
        rotations=Slerp([0.,1.],Rotation.from_matrix(np.stack(
          [start_tool.rotation,target_tool.rotation])))(s).as_matrix()
        positions=start_tool.position_m+s[:,None]*(target_tool.position_m-start_tool.position_m)
        if np.any(positions<self.config.workspace_min_m) or np.any(positions>self.config.workspace_max_m):
            raise ServoJPrecheckError(["WORKSPACE_CANDIDATE_EXCEEDED"])
        tools=tuple(Pose(p,r) for p,r in zip(positions,rotations));links=tuple(
          self.tool.tool_to_link(pose) for pose in tools)
        q=np.asarray(seed_joint_rad,dtype=float).copy();joints=[];ik_times=[];sigmas=[];reasons=[]
        for index,pose in enumerate(links):
            tick=time.perf_counter()
            try:q=self.ik.solve(pose,q,joint_lower_rad=self.config.effective_lower_rad,
              joint_upper_rad=self.config.effective_upper_rad,
              max_joint_change_rad=self.config.max_sample_joint_change_rad)
            except Exception as error:
                try:q=self.fallback_ik.solve(pose,q,joint_lower_rad=self.config.effective_lower_rad,
                  joint_upper_rad=self.config.effective_upper_rad,
                  max_joint_change_rad=self.config.max_sample_joint_change_rad)
                except Exception as fallback_error:
                    diagnostics={"sample":index,"error":str(error),
                      "fallback_error":str(fallback_error),"fallback_error_type":type(fallback_error).__name__}
                    best=getattr(fallback_error,"best_joint_rad",None)
                    if best is not None:
                        diagnostics["best_joint_rad"]=np.asarray(best).tolist()
                        diagnostics["best_joint_deg"]=np.rad2deg(best).tolist()
                    if getattr(fallback_error,"best_residual",None) is not None:
                        diagnostics["best_residual"]=fallback_error.best_residual
                    if getattr(fallback_error,"iterations",None) is not None:
                        diagnostics["iterations"]=fallback_error.iterations
                    raise ServoJPrecheckError([f"IK_FAILED:{index}"],diagnostics) from fallback_error
            ik_times.append(time.perf_counter()-tick);joints.append(q.copy())
            reasons.extend(f"SAMPLE_{index}:{x}" for x in joint_margin_reasons(q,self.config))
            sigma=float(np.linalg.svd(self.kin.jacobian(q),compute_uv=False)[-1]);sigmas.append(sigma)
            if sigma<self.sigma_min:reasons.append(f"SAMPLE_{index}:SINGULARITY_APPROACH")
        joints=np.asarray(joints);dt=np.diff(times);velocity=np.diff(joints,axis=0)/dt[:,None]
        acceleration=np.diff(velocity,axis=0)/(((dt[:-1]+dt[1:])/2)[:,None])
        sample_delta=np.abs(np.diff(joints,axis=0))
        for joint in np.flatnonzero(np.any(sample_delta>self.config.max_sample_joint_change_rad,axis=0)):
            reasons.append(f"SAMPLE_JOINT_DELTA:J{joint+1}")
        max_v=np.max(np.abs(velocity),axis=0);max_a=(np.max(np.abs(acceleration),axis=0)
          if len(acceleration) else np.zeros(6))
        for joint in np.flatnonzero(max_v>self.config.max_joint_velocity_rad_s):reasons.append(f"JOINT_VELOCITY:J{joint+1}")
        for joint in np.flatnonzero(max_a>self.config.max_joint_acceleration_rad_s2):reasons.append(f"JOINT_ACCELERATION:J{joint+1}")
        diagnostics={"max_velocity_rad_s":max_v.tolist(),"max_acceleration_rad_s2":max_a.tolist(),
          "minimum_sigma":float(min(sigmas)),"precompute_s":time.perf_counter()-started}
        if reasons:raise ServoJPrecheckError(reasons,diagnostics)
        return ServoJSegment(duration,times,tools,links,joints,velocity,acceleration,
          np.asarray(sigmas),np.asarray(ik_times),time.perf_counter()-started)
