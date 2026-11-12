"""Command-free URDF differential kinematics and fail-closed dispatch boundary."""
import json
import math
from pathlib import Path
import sys
import threading
import time
import numpy as np
import yaml
from scipy.spatial.transform import Rotation

sys.path.insert(0, str(Path(__file__).resolve().parents[2]/'03_shadow_mode'))
from jointstate_flange_fk import A0509FlangeFK, CANONICAL_JOINTS, _origin


def ordered_joint_state(row):
    names = row.get('names', row.get('name', []))
    q = row.get('position', [])
    if (len(names) != 6 or len(q) != 6 or set(names) != set(CANONICAL_JOINTS)
            or row.get('valid') is False):
        raise ValueError('invalid_joint_names_or_count')
    q = np.array([dict(zip(names, q))[n] for n in CANONICAL_JOINTS], dtype=float)
    if not np.all(np.isfinite(q)):
        raise ValueError('nonfinite_joints')
    return q


class A0509Kinematics:
    def __init__(self, urdf):
        self.fk = A0509FlangeFK(urdf)

    def matrix_and_jacobian(self, q):
        q = np.asarray(q, dtype=float)
        if q.shape != (6,) or not np.all(np.isfinite(q)):
            raise ValueError('invalid_kinematic_joints')
        t = np.eye(4); origins = []; axes = []
        for joint, angle in zip(self.fk.chain, q):
            t = t @ np.array(_origin(joint.xyz, joint.rpy))
            axis = np.array(joint.axis, dtype=float)
            axis /= np.linalg.norm(axis)
            origins.append(t[:3, 3].copy()); axes.append(t[:3, :3] @ axis)
            r = np.eye(4); r[:3, :3] = Rotation.from_rotvec(axis*angle).as_matrix()
            t = t @ r
        j = np.column_stack([np.r_[np.cross(a, t[:3, 3]-o), a] for a, o in zip(axes, origins)])
        return t, j


class JointRangePredictor:
    """Constant candidate BASE twist rollout; no angle wrapping, no controller IK.

    Reject singular/nonfinite/unreachable local solves rather than damping away
    a dangerous joint velocity. Full target IK is additionally checked if given.
    Neither test proves future vendor IK branch or braking-envelope safety.
    """
    def __init__(self, urdf, config):
        self.model = A0509Kinematics(urdf)
        self.config = dict(config)
        self.lower = np.deg2rad(np.asarray(config['lower_deg'], dtype=float))
        self.upper = np.deg2rad(np.asarray(config['upper_deg'], dtype=float))
        self.speed = np.deg2rad(np.asarray(config['max_joint_speed_deg_s'], dtype=float))
        if any(a.shape != (6,) or not np.all(np.isfinite(a)) for a in [self.lower, self.upper, self.speed]):
            raise ValueError('invalid_joint_contract')
        if np.any(self.lower >= self.upper) or np.any(self.speed <= 0):
            raise ValueError('invalid_joint_contract')
        for name in ['lookahead_s', 'integration_step_s', 'jointstate_max_age_s',
                     'scaled_jacobian_min_singular_value', 'jacobian_characteristic_length_m']:
            value = config[name]
            if type(value) not in (int, float) or not math.isfinite(value) or value <= 0:
                raise ValueError('invalid_'+name)
        if config['lookahead_s'] > 2 or config['integration_step_s'] > config['lookahead_s']:
            raise ValueError('unbounded_prediction_horizon')

    def _bounds(self, q):
        bad = np.flatnonzero((q < self.lower) | (q > self.upper))
        if len(bad):
            raise ValueError('joint_limit_predicted:'+','.join(CANONICAL_JOINTS[i] for i in bad))

    def _solve(self, q, twist):
        _, j = self.model.matrix_and_jacobian(q)
        scale = np.r_[np.full(3, 1/self.config['jacobian_characteristic_length_m']), np.ones(3)]
        s = np.linalg.svd(j*scale[:, None], compute_uv=False)
        if s[-1] < self.config['scaled_jacobian_min_singular_value']:
            raise ValueError('jacobian_singular_or_ill_conditioned')
        return np.linalg.solve(j, twist), float(s[-1])

    def assess(self, row, velocity_mm_deg_s, *, now, target_pose=None):
        path = []; ik_path = []; min_s = None; stage = 'input_validation'
        result = dict(allowed=False, physical_dispatch_allowed=False,
            prediction_kind='LOCAL_URDF_DIFFERENTIAL_ESTIMATE_NOT_VENDOR_IK',
            joint_limits_changed=False, controller_limits_verified=self.config.get('controller_limits_verified') is True)
        try:
            if row.get('valid') is not True:raise ValueError('jointstate_validity_unverified')
            q = ordered_joint_state(row); path.append(q.copy())
            stage = 'measured_joint_bounds'; self._bounds(q)
            stage = 'feedback_and_velocity_validation'
            receive = float(row['receive'])
            if not math.isfinite(now) or not math.isfinite(receive) or not 0 <= now-receive < self.config['jointstate_max_age_s']:
                raise ValueError('jointstate_stale_or_clock_regression')
            for key in ('source_gap', 'receive_gap'):
                if key in row and (not math.isfinite(row[key]) or not 0 < row[key] < .1):
                    raise ValueError('jointstate_gap_or_regression:'+key)
            vel = np.asarray(velocity_mm_deg_s, dtype=float)
            if vel.shape != (6,) or not np.all(np.isfinite(vel)):
                raise ValueError('invalid_speedl_velocity')
            twist = np.r_[vel[:3]/1000, np.deg2rad(vel[3:])]
            duration = self.config['lookahead_s'] + now-receive
            count = int(math.ceil(duration/self.config['integration_step_s']))
            dt = duration/count
            stage = 'velocity_horizon_prediction'
            for _ in range(count):
                dq, s = self._solve(q, twist); min_s = s if min_s is None else min(min_s, s)
                if np.any(abs(dq) > self.speed):
                    raise ValueError('predicted_joint_speed_limit')
                q = q + dt*dq; path.append(q.copy()); self._bounds(q)
            if target_pose is not None:
                stage = 'seeded_target_ik'
                pose = np.asarray(target_pose, dtype=float)
                if pose.shape != (6,) or not np.all(np.isfinite(pose)):
                    raise ValueError('invalid_ik_target')
                target_r = Rotation.from_euler('ZYZ', pose[3:], degrees=True)
                qi = ordered_joint_state(row)
                for _ in range(80):
                    t, _ = self.model.matrix_and_jacobian(qi)
                    error = np.r_[pose[:3]/1000-t[:3, 3], (target_r*Rotation.from_matrix(t[:3, :3]).inv()).as_rotvec()]
                    if np.linalg.norm(error[:3]) < 1e-5 and np.linalg.norm(error[3:]) < 1e-4:
                        break
                    dq, s = self._solve(qi, error)
                    dq *= min(1, .05/max(np.linalg.norm(dq), 1e-15))
                    qi = qi+dq; ik_path.append(qi.copy()); self._bounds(qi)
                else:
                    raise ValueError('local_ik_did_not_converge')
                result['target_ik_joint_deg'] = np.rad2deg(qi).tolist()
            result.update(allowed=True, reason='NOMINAL_GEOMETRIC_CHECK_PASS_NOT_LIVE_APPROVAL')
        except (ValueError, KeyError, TypeError, np.linalg.LinAlgError) as exc:
            result['reason'] = str(exc)
            result['rejection_stage'] = stage
        if path:
            a = np.rad2deg(np.array(path))
            result.update(predicted_min_deg=a.min(0).tolist(), predicted_max_deg=a.max(0).tolist(),
                          path_joint_deg=a.tolist(), min_scaled_jacobian_singular_value=min_s)
        if ik_path:result['ik_path_joint_deg'] = np.rad2deg(ik_path).tolist()
        blockers = [k for k in ('controller_limits_verified', 'physical_speedl_contract_verified',
                                'stop_latency_bound_verified') if self.config.get(k) is not True]
        if row.get('phase') != 'RUNTIME_READY':blockers.append('JOINTSTATE_NOT_RUNTIME_READY')
        # Nominal forecast alone never certifies controller IK or stopping.
        result['live_blockers'] = blockers
        result['physical_dispatch_allowed'] = result['allowed'] and not blockers
        return result


class ProtectedSpeedLBoundary:
    """Validate, persist, then call supplied transport under one inhibit lock.

    No transport is created here. Tests supply lists only; physical code must
    additionally establish the verified controller/frame/braking contracts.
    """
    def __init__(self, predictor, logger, send, request_stop, *, clock=time.monotonic, task_limits=None):
        self.predictor = predictor; self.logger = logger; self.send = send
        self.request_stop = request_stop; self.lock = threading.RLock()
        self.closed = False; self.reason = None; self.commands = 0
        self.stop_requested = False; self.stop_evidence = None
        self.clock = clock
        self.events = []
        self.task_limits = task_limits or yaml.safe_load((Path(__file__).resolve().parents[2]/'01_configs/safety_limits.yaml').read_text())

    def _log(self, row):
        json.dumps(row, allow_nan=False)
        self.logger(row)

    def submit(self, jointstate, candidate, *, now, target_pose=None):
        with self.lock:
            if self.closed:raise RuntimeError('speedl_dispatch_inhibited:'+str(self.reason))
            try:
                check_started = self.clock()
                vel = np.asarray(candidate['vel'], dtype=float)
                acc = np.asarray(candidate['acc'], dtype=float)
                horizon = float(candidate['time'])
                max_vel = [self.task_limits['translation_velocity_mm_s'], self.task_limits['rotation_velocity_deg_s']]
                max_acc = np.array([self.task_limits['translation_acceleration_mm_s2'], self.task_limits['rotation_acceleration_deg_s2']])
                if (vel.shape != (6,) or acc.shape != (2,) or not np.all(np.isfinite(np.r_[vel,acc,horizon]))
                        or np.any(acc <= 0) or np.any(acc > max_acc) or not 0 < horizon <= .1
                        or np.linalg.norm(vel[:3]) > max_vel[0]+1e-12 or np.linalg.norm(vel[3:]) > max_vel[1]+1e-12):
                    raise ValueError('invalid_or_excessive_speedl_transport')
                age = now-candidate['source_monotonic']
                if not math.isfinite(age) or not 0 <= age <= self.task_limits['max_action_age_sec']:
                    raise ValueError('stale_speedl_source')
                proof = self.predictor.assess(jointstate, candidate['vel'], now=now, target_pose=target_pose)
                self._log(dict(event='SPEEDL_JOINT_PREFLIGHT', assessment=proof, command_issued=False))
                if not proof['physical_dispatch_allowed']:
                    raise ValueError(proof['reason'] if not proof['allowed'] else 'live_contract_unverified')
                elapsed = self.clock()-check_started
                if not math.isfinite(elapsed) or not 0 <= elapsed < .1:
                    raise ValueError('speedl_pre_dispatch_stall')
                dispatch_now = now+elapsed
                if dispatch_now-candidate['source_monotonic'] > self.task_limits['max_action_age_sec']:
                    raise ValueError('stale_speedl_source_after_logging')
                proof = self.predictor.assess(jointstate, candidate['vel'], now=dispatch_now, target_pose=target_pose)
                if not proof['physical_dispatch_allowed']:
                    raise ValueError('pre_dispatch_recheck:'+proof['reason'])
                self.commands += 1  # Count attempted delivery even if send raises.
                self.send(candidate)
                return proof
            except Exception as exc:
                self.events.append(dict(event='SPEEDL_DISPATCH_INHIBITED', reason=str(exc), additional_commands=0))
                self.abort(str(exc)); raise

    def abort(self, reason):
        with self.lock:
            self.closed = True
            if self.reason is None:self.reason = reason
            if self.commands and not self.stop_requested:
                self.stop_requested = True; self.request_stop()

    def confirm_stop(self, evidence):
        with self.lock:
            if not self.closed:raise RuntimeError('stop_not_requested')
            self.stop_evidence = dict(evidence)
            # Even a successful stop never implicitly reopens this trial.
            if evidence.get('confirmed') is not True:
                self.reason = self.reason or 'stop_confirmation_failed'
