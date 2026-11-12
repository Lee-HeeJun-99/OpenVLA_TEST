"""Offline BASE twist candidate from measured pose error, not Euler-rate deltas.

No hardware or network capability. This is not a live SpeedL controller. Controller
joint limits, target IK, Jacobian feasibility and physical stopping are NOT verified
by this calculation. Existing production launchers deliberately do not import it.
"""
import math
import numpy as np
from scipy.spatial.transform import Rotation


def cap(vector, bound):
    norm = np.linalg.norm(vector)
    return vector if norm <= bound else vector * (bound / norm)


class SpeedLShadowController:
    def __init__(self, limits, *, response_time_s=0.7, rate_hz=20):
        if rate_hz != 20 or not math.isfinite(response_time_s) or response_time_s <= 0:
            raise ValueError('invalid_offline_speedl_profile')
        self.tau = response_time_s
        self.period = 1.0 / rate_hz
        self.speed = np.array([limits['translation_velocity_mm_s'],
                               limits['rotation_velocity_deg_s']], dtype=float)
        self.acceleration = np.array([limits['translation_acceleration_mm_s2'],
                                      limits['rotation_acceleration_deg_s2']], dtype=float)
        self.age_limit = float(limits['max_action_age_sec'])
        self.workspace = limits['workspace_m']
        if not np.all(np.isfinite(np.r_[self.speed, self.acceleration, self.age_limit])) or np.any(np.r_[self.speed, self.acceleration, self.age_limit] <= 0):
            raise ValueError('invalid_existing_limits')
        for key in ('x', 'y', 'z'):
            bounds = np.asarray(self.workspace[key], dtype=float)
            if bounds.shape != (2,) or not np.all(np.isfinite(bounds)) or bounds[0] >= bounds[1]:
                raise ValueError('invalid_workspace')
        self.velocity = np.zeros(6)
        self.previous_time = None
        self.aborted = False

    def _pose(self, pose):
        p = np.asarray(pose, dtype=float)
        if p.shape != (6,) or not np.all(np.isfinite(p)):
            raise ValueError('invalid_pose')
        if any(not self.workspace[key][0] <= p[i]/1000 <= self.workspace[key][1]
               for i, key in enumerate(('x', 'y', 'z'))):
            raise ValueError('workspace_rejection')
        return p

    def candidate(self, measured_pose_mm_zyz, target_pose_mm_zyz, *, now, source_time):
        if self.aborted:
            raise RuntimeError('speedl_shadow_inhibited')
        try:
            if not all(math.isfinite(v) for v in (now, source_time)) or not 0 <= now-source_time <= self.age_limit:
                raise ValueError('stale_action')
            dt = self.period if self.previous_time is None else now-self.previous_time
            if not 0 < dt < 0.1:
                raise ValueError('speedl_tick_gap')
            current = self._pose(measured_pose_mm_zyz)
            target = self._pose(target_pose_mm_zyz)
            relative = Rotation.from_euler('ZYZ', target[3:], degrees=True) * Rotation.from_euler('ZYZ', current[3:], degrees=True).inv()
            error = np.r_[target[:3]-current[:3], np.rad2deg(relative.as_rotvec())]
            desired = np.r_[cap(error[:3]/self.tau, self.speed[0]), cap(error[3:]/self.tau, self.speed[1])]
            velocity = self.velocity.copy()
            for part, acc in ((slice(0, 3), self.acceleration[0]), (slice(3, 6), self.acceleration[1])):
                velocity[part] += cap(desired[part]-velocity[part], acc*dt)
            # Test a one-period ideal projection as an extra offline check only.
            projected = current.copy(); projected[:3] += velocity[:3]*dt
            self._pose(projected)
            self.velocity = velocity
            self.previous_time = now
            return dict(classification='OFFLINE_SPEEDL_CANDIDATE_NOT_DISPATCHABLE',
                topic_candidate='/dsr01/speedl_stream', message_type='dsr_msgs2/msg/SpeedlStream',
                vel=velocity.tolist(), acc=self.acceleration.tolist(), time=2*self.period,
                linear_velocity_unit='mm/s', angular_velocity_unit='deg/s',
                orientation_error='BASE_SO3_ROTATION_VECTOR_NOT_ZYZ_COMPONENT_SUBTRACTION',
                response_time_s=self.tau, actual_tick_dt_s=dt,
                source_monotonic=source_time, action_age_s=now-source_time,
                command_requested=False, command_issued=False, vendor_ack=None,
                physical_rollout_ready=False,
                blockers=['CURRENT_CONTROLLER_JOINT_LIMITS_AND_IK_UNVERIFIED',
                          'PHYSICAL_SPEEDL_STOP_PATH_UNVERIFIED',
                          'SPEEDL_HARDWARE_FRAME_CONTRACT_NOT_LIVE_VALIDATED'])
        except Exception:
            self.abort()
            raise

    def abort(self):
        self.aborted = True
        self.velocity[:] = 0
        # This resets virtual state only. It sends NO physical zero/stop command.

    def protected_candidate(self, measured_pose_mm_zyz, target_pose_mm_zyz, *,
                            now, source_time, jointstate, joint_predictor):
        """Offline integration entry: attach actual joint range proof every tick.

        The physical caller must still use ProtectedSpeedLBoundary (which checks
        again immediately before dispatch); a shadow row is never authorization.
        """
        row = self.candidate(measured_pose_mm_zyz, target_pose_mm_zyz,
                             now=now, source_time=source_time)
        proof = joint_predictor.assess(jointstate, row['vel'], now=now,
                                       target_pose=target_pose_mm_zyz)
        row['joint_protection'] = proof
        row['physical_rollout_ready'] = False
        if not proof['allowed']:
            self.abort()
            row['inhibited_reason'] = proof['reason']
        return row
