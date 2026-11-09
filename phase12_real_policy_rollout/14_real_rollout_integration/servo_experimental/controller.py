"""Clock-driven task-space tracking, deliberately without ROS/publish capability."""
import math
import numpy as np
from scipy.spatial.transform import Rotation


def capped(vector, maximum):
    vector = np.asarray(vector, dtype=float)
    norm = np.linalg.norm(vector)
    return vector if norm <= maximum else vector * maximum / norm


def continuous_zyz(rotation, previous):
    abc = rotation.as_euler('ZYZ', degrees=True)
    alternatives = (abc, np.array([abc[0]+180, -abc[1], abc[2]+180]))
    candidates = [v + 360*np.round((previous-v)/360) for v in alternatives]
    return min(candidates, key=lambda v: np.linalg.norm(v-previous))


class ServoShadowController:
    """Latest absolute target; actual dt dynamics; world-frame SO(3) rotation.

    This is NOT a validated physical ServoL controller. A topic publish has no
    vendor ACK, and loss-of-publisher stopping behavior remains unverified.
    """
    def __init__(self, pose_mm_zyz, limits, *, rate_hz=20., transport=None):
        if rate_hz != 20.:
            raise ValueError('experimental_rate_is_20hz')
        pose = np.asarray(pose_mm_zyz, dtype=float)
        if pose.shape != (6,) or not np.all(np.isfinite(pose)):
            raise ValueError('invalid_initial_pose')
        self.p = pose[:3].copy()
        self.r = Rotation.from_euler('ZYZ', pose[3:], degrees=True)
        self.abc = pose[3:].copy()
        self.v = np.zeros(3); self.w = np.zeros(3)
        self.tv = float(limits['translation_velocity_mm_s'])
        self.ta = float(limits['translation_acceleration_mm_s2'])
        self.rv = math.radians(limits['rotation_velocity_deg_s'])
        self.ra = math.radians(limits['rotation_acceleration_deg_s2'])
        self.age_limit = float(limits['max_action_age_sec'])
        self.workspace = limits['workspace_m']
        self.period = 1/rate_hz
        if transport is None:
            transport = dict(velocity_mm_s_deg_s=[self.tv,math.degrees(self.rv)],
                acceleration_mm_s2_deg_s2=[self.ta,math.degrees(self.ra)],target_time_s=2*self.period)
        self.transport_vel = np.asarray(transport['velocity_mm_s_deg_s'],dtype=float)
        self.transport_acc = np.asarray(transport['acceleration_mm_s2_deg_s2'],dtype=float)
        self.target_time = float(transport['target_time_s'])
        if (self.transport_vel.shape!=(2,) or self.transport_acc.shape!=(2,) or
                not np.all(np.isfinite(np.r_[self.transport_vel,self.transport_acc,self.target_time])) or
                np.any(self.transport_vel<=0) or np.any(self.transport_acc<=0) or self.target_time<=0):
            raise ValueError('invalid_servo_transport_profile')
        if (np.any(self.transport_vel+1e-12<np.array([self.tv,math.degrees(self.rv)])) or
                np.any(self.transport_acc+1e-12<np.array([self.ta,math.degrees(self.ra)]))):
            raise ValueError('transport_bounds_below_planning_limits')
        self.target = None; self.source = None; self.last_tick = None
        self.aborted = False; self.reason = None; self.stop_requests = 0
        self._workspace(self.p)

    def _workspace(self, position):
        if any(not self.workspace[k][0] <= position[i]/1000 <= self.workspace[k][1]
               for i,k in enumerate(('x','y','z'))):
            raise ValueError('workspace_rejection')

    def abort(self, reason):
        if not self.aborted:
            self.aborted = True; self.reason = reason; self.stop_requests += 1

    def pose(self):
        return np.r_[self.p, self.abc].tolist()

    def set_target(self, target_mm_zyz, source_monotonic, now):
        if self.aborted:
            raise RuntimeError('servo_inhibited')
        target = np.asarray(target_mm_zyz, dtype=float)
        if target.shape != (6,) or not np.all(np.isfinite(target)):
            self.abort('invalid_target'); raise ValueError('invalid_target')
        if not math.isfinite(source_monotonic) or not 0 <= now-source_monotonic <= self.age_limit:
            self.abort('stale_action'); raise ValueError('stale_action')
        try:self._workspace(target[:3])
        except ValueError:
            self.abort('workspace_rejection'); raise
        self.target = (target[:3].copy(),Rotation.from_euler('ZYZ',target[3:],degrees=True))
        self.source = source_monotonic  # Never reset age on periodic resampling.

    @staticmethod
    def _desired(error, speed, acceleration):
        distance = np.linalg.norm(error)
        if distance == 0:return np.zeros(3)
        return error/distance*min(speed, math.sqrt(2*acceleration*distance))

    def tick(self, now, *, health_ok=True):
        if self.aborted:raise RuntimeError('servo_inhibited:'+str(self.reason))
        if not health_ok:
            self.abort('sensor_fault'); raise RuntimeError('sensor_fault')
        if not math.isfinite(now):
            self.abort('invalid_clock'); raise ValueError('invalid_clock')
        dt = self.period if self.last_tick is None else now-self.last_tick
        if not 0 < dt <= .1:
            self.abort('servo_tick_gap'); raise RuntimeError('servo_tick_gap')
        self.last_tick = now
        if self.target is None:return None
        if not 0 <= now-self.source <= self.age_limit:
            self.abort('stale_action'); raise RuntimeError('stale_action')
        p_target,r_target = self.target
        desired_v = self._desired(p_target-self.p,self.tv,self.ta)
        desired_w = self._desired((r_target*self.r.inv()).as_rotvec(),self.rv,self.ra)
        v = self.v+capped(desired_v-self.v,self.ta*dt)
        w = self.w+capped(desired_w-self.w,self.ra*dt)
        p = self.p+(self.v+v)*(.5*dt)
        try:self._workspace(p)
        except ValueError:
            self.abort('workspace_rejection'); raise
        r = Rotation.from_rotvec((self.w+w)*(.5*dt))*self.r
        abc = continuous_zyz(r,self.abc)
        self.p,self.r,self.abc,self.v,self.w = p,r,abc,v,w
        return dict(topic_candidate='/dsr01/servol_stream',message_type='dsr_msgs2/msg/ServolStream',
                    pos=self.pose(),vel=self.transport_vel.tolist(),
                    acc=self.transport_acc.tolist(),time=self.target_time,
                    planning_velocity_mm_s_deg_s=[self.tv,math.degrees(self.rv)],
                    planning_acceleration_mm_s2_deg_s2=[self.ta,math.degrees(self.ra)],
                    source_monotonic=self.source,action_age_s=now-self.source,
                    translation_velocity_mm_s=v.tolist(),rotation_velocity_rad_s=w.tolist(),
                    actual_tick_dt_s=dt,command_issued=False,ack=None,
                    target_time_contract='EXPERIMENTAL_NOT_PHYSICALLY_VERIFIED')
