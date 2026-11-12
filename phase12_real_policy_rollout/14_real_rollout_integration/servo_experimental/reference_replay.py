"""Offline-only reference alternatives and fail-closed stream checks.

No ROS or hardware transport. Guard tolerances must be explicitly supplied;
this module does not choose or enable physical safety thresholds.
"""
from dataclasses import dataclass
import math
import numpy as np
from scipy.spatial.transform import Rotation
from servo_experimental.controller import continuous_zyz


def rotation(abc):
    return Rotation.from_euler('ZYZ', abc, degrees=True)


def pose_distance(a,b):
    a,b=np.asarray(a,dtype=float),np.asarray(b,dtype=float)
    if a.shape!=(6,) or b.shape!=(6,) or not np.all(np.isfinite(np.r_[a,b])):
        raise ValueError('invalid_pose')
    return float(np.linalg.norm(a[:3]-b[:3])),float((rotation(a[3:]).inv()*rotation(b[3:])).magnitude()*180/math.pi)


def delta_target(anchor,limited_action):
    anchor=np.asarray(anchor,dtype=float);action=np.asarray(limited_action,dtype=float)
    if anchor.shape!=(6,) or action.shape!=(7,) or not np.all(np.isfinite(np.r_[anchor,action])):
        raise ValueError('invalid_anchor_or_action')
    target_rotation=Rotation.from_rotvec(action[3:6])*rotation(anchor[3:])
    return np.r_[anchor[:3]+1000*action[:3],continuous_zyz(target_rotation,anchor[3:])].tolist()


def install_reference(tracker,action,actual_pose,mode):
    """Proposal A keeps commanded state; B rebases onto recorded actual pose.

    B reset is deliberately exposed rather than hidden: it can create an
    unsafe discontinuity. The guarded replay must reject that discontinuity.
    Neither alternative is connected to a live launcher.
    """
    if mode=='command':
        anchor=tracker.pose()
    elif mode=='actual':
        anchor=list(actual_pose)
        previous_abc=tracker.abc.copy()
        tracker.p=np.array(anchor[:3],dtype=float)
        tracker.r=rotation(anchor[3:])
        tracker.abc=continuous_zyz(tracker.r,previous_abc)
        tracker.v=np.zeros(3);tracker.w=np.zeros(3)
    else:
        raise ValueError('unknown_reference_mode')
    return anchor,delta_target(anchor,action)


@dataclass(frozen=True)
class DiagnosticGuardLimits:
    budget_translation_tolerance_mm: float
    budget_rotation_tolerance_deg: float
    following_translation_mm: float
    following_rotation_deg: float
    feedback_max_age_s: float

    def __post_init__(self):
        for value in vars(self).values():
            if not math.isfinite(value) or value<0:
                raise ValueError('invalid_guard_tolerance')
        if min(self.following_translation_mm,self.following_rotation_deg,self.feedback_max_age_s)<=0:
            raise ValueError('invalid_guard_tolerance')


class OfflineStreamBoundary:
    """Fake callback boundary: validate before publish; stop once after fault.

    Feedback is compared with command history at its receive time, not a future
    candidate. Receive time does not prove sensor acquisition/vendor use time.
    Segment budget checks both net excursion and cumulative arc length so
    overshoot/oscillation cannot cancel out in the endpoint norm.
    """
    def __init__(self,initial_pose,limits,publish,stop,*,check_action_budget=True):
        self.limits=limits;self.publish=publish;self.stop=stop
        self.initial=list(initial_pose);self.last=list(initial_pose)
        self.history=[];self.closed=False;self.reason=None;self.stop_requests=0
        self.published=0;self.events=[];self.segment=None
        if type(check_action_budget) is not bool:raise ValueError('invalid_budget_check')
        # False is exclusively for isolating the following check in offline
        # experiments. Normal protected replay enables BOTH checks.
        self.check_action_budget=check_action_budget

    def fail(self,reason):
        if not self.closed:
            self.closed=True;self.reason=reason
            if self.published:
                self.stop_requests+=1;self.stop()
        raise RuntimeError(self.reason)

    def begin_segment(self,key,limited_action):
        if self.closed:raise RuntimeError('dispatch_inhibited:'+str(self.reason))
        action=np.asarray(limited_action,dtype=float)
        if action.shape!=(7,) or not np.all(np.isfinite(action)):
            self.fail('invalid_limited_action')
        self.segment=dict(key=key,start=list(self.last),
            translation_budget_mm=float(np.linalg.norm(action[:3])*1000),
            rotation_budget_deg=float(Rotation.from_rotvec(action[3:6]).magnitude()*180/math.pi),
            translation_arc_mm=0.,rotation_arc_deg=0.)

    def inspect_and_publish(self,pose,now,feedback,feedback_received):
        if self.closed:raise RuntimeError('dispatch_inhibited:'+str(self.reason))
        if self.segment is None:self.fail('missing_segment')
        if not math.isfinite(now) or not math.isfinite(feedback_received) or not 0<=now-feedback_received<self.limits.feedback_max_age_s:
            self.fail('stale_or_future_feedback')
        if self.history and now<=self.history[-1][0]:self.fail('stream_time_regression')
        reference=self.initial
        for received,command in reversed(self.history):
            if received<=feedback_received:
                reference=command;break
        try:
            following_t,following_r=pose_distance(reference,feedback)
            net_t,net_r=pose_distance(self.segment['start'],pose)
            tick_t,tick_r=pose_distance(self.last,pose)
        except (ValueError,TypeError):self.fail('invalid_pose')
        arc_t=self.segment['translation_arc_mm']+tick_t
        arc_r=self.segment['rotation_arc_deg']+tick_r
        event=dict(key=self.segment['key'],now=now,following_translation_mm=following_t,
            following_rotation_deg=following_r,net_translation_mm=net_t,net_rotation_deg=net_r,
            arc_translation_mm=arc_t,arc_rotation_deg=arc_r,
            translation_budget_mm=self.segment['translation_budget_mm'],rotation_budget_deg=self.segment['rotation_budget_deg'])
        self.events.append(event)
        if following_t>self.limits.following_translation_mm or following_r>self.limits.following_rotation_deg:
            self.fail('following_error_limit')
        if self.check_action_budget and max(net_t,arc_t)>self.segment['translation_budget_mm']+self.limits.budget_translation_tolerance_mm:
            self.fail('stream_translation_action_budget')
        if self.check_action_budget and max(net_r,arc_r)>self.segment['rotation_budget_deg']+self.limits.budget_rotation_tolerance_deg:
            self.fail('stream_rotation_action_budget')
        try:self.publish(pose)
        except Exception:
            # Delivery may be unknown even if callback raises.
            self.published+=1;self.fail('publish_exception_delivery_unknown')
        self.published+=1;self.last=list(pose);self.history.append((now,list(pose)))
        self.segment['translation_arc_mm']=arc_t;self.segment['rotation_arc_deg']=arc_r
        return event
