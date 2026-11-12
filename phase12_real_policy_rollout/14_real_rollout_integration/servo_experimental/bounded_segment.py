"""Offline command-reference interpolation: no velocity integration or ROS."""
import itertools
import math
import numpy as np
from scipy.spatial.transform import Rotation
from servo_experimental.controller import continuous_zyz,interpolate_segment
from servo_experimental.reference_replay import delta_target,pose_distance


def normalized_zyz(rotation,previous):
    """Equivalent intrinsic-ZYZ representations entirely in [-180,180].

    Minimize numeric distance to the previous *normalized* publication. At a
    wrap, a large Euler numeric jump can be unavoidable; physical continuity is
    guaranteed by rotation reconstruction, not by pretending the jump is gone.
    """
    previous=np.asarray(previous,dtype=float)
    if previous.shape!=(3,) or not np.all(np.isfinite(previous)):
        raise ValueError('invalid_previous_abc')
    base=continuous_zyz(rotation,previous)
    alternatives=(base,np.array([base[0]+180,-base[1],base[2]+180]))
    candidates=[]
    for value in alternatives:
        wrapped=(value+180)%360-180
        choices=[[-180.,180.] if abs(abs(v)-180)<1e-10 else [float(v)] for v in wrapped]
        for candidate in itertools.product(*choices):
            candidate=np.asarray(candidate)
            error=(Rotation.from_euler('ZYZ',candidate,degrees=True).inv()*rotation).magnitude()
            if error<=1e-6:candidates.append(candidate)
    if not candidates:raise ValueError('normalized_zyz_reconstruction_failure')
    return min(candidates,key=lambda c:np.linalg.norm(c-previous))


class CommandSegmentInterpolator:
    """Clamped linear position + Slerp over a causal inference-period estimate.

    Starts at the last generated command; that command must have been published
    successfully before accepting another target. Caller retains publish/fault
    serialization. No hardware profile, safety threshold or source age rebasing.
    Piecewise-linear time parameterization is NOT acceleration-continuous.
    """
    def __init__(self,initial_pose,limits):
        self.limits=limits
        self.last=np.asarray(initial_pose,dtype=float).copy()
        if self.last.shape!=(6,) or not np.all(np.isfinite(self.last)):
            raise ValueError('invalid_initial_pose')
        self.last[3:]=normalized_zyz(Rotation.from_euler('ZYZ',self.last[3:],degrees=True),self.last[3:])
        self.segment=None;self.closed=False;self.last_tick=None;self.last_activation=None
        self._workspace(self.last[:3])

    def _workspace(self,p):
        for i,key in enumerate(('x','y','z')):
            lo,hi=self.limits['workspace_m'][key]
            if not lo<=p[i]/1000<=hi:raise ValueError('workspace_rejection')

    def set_action(self,limited_action,source,now,period=None):
        if self.closed:raise RuntimeError('segment_inhibited')
        if not math.isfinite(source) or not 0<=now-source<=self.limits['max_action_age_sec']:
            self.closed=True;raise ValueError('stale_action')
        # Previous completed activation interval: no future inference time used.
        duration=(1/self.limits['control_rate_hz'] if self.last_activation is None
                  else now-self.last_activation) if period is None else float(period)
        if not math.isfinite(duration) or duration<=0:
            self.closed=True;raise ValueError('invalid_inference_period')
        action=np.asarray(limited_action,dtype=float)
        try:target=np.asarray(delta_target(self.last,action))
        except (ValueError,TypeError):
            self.closed=True;raise
        try:self._workspace(target[:3])
        except ValueError:
            self.closed=True;raise
        self.segment=dict(start=self.last.copy(),target=target,source=source,activated=now,
            duration=duration,translation_budget=float(np.linalg.norm(action[:3])*1000),
            rotation_budget=float(Rotation.from_rotvec(action[3:6]).magnitude()*180/math.pi),
            arc_translation=0.,arc_rotation=0.,last_fraction=0.)
        self.last_activation=now

    def tick(self,now):
        if self.closed:raise RuntimeError('segment_inhibited')
        if self.segment is None:return None
        s=self.segment
        if not math.isfinite(now) or now<s['activated'] or (self.last_tick is not None and now<=self.last_tick):
            self.closed=True;raise ValueError('segment_clock_regression')
        if not 0<=now-s['source']<=self.limits['max_action_age_sec']:
            self.closed=True;raise ValueError('stale_action')
        if self.last_tick is not None and now-self.last_tick>.1+1e-12:
            self.closed=True;raise ValueError('servo_tick_gap')
        u=float(np.clip((now-s['activated'])/s['duration'],0,1))
        p,r=interpolate_segment(s['start'][:3],Rotation.from_euler('ZYZ',s['start'][3:],degrees=True),
            s['target'][:3],Rotation.from_euler('ZYZ',s['target'][3:],degrees=True),u)
        candidate=np.r_[p,normalized_zyz(r,self.last[3:])]
        self._workspace(p)
        dt,dr=pose_distance(self.last,candidate)
        arc_t=s['arc_translation']+dt;arc_r=s['arc_rotation']+dr
        if arc_t>s['translation_budget']+1e-6 or arc_r>s['rotation_budget']+1e-6:
            self.closed=True;raise RuntimeError('segment_action_budget')
        s.update(arc_translation=arc_t,arc_rotation=arc_r,last_fraction=u)
        self.last=candidate;self.last_tick=now
        return dict(pos=candidate.tolist(),fraction=u,duration=s['duration'],
            cumulative_translation_mm=arc_t,cumulative_rotation_deg=arc_r,
            source_monotonic=s['source'],command_issued=False)
