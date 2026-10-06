"""Stateful command-free limiter for canonical 5 Hz delta actions."""
from __future__ import annotations
from dataclasses import dataclass
import math
from typing import Sequence

@dataclass(frozen=True)
class LimitedAction:
    raw_action: tuple[float,...]
    limited_action: tuple[float,...]
    translation_velocity_m_s: tuple[float,...]
    rotation_velocity_rad_s: tuple[float,...]
    translation_speed_clipped: bool
    rotation_speed_clipped: bool
    translation_acceleration_clipped: bool
    rotation_acceleration_clipped: bool
    translation_change_m: float
    rotation_change_rad: float
    command_issued: bool=False

def _norm(v): return math.sqrt(sum(float(x)*float(x) for x in v))
def _scale_to(v, limit):
    n=_norm(v)
    if n<=limit or n==0:return tuple(v),False
    return tuple(float(x)*limit/n for x in v),True
def _accel_limit(desired, previous, max_delta):
    dv=tuple(float(a)-float(b) for a,b in zip(desired,previous)); limited,clipped=_scale_to(dv,max_delta)
    return tuple(float(b)+float(d) for b,d in zip(previous,limited)),clipped

class CanonicalActionRateLimiter:
    """Limits speed then acceleration; first sample accelerates from zero."""
    def __init__(self, *, control_rate_hz:float=5.0,
                 max_translation_velocity_m_s:float=.02,
                 max_rotation_velocity_rad_s:float=math.radians(20),
                 max_translation_acceleration_m_s2:float=.02,
                 max_rotation_acceleration_rad_s2:float=math.radians(20)):
        vals=(control_rate_hz,max_translation_velocity_m_s,max_rotation_velocity_rad_s,
              max_translation_acceleration_m_s2,max_rotation_acceleration_rad_s2)
        if not all(math.isfinite(float(x)) and x>0 for x in vals):raise ValueError('limits_must_be_positive_finite')
        self.dt=1.0/float(control_rate_hz);self.tv=float(max_translation_velocity_m_s);self.rv=float(max_rotation_velocity_rad_s)
        self.ta=float(max_translation_acceleration_m_s2);self.ra=float(max_rotation_acceleration_rad_s2)
        self.previous_translation_velocity=(0.,0.,0.);self.previous_rotation_velocity=(0.,0.,0.)
    def reset(self):
        self.previous_translation_velocity=(0.,0.,0.);self.previous_rotation_velocity=(0.,0.,0.)
    def limit(self, action:Sequence[float])->LimitedAction:
        raw=tuple(float(x) for x in action)
        if len(raw)!=7 or not all(math.isfinite(x) for x in raw):raise ValueError('invalid_canonical_action')
        desired_t=tuple(x/self.dt for x in raw[:3]); desired_r=tuple(x/self.dt for x in raw[3:6])
        desired_t,ts=_scale_to(desired_t,self.tv);desired_r,rs=_scale_to(desired_r,self.rv)
        out_t,ta=_accel_limit(desired_t,self.previous_translation_velocity,self.ta*self.dt)
        out_r,ra=_accel_limit(desired_r,self.previous_rotation_velocity,self.ra*self.dt)
        self.previous_translation_velocity=out_t;self.previous_rotation_velocity=out_r
        limited=tuple(x*self.dt for x in out_t)+tuple(x*self.dt for x in out_r)+(raw[6],)
        return LimitedAction(raw,limited,out_t,out_r,ts,rs,ta,ra,
          _norm(tuple(a-b for a,b in zip(raw[:3],limited[:3]))),
          _norm(tuple(a-b for a,b in zip(raw[3:6],limited[3:6]))),False)
