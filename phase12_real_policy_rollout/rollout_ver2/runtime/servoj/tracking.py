from __future__ import annotations
from dataclasses import dataclass
import numpy as np

def validate_servoj_time(time_s,minimum_s=.02,maximum_s=.10):
    value=float(time_s)
    if not np.isfinite(value) or value<float(minimum_s):raise ValueError(f"SERVOJ_TIME_BELOW_VERIFIED_MIN:{value}<{minimum_s}")
    if value>float(maximum_s):raise ValueError(f"SERVOJ_TIME_ABOVE_VERIFIED_MAX:{value}>{maximum_s}")
    return value

def tracking_delay_s(time_s,base_s=.020,time_scale=.6):
    return float(base_s)+float(time_scale)*float(time_s)

def tracking_threshold_deg(time_s,segment_max_joint_velocity_deg_s,*,base_s=.020,time_scale=.6,
                           multiplier=2.,floor_deg=.1):
    validate_servoj_time(time_s)
    velocity=np.asarray(segment_max_joint_velocity_deg_s,float)
    if np.any(~np.isfinite(velocity)) or np.any(velocity<0):raise ValueError("invalid segment velocity")
    return float(multiplier)*tracking_delay_s(time_s,base_s,time_scale)*velocity+float(floor_deg)
from runtime import robot_state as rs


@dataclass(frozen=True)
class TrackingFault:
    reasons:tuple;error_deg:np.ndarray


class TrackingMonitor:
    def __init__(self,threshold_deg):
        value=np.asarray(threshold_deg,dtype=float)
        self.threshold=np.repeat(value,6) if value.ndim==0 else value
        if self.threshold.shape!=(6,) or np.any(self.threshold<=0):raise ValueError("six positive thresholds required")
        self.maximum=np.zeros(6)
    def check(self,command_joint_rad,snapshot,stale_reasons=()):
        error=np.abs(np.rad2deg(np.asarray(snapshot.joint_rad)-np.asarray(command_joint_rad)))
        self.maximum=np.maximum(self.maximum,error);reasons=list(stale_reasons)
        if snapshot.robot_state not in rs.ALLOWED_RUNNING_STATES:reasons.append(f"ROBOT_STATE_NOT_RUNNING:{snapshot.robot_state}")
        reasons.extend(f"TRACKING_ERROR:J{i+1}" for i in np.flatnonzero(error>self.threshold))
        return None if not reasons else TrackingFault(tuple(reasons),error)
