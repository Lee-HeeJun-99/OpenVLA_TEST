from __future__ import annotations
from dataclasses import dataclass
import numpy as np
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
