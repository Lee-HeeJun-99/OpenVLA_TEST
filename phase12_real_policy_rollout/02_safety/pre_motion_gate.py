"""Fail-closed pre-motion gate; it has no ROS or command capability."""
from __future__ import annotations
from dataclasses import dataclass
import math
from typing import Mapping,Sequence
from workspace_contract import CartesianWorkspace

@dataclass(frozen=True)
class PreMotionDecision:
    ready: bool
    blockers: tuple[str,...]
    hold_required: bool
    command_issued: bool=False

class PreMotionGate:
    REQUIRED_SIGNALS=('operator_approved','hardware_estop_verified','protective_stop_clear',
      'servo_state_verified','robot_mode_verified','controller_alarm_clear','hold_ack_verified',
      'gripper_feedback_or_approved_no_feedback_policy','workspace_site_approved')
    def __init__(self,workspace:CartesianWorkspace,max_rollout_duration_s:float):
        if not math.isfinite(max_rollout_duration_s) or max_rollout_duration_s<=0:raise ValueError('invalid_rollout_duration')
        self.workspace=workspace;self.max_rollout_duration_s=float(max_rollout_duration_s)
    def inspect(self,*,signals:Mapping[str,bool],elapsed_s:float,current_position_m:Sequence[float],
                camera_ok:bool,joint_state_ok:bool,tcp_ok:bool,logger_ok:bool,model_ok:bool)->PreMotionDecision:
        blockers=[]
        for name in self.REQUIRED_SIGNALS:
            if signals.get(name) is not True:blockers.append(name)
        if not math.isfinite(float(elapsed_s)) or elapsed_s<0:blockers.append('invalid_elapsed_time')
        elif elapsed_s>self.max_rollout_duration_s:blockers.append('rollout_timeout')
        for ok,name in ((camera_ok,'camera_unavailable'),(joint_state_ok,'joint_state_unavailable'),
                        (tcp_ok,'tcp_unavailable'),(logger_ok,'logger_unavailable'),(model_ok,'model_unavailable')):
            if not ok:blockers.append(name)
        w=self.workspace.inspect(current_position_m)
        if not w.accepted:blockers.append(w.reason or 'workspace_violation')
        return PreMotionDecision(not blockers,tuple(blockers),bool(blockers),False)
