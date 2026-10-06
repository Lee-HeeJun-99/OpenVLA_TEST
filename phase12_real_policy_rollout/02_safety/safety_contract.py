"""Fail-closed offline safety decision. It reports hold_required but never calls Hold."""
from dataclasses import dataclass
import math

@dataclass(frozen=True)
class SafetyDecision:
    accepted: bool
    hold_required: bool
    reason: str

def validate_startup(flags):
    required_false=('allow_robot_command','allow_gripper_command','allow_home_command','allow_trajectory_execution','allow_motion_service','allow_stop_service','allow_estop_service')
    unsafe=[k for k in required_false if flags.get(k) is not False]
    if flags.get('mode') not in ('offline_preparation','subscriber_only_shadow') or flags.get('shadow_mode') is not True or unsafe:raise RuntimeError('unsafe_startup:'+','.join(unsafe))

def validate_action(action,current_pose,workspace_min,workspace_max,max_translation_m,max_rotation_rad,logger_ok=True,communication_ok=True,timed_out=False):
    if not logger_ok:return SafetyDecision(False,True,'logger_failure')
    if not communication_ok:return SafetyDecision(False,True,'communication_loss')
    if timed_out:return SafetyDecision(False,True,'inference_timeout')
    if len(action)!=7 or not all(math.isfinite(float(x)) for x in action):return SafetyDecision(False,True,'invalid_action')
    t=math.sqrt(sum(float(x)**2 for x in action[:3]));r=math.sqrt(sum(float(x)**2 for x in action[3:6]))
    if t>max_translation_m:return SafetyDecision(False,True,'excessive_translation')
    if r>max_rotation_rad:return SafetyDecision(False,True,'excessive_rotation')
    target=[float(current_pose[i])+float(action[i]) for i in range(6)]
    if any(x<lo or x>hi for x,lo,hi in zip(target,workspace_min,workspace_max)):return SafetyDecision(False,True,'workspace_violation')
    return SafetyDecision(True,False,'accepted_offline_only')

def validate_joint_state(position,velocity,position_min,position_max,velocity_max):
    arrays=(position,velocity,position_min,position_max,velocity_max)
    if len({len(x) for x in arrays})!=1 or not all(math.isfinite(float(v)) for x in arrays for v in x):return SafetyDecision(False,True,'invalid_joint_state')
    if any(float(q)<float(lo) or float(q)>float(hi) for q,lo,hi in zip(position,position_min,position_max)):return SafetyDecision(False,True,'joint_limit_violation')
    if any(abs(float(v))>float(limit) for v,limit in zip(velocity,velocity_max)):return SafetyDecision(False,True,'joint_velocity_violation')
    return SafetyDecision(True,False,'joint_state_accepted_offline_only')
