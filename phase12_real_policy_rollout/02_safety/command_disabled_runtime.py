"""Phase 12 end-to-end command candidate runtime with no delivery capability."""
from __future__ import annotations
import math
from scipy.spatial.transform import Rotation
from action_contract import compose_world_rotvec_with_doosan_zyz,translation_m_to_mm,rotation_step_geodesic_rad
from action_rate_limiter import CanonicalActionRateLimiter
from open_loop_gripper_supervisor import OpenLoopGripperSupervisor,GripperRuntimeContext,OpenLoopDecision
from oft_chunk_queue import OFTChunkQueue
from runtime_safety_supervisor import RuntimeSafetySupervisor
from workspace_contract import PHASE12_DATA_DERIVED_WORKSPACE
from action_step_limits import RAW_TRANSLATION_STEP_LIMIT_M,raw_translation_exceeds_limit

ALLOWED_CLOSE_PHASES=frozenset(('grasp_close','lift','post_lift'))

def _supervisor(*,full_endpoint_delta=False,enforce_raw_translation_limit=True,translation_endpoint_delta=False):
 return RuntimeSafetySupervisor(max_action_age_sec=1.2,min_command_period_sec=.19,
  max_translation_m=RAW_TRANSLATION_STEP_LIMIT_M,max_rotation_rad=math.radians(4),
  max_translation_velocity_m_s=.02,max_rotation_velocity_rad_s=math.radians(20),
  max_translation_acceleration_m_s2=.02,max_rotation_acceleration_rad_s2=math.radians(20),
  dynamics_managed_by_controller=full_endpoint_delta,translation_step_limit_enabled=enforce_raw_translation_limit,
  translation_dynamics_managed_by_controller=translation_endpoint_delta)

class CommandDisabledPolicyRuntime:
 """Calculates auditable candidates. There is deliberately no command method."""
 def __init__(self,model,*,operator_confirmed_initial_open=False,full_endpoint_delta=False,enforce_raw_translation_limit=True,translation_endpoint_delta=False,raw_rotation_step_limit_deg=4.):
  if model not in ('openvla','oft'):raise ValueError('unsupported_model')
  if type(full_endpoint_delta) is not bool:raise ValueError('invalid_endpoint_delta_mode')
  if full_endpoint_delta and model!='openvla':raise ValueError('full_endpoint_requires_openvla')
  if type(enforce_raw_translation_limit) is not bool:raise ValueError('invalid_translation_gate_mode')
  if not enforce_raw_translation_limit and not full_endpoint_delta:raise PermissionError('disabled_translation_gate_requires_full_endpoint')
  self.full_endpoint_delta=full_endpoint_delta
  if type(translation_endpoint_delta) is not bool:raise ValueError('invalid_translation_endpoint_mode')
  if translation_endpoint_delta and (model!='openvla' or full_endpoint_delta or not enforce_raw_translation_limit):raise ValueError('invalid_translation_endpoint_combination')
  self.translation_endpoint_delta=translation_endpoint_delta
  self.raw_rotation_step_limit_deg=float(raw_rotation_step_limit_deg)
  if not math.isfinite(self.raw_rotation_step_limit_deg) or not 0 < self.raw_rotation_step_limit_deg < 180:
   raise ValueError('invalid_raw_rotation_step_limit')
  self.enforce_raw_translation_limit=enforce_raw_translation_limit
  self.model=model;self.limiter=CanonicalActionRateLimiter();self.safety=_supervisor(full_endpoint_delta=full_endpoint_delta,enforce_raw_translation_limit=enforce_raw_translation_limit,translation_endpoint_delta=translation_endpoint_delta);self.gripper=OpenLoopGripperSupervisor()
  if operator_confirmed_initial_open:self.gripper.confirm_initial_open(True)
  self.queue=OFTChunkQueue('sequential_k5',1.2) if model=='oft' else None
 def enqueue_oft(self,actions,chunk_id,inference_monotonic):
  if self.model!='oft':raise RuntimeError('not_oft_runtime')
  self.queue.enqueue(actions,chunk_id,inference_monotonic)
 def next_oft(self,now_monotonic,**kwargs):
  if self.model!='oft':raise RuntimeError('not_oft_runtime')
  q=self.queue.pop(now_monotonic)
  return self.evaluate(q.action,action_id=f'{q.chunk_id}-k{q.chunk_index}',source_monotonic=q.inference_monotonic,now_monotonic=now_monotonic,chunk_id=q.chunk_id,chunk_index=q.chunk_index,action_age_sec=q.action_age_sec,**kwargs)
 def evaluate(self,raw_action,*,action_id,source_monotonic,now_monotonic,current_position_m,current_abc_deg,phase,
              camera_ok=True,state_ok=True,tcp_ok=True,inference_ok=True,communication_ok=True,logger_ok=True,
              chunk_id=None,chunk_index=0,action_age_sec=None,arm_only=False):
  raw=tuple(float(v) for v in raw_action)
  if len(raw)!=7 or not all(math.isfinite(v) for v in raw):raise ValueError('invalid_canonical_action')
  a=raw if self.full_endpoint_delta else self.limiter.limit(raw).limited_action
  if self.translation_endpoint_delta:a=raw[:3]+tuple(a[3:])
  safety=self.safety.inspect(action_id=action_id,action=a,source_monotonic=source_monotonic,now_monotonic=now_monotonic,
    camera_ok=camera_ok,state_ok=state_ok,tcp_ok=tcp_ok,inference_ok=inference_ok,communication_ok=communication_ok,logger_ok=logger_ok)
  workspace=PHASE12_DATA_DERIVED_WORKSPACE.inspect_delta(current_position_m,a[:3])
  grip=OpenLoopDecision(raw_closedness=raw[6],command_knowledge_before=self.gripper.state.value,
    command_knowledge_after=self.gripper.state.value,candidate_command=None,command_suppressed=True,
    accepted=True,lift_allowed=False,close_count=self.gripper.close_count,reason='arm_only_gripper_output_disabled') if arm_only else self.gripper.resolve_with_context(raw_action[6],phase=phase,context=GripperRuntimeContext(
    prediction_fresh=inference_ok,action_accepted=safety.accepted and workspace.accepted,
    communication_ok=communication_ok,command_channel_ok=communication_ok,
    initial_state_known=self.gripper.state.value!='UNKNOWN',candidate_executed=False))
  premature=grip.reason=='close_forbidden_outside_grasp_close'
  blockers=[]
  if self.enforce_raw_translation_limit and raw_translation_exceeds_limit(raw[:3]):blockers.append('raw_translation_step_limit')
  raw_target_abc,_=compose_world_rotvec_with_doosan_zyz(current_abc_deg,raw[3:6])
  raw_rotation_angle=rotation_step_geodesic_rad(current_abc_deg,raw_target_abc)
  if raw_rotation_angle > math.radians(self.raw_rotation_step_limit_deg)+1e-12:blockers.append('raw_rotation_step_limit')
  if not safety.accepted:blockers.append(safety.hold_reason)
  if not workspace.accepted:blockers.append(workspace.reason)
  if premature:blockers.append('premature_gripper_close')
  if not grip.accepted and not premature:blockers.append('gripper_'+grip.reason)
  dmm=translation_m_to_mm(a[:3]).delta_mm;abc,_=compose_world_rotvec_with_doosan_zyz(current_abc_deg,a[3:6])
  target=[(float(current_position_m[i])*1000)+dmm[i] for i in range(3)]+abc.tolist()
  return {'classification':'COMMAND_DISABLED_RUNTIME_DRY_RUN','model':self.model,'phase':phase,
   'action_id':action_id,'chunk_id':chunk_id,'chunk_index':chunk_index,'action_age_sec':action_age_sec,
   'raw_canonical_action':list(raw),'limited_canonical_action':list(a),
   'raw_rotation_geodesic_rad':raw_rotation_angle,
   'effective_raw_rotation_limit_deg':self.raw_rotation_step_limit_deg,
   'limiter_modified':list(raw)!=list(a),'target_pose_candidate_mm_zyz_deg':target,
   'delta_limiter_applied':not self.full_endpoint_delta,
   'translation_delta_limiter_applied':not (self.full_endpoint_delta or self.translation_endpoint_delta),
   'rotation_delta_limiter_applied':not self.full_endpoint_delta,
   'translation_dynamic_profile_owner':'SERVO_20HZ_TRACKER' if self.translation_endpoint_delta else ('HARDWARE_ENDPOINT_PROFILE' if self.full_endpoint_delta else 'CANONICAL_RATE_LIMITER'),
   'raw_translation_gate_enabled':self.enforce_raw_translation_limit,
   'effective_raw_translation_limit_m':RAW_TRANSLATION_STEP_LIMIT_M if self.enforce_raw_translation_limit else None,
   'dynamic_profile_owner':'HARDWARE_ENDPOINT_PROFILE' if self.full_endpoint_delta else 'CANONICAL_RATE_LIMITER',
   'gripper':{'raw_closedness':grip.raw_closedness,'command_knowledge_before':grip.command_knowledge_before,
    'command_knowledge_after':grip.command_knowledge_after,'candidate_command':grip.candidate_command,
    'candidate_closed':grip.command_knowledge_after=='COMMAND_CLOSED','suppressed':grip.command_suppressed,
    'premature_close':premature,'close_count':grip.close_count,'lift_allowed':grip.lift_allowed,
    'model_gripper_closedness':grip.raw_closedness,
    'gripper_command_knowledge_before':grip.command_knowledge_before,
    'gripper_command_knowledge_after':grip.command_knowledge_after,
    'gripper_candidate':grip.candidate_command,
    'gripper_candidate_executed':False,
    'gripper_state_invalidated':grip.state_invalidated,
    'gripper_state_invalidation_reason':grip.state_invalidation_reason,
    'measured_gripper_state':None,'reason':grip.reason},
   'workspace_status':PHASE12_DATA_DERIVED_WORKSPACE.provenance,'technical_blockers':blockers,
   'hold_required':bool(blockers),'technical_valid':not blockers,
   'pre_motion_ready':False,'physical_safety_deferred':True,
   'executed_action':None,'robot_delivered_command':None,'command_issued':False}
