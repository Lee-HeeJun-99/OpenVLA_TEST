"""Phase 12 end-to-end command candidate runtime with no delivery capability."""
from __future__ import annotations
import math
from scipy.spatial.transform import Rotation
from action_contract import compose_world_rotvec_with_doosan_zyz,translation_m_to_mm
from action_rate_limiter import CanonicalActionRateLimiter
from open_loop_gripper_supervisor import OpenLoopGripperSupervisor,GripperRuntimeContext
from oft_chunk_queue import OFTChunkQueue
from runtime_safety_supervisor import RuntimeSafetySupervisor
from workspace_contract import PHASE12_DATA_DERIVED_WORKSPACE
from action_step_limits import RAW_TRANSLATION_STEP_LIMIT_M,raw_translation_exceeds_limit

ALLOWED_CLOSE_PHASES=frozenset(('grasp_close','lift','post_lift'))

def _supervisor():
 return RuntimeSafetySupervisor(max_action_age_sec=1.2,min_command_period_sec=.19,
  max_translation_m=RAW_TRANSLATION_STEP_LIMIT_M,max_rotation_rad=math.radians(4),
  max_translation_velocity_m_s=.02,max_rotation_velocity_rad_s=math.radians(20),
  max_translation_acceleration_m_s2=.02,max_rotation_acceleration_rad_s2=math.radians(20))

class CommandDisabledPolicyRuntime:
 """Calculates auditable candidates. There is deliberately no command method."""
 def __init__(self,model,*,operator_confirmed_initial_open=False):
  if model not in ('openvla','oft'):raise ValueError('unsupported_model')
  self.model=model;self.limiter=CanonicalActionRateLimiter();self.safety=_supervisor();self.gripper=OpenLoopGripperSupervisor()
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
              chunk_id=None,chunk_index=0,action_age_sec=None):
  raw=tuple(float(v) for v in raw_action)
  limited=self.limiter.limit(raw);a=limited.limited_action
  safety=self.safety.inspect(action_id=action_id,action=a,source_monotonic=source_monotonic,now_monotonic=now_monotonic,
    camera_ok=camera_ok,state_ok=state_ok,tcp_ok=tcp_ok,inference_ok=inference_ok,communication_ok=communication_ok,logger_ok=logger_ok)
  workspace=PHASE12_DATA_DERIVED_WORKSPACE.inspect_delta(current_position_m,a[:3])
  grip=self.gripper.resolve_with_context(raw_action[6],phase=phase,context=GripperRuntimeContext(
    prediction_fresh=inference_ok,action_accepted=safety.accepted and workspace.accepted,
    communication_ok=communication_ok,command_channel_ok=communication_ok,
    initial_state_known=self.gripper.state.value!='UNKNOWN',candidate_executed=False))
  premature=grip.reason=='close_forbidden_outside_grasp_close'
  blockers=[]
  if raw_translation_exceeds_limit(raw[:3]):blockers.append('raw_translation_step_limit')
  if math.sqrt(sum(v*v for v in raw[3:6])) > math.radians(4)+1e-12:blockers.append('raw_rotation_step_limit')
  if not safety.accepted:blockers.append(safety.hold_reason)
  if not workspace.accepted:blockers.append(workspace.reason)
  if premature:blockers.append('premature_gripper_close')
  if not grip.accepted and not premature:blockers.append('gripper_'+grip.reason)
  dmm=translation_m_to_mm(a[:3]).delta_mm;abc,_=compose_world_rotvec_with_doosan_zyz(current_abc_deg,a[3:6])
  target=[(float(current_position_m[i])*1000)+dmm[i] for i in range(3)]+abc.tolist()
  return {'classification':'COMMAND_DISABLED_RUNTIME_DRY_RUN','model':self.model,'phase':phase,
   'action_id':action_id,'chunk_id':chunk_id,'chunk_index':chunk_index,'action_age_sec':action_age_sec,
   'raw_canonical_action':list(raw),'limited_canonical_action':list(a),
   'limiter_modified':list(raw)!=list(a),'target_pose_candidate_mm_zyz_deg':target,
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
