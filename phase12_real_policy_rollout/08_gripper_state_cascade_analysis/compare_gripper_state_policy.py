#!/usr/bin/env python3
"""Causal replay of legacy vs decoupled gripper bookkeeping (offline only)."""
from __future__ import annotations
import csv,json,math,sys
from collections import Counter
from pathlib import Path
import numpy as np

HERE=Path(__file__).resolve().parent;P12=HERE.parent;LHJ=P12.parent
sys.path[:0]=[str(P12/'02_safety'),str(P12/'07_safety_reject_analysis')]
from action_rate_limiter import CanonicalActionRateLimiter
from open_loop_gripper_supervisor import GripperRuntimeContext,OpenLoopGripperSupervisor
from runtime_safety_supervisor import RuntimeSafetySupervisor
from analysis_core import OFT_OUT,OFT_SRC,read_jsonl,source_position_map,vector,now_and_source

OUT=HERE/'results';WMIN=np.array([.275,-.355,.267]);WMAX=np.array([.554,.386,.754])

def write_csv(path,rows):
 rows=list(rows);path.parent.mkdir(parents=True,exist_ok=True)
 with path.open('w',newline='') as f:
  w=csv.DictWriter(f,fieldnames=list(rows[0]) if rows else []);w.writeheader();w.writerows(rows)

def run(policy):
 rows=read_jsonl(OFT_OUT);positions=source_position_map(OFT_SRC)
 limiter=CanonicalActionRateLimiter()
 safety=RuntimeSafetySupervisor(max_action_age_sec=1.2,min_command_period_sec=.19,
  max_translation_m=.004,max_rotation_rad=math.radians(4),max_translation_velocity_m_s=.02,
  max_rotation_velocity_rad_s=math.radians(20),max_translation_acceleration_m_s2=.02,
  max_rotation_acceleration_rad_s2=math.radians(20))
 grip=OpenLoopGripperSupervisor();grip.confirm_initial_open(True);out=[]
 for ordinal,row in enumerate(rows):
  raw=vector(row);limited=np.array(limiter.limit(raw).limited_action);now,source=now_and_source('oft',row)
  sid=row['canonical_action']['sequence_id'];sr=safety.inspect(action_id=sid,action=limited,
    source_monotonic=source,now_monotonic=now)
  pos=positions[int(row['frame_id'])];workspace_ok=not bool(np.any(pos+limited[:3]<WMIN) or np.any(pos+limited[:3]>WMAX))
  before=grip.state.value
  if policy=='current':
   gd=grip.resolve(raw[6],phase=row['phase'],fresh=sr.accepted,communication_ok=True,logger_ok=True)
  else:
   gd=grip.resolve_with_context(raw[6],phase=row['phase'],context=GripperRuntimeContext(
    prediction_fresh=True,action_accepted=sr.accepted and workspace_ok,communication_ok=True,
    command_channel_ok=True,initial_state_known=True,candidate_executed=False))
  premature=gd.reason=='close_forbidden_outside_grasp_close';blockers=[]
  if np.linalg.norm(raw[:3])>.004+1e-12:blockers.append('raw_translation_step_limit')
  if np.linalg.norm(raw[3:6])>math.radians(4)+1e-12:blockers.append('raw_rotation_step_limit')
  if not sr.accepted:blockers.append(sr.hold_reason)
  if not workspace_ok:blockers.append('workspace_violation')
  if premature:blockers.append('premature_gripper_close')
  if not gd.accepted and not premature:blockers.append('gripper_'+gd.reason)
  ca=row['canonical_action']
  out.append({'policy':policy,'action_ordinal':ordinal,'frame_id':row['frame_id'],'phase':row['phase'],
   'sequence_id':sid,'chunk_id':f"episode4-frame{row['frame_id']}",'chunk_index':ca['chunk_index'],
   'accepted':not blockers,'blockers':json.dumps(blockers),
   'gripper_model_closedness':float(raw[6]),'gripper_command_knowledge_before':before,
   'gripper_candidate':gd.candidate_command,'gripper_candidate_executed':False,
   'gripper_command_knowledge_after':gd.command_knowledge_after,
   'gripper_state_invalidated':gd.state_invalidated if policy=='decoupled' else gd.command_knowledge_after=='UNKNOWN' and before!='UNKNOWN',
   'gripper_state_invalidation_reason':gd.state_invalidation_reason if policy=='decoupled' else (gd.reason if gd.command_knowledge_after=='UNKNOWN' and before!='UNKNOWN' else ''),
   'gripper_reason':gd.reason,'measured_gripper_state':'MEASURED_UNKNOWN','command_issued':False})
 return out

def counts(rows):return Counter(b for r in rows for b in json.loads(r['blockers']))

def main():
 current=run('current');decoupled=run('decoupled')
 # Ensure the legacy causal replay is the already reported baseline.
 original=read_jsonl(OFT_OUT)
 assert [json.loads(r['blockers']) for r in current]==[r['safety_decision']['reason'] for r in original]
 write_csv(OUT/'current_policy.csv',current);write_csv(OUT/'decoupled_policy.csv',decoupled)
 write_csv(OUT/'oft_gripper_state_transitions_current.csv',current)
 write_csv(OUT/'oft_gripper_state_transitions_decoupled.csv',decoupled)
 cc,dc=counts(current),counts(decoupled);comparison=[]
 for b in sorted(set(cc)|set(dc)):
  comparison.append({'blocker':b,'current_count':cc[b],'decoupled_count':dc[b],'difference':dc[b]-cc[b]})
 write_csv(OUT/'blocker_comparison.csv',comparison)
 krows=[]
 for k in range(5):
  a=[r for r in current if int(r['chunk_index'])==k];b=[r for r in decoupled if int(r['chunk_index'])==k]
  krows.append({'chunk_index':k,'total':len(a),'current_accepted':sum(r['accepted'] for r in a),
   'current_rejected':sum(not r['accepted'] for r in a),'decoupled_accepted':sum(r['accepted'] for r in b),
   'decoupled_rejected':sum(not r['accepted'] for r in b)})
 write_csv(OUT/'oft_k_index_comparison.csv',krows)
 before=sum(r['accepted'] for r in current);after=sum(r['accepted'] for r in decoupled);removed=after-before
 # A change affecting >=20% but not a majority of all rejects is classified partial.
 cascade='GRIPPER_STATE_CASCADE_CONFIRMED_DOMINANT' if removed>21 else ('GRIPPER_STATE_CASCADE_CONFIRMED_PARTIAL' if removed>0 else 'GRIPPER_STATE_CASCADE_NOT_SIGNIFICANT')
 remain=counts([r for r in decoupled if not r['accepted']])
 # Multiple independent blocker families remain in this episode.
 dominant='MIXED' if len([x for x in remain.values() if x])>1 else ('ACTION_MAGNITUDE_DOMINANT' if remain.get('raw_translation_step_limit') else 'MIXED')
 summary={'classification':'OFT_EPISODE4_GRIPPER_STATE_CAUSAL_REPLAY','current':{'total':45,'accepted':before,'rejected':45-before,'blockers':dict(cc)},
  'decoupled':{'total':45,'accepted':after,'rejected':45-after,'blockers':dict(dc)},
  'cascade_generated_rejects_removed':removed,'cascade_classification':cascade,'remaining_oft_dominant_cause':dominant,
  'k_index':krows,'production_thresholds_changed':False,'real_robot_commands':0,
  'interpretation_limit':'Recorded Episode 4 command-disabled causal software replay; not task success or real rollout.'}
 (OUT/'summary.json').write_text(json.dumps(summary,indent=2)+"\n")
 print(json.dumps(summary,indent=2))

if __name__=='__main__':main()
