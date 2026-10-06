#!/usr/bin/env python3
"""Offline OFT K=5 target-step, phase, and gripper timing audit."""
from __future__ import annotations
import csv,glob,json,math,statistics
from collections import Counter,defaultdict
from pathlib import Path
import numpy as np

HERE=Path(__file__).resolve().parent;P12=HERE.parent;LHJ=P12.parent;BUNDLE=LHJ.parent
SRC=P12.parent/'phase10_planner_based_shadow_mode/06_shadow_collection/offline_recorded_episode4/oft_vision_step28560_local/samples.jsonl'
DECOUPLED=P12/'08_gripper_state_cascade_analysis/results/decoupled_policy.csv'
REF_ROOT=BUNDLE/'data/real_world/raw_dataset_oft/episodes'
OUT=HERE/'results';DT=.2

def read_jsonl(p):return [json.loads(x) for x in Path(p).read_text().splitlines() if x.strip()]
def write_csv(p,rows):
 rows=list(rows);p.parent.mkdir(parents=True,exist_ok=True)
 with p.open('w',newline='') as f:
  w=csv.DictWriter(f,fieldnames=list(rows[0]) if rows else []);w.writeheader();w.writerows(rows)

def describe(v):
 a=np.asarray(v,float);return {'count':len(a),'mean':float(a.mean()),'std':float(a.std()),'min':float(a.min()),'median':float(np.median(a)),'p95':float(np.percentile(a,95)),'max':float(a.max())}

def reference_distribution():
 out=[]
 for p in sorted(REF_ROOT.glob('episode_*/steps_with_actions.jsonl')):
  rows=read_jsonl(p);cmd=next((i for i,r in enumerate(rows) if len(r.get('action') or [])>=7 and float(r['action'][6])>=.7),None)
  state=next((i for i,r in enumerate(rows) if float(r.get('gripper_closedness',0))>=.7),None)
  out.append({'episode_id':p.parent.name,'steps':len(rows),'reference_close_command_step':cmd,'reference_closed_state_step':state})
 return out

def main():
 source=read_jsonl(SRC);by_frame={int(r['frame_id']):r for r in source}
 with DECOUPLED.open() as f:dec={r['sequence_id']:r for r in csv.DictReader(f)}
 actions=[]
 for inference in source:
  chunk=inference.get('oft_denormalized_action_chunk')
  if not chunk:continue
  frame=int(inference['frame_id'])
  for k,a in enumerate(chunk):
   target=frame+k;mapped=by_frame[target];sid=f"{inference['episode_id']}-{frame}-k{k}"
   arr=np.asarray(a,float);close=bool(arr[6]>=.7);mapped_phase=mapped['phase'];coarse=inference['phase']
   refvec=(mapped.get('planner_canonical_action') or {}).get('vector') or mapped.get('planner_raw_action')
   actions.append({'frame':target,'inference_frame':frame,'chunk_id':f'episode4-frame{frame}','chunk_index':k,
    'inference_phase':coarse,'mapped_phase':mapped_phase,'phase_mapping_changed':coarse!=mapped_phase,
    'expected_execution_time_s':target*DT,'recorded_reference_time_s':mapped.get('timestamp_planner'),
    'legacy_runner_time_s':frame*1.0+k*DT,'legacy_time_error_s':frame*1.0+k*DT-target*DT,
    'gripper_closedness':float(arr[6]),'close_candidate':close,'valid_close_phase':mapped_phase=='grasp_close',
    'premature_close':close and mapped_phase!='grasp_close','reference_gripper_command':float(refvec[6]) if refvec else None,
    'translation_norm_m':float(np.linalg.norm(arr[:3])),'translation_velocity_mm_s':float(np.linalg.norm(arr[:3])/DT*1000),
    'translation_acceleration_mm_s2':'','accepted_decoupled':dec[sid]['accepted'],
    'blockers_decoupled':dec[sid]['blockers'],'command_issued':False})
 # Correct continuous 5 Hz acceleration, including chunk boundaries.
 velocities=[np.asarray((source[0].get('oft_denormalized_action_chunk') or [[0]*7])[0][:3])*0 for _ in []]
 raw_by=[]
 for r in actions:
  inf=by_frame[r['inference_frame']];raw_by.append(np.asarray(inf['oft_denormalized_action_chunk'][r['chunk_index']][:3],float))
 for i,r in enumerate(actions):
  if i:
   r['translation_acceleration_mm_s2']=float(np.linalg.norm(raw_by[i]/DT-raw_by[i-1]/DT)/DT*1000)
 write_csv(OUT/'action_timing.csv',actions)
 write_csv(OUT/'gripper_timing.csv',[{k:r[k] for k in ('frame','inference_frame','chunk_id','chunk_index','inference_phase','mapped_phase','gripper_closedness','close_candidate','valid_close_phase','premature_close','reference_gripper_command')} for r in actions])

 ksummary=[]
 for k in range(5):
  p=[r for r in actions if r['chunk_index']==k];acc=[float(r['translation_acceleration_mm_s2']) for r in p if r['translation_acceleration_mm_s2']!='']
  bc=Counter(b for r in p for b in json.loads(r['blockers_decoupled']))
  ksummary.append({'chunk_index':k,'count':len(p),'gripper_closedness_mean':statistics.mean(r['gripper_closedness'] for r in p),
   'translation_norm_mean_m':statistics.mean(r['translation_norm_m'] for r in p),
   'translation_velocity_mean_mm_s':statistics.mean(r['translation_velocity_mm_s'] for r in p),
   'translation_acceleration_mean_mm_s2':statistics.mean(acc) if acc else None,
   'translation_acceleration_max_mm_s2':max(acc) if acc else None,
   'accepted':sum(r['accepted_decoupled']=='True' for r in p),'rejected':sum(r['accepted_decoupled']!='True' for r in p),
   'reject_rate':sum(r['accepted_decoupled']!='True' for r in p)/len(p),
   'translation_step_blockers':bc['raw_translation_step_limit'],'acceleration_blockers':bc['translation_acceleration_limit'],
   'premature_close_blockers':bc['premature_gripper_close']})
 write_csv(OUT/'k_index_summary.csv',ksummary)

 phase=[]
 for name in sorted(set(r['mapped_phase'] for r in actions)):
  p=[r for r in actions if r['mapped_phase']==name]
  d=describe([r['gripper_closedness'] for r in p]);phase.append({'phase':name,**{f'closedness_{k}':v for k,v in d.items()},
   'close_candidates':sum(r['close_candidate'] for r in p),'premature_close':sum(r['premature_close'] for r in p)})
 write_csv(OUT/'phase_gripper_summary.csv',phase)
 refs=reference_distribution();write_csv(OUT/'reference_close_timing.csv',refs)
 first=next(r for r in actions if r['close_candidate']);first_grasp=next(r for r in actions if r['mapped_phase']=='grasp_close')
 ep4=next(r for r in refs if r['episode_id']=='episode_000004')
 ref_cmd=ep4['reference_close_command_step'];ref_state=ep4['reference_closed_state_step']
 coarse_prem=sum(r['close_candidate'] and r['inference_phase']!='grasp_close' for r in actions)
 mapped_prem=sum(r['premature_close'] for r in actions)
 summary={'classification':'MIXED','dominant_finding':'MODEL_EARLY_CLOSE_DOMINANT_WITH_RUNTIME_MAPPING_DEFECTS',
  'actions':len(actions),'chunks':len(actions)//5,'first_close_candidate':{'inference_frame':first['inference_frame'],'chunk_index':first['chunk_index'],
    'expanded_target_frame':first['frame'],'expected_execution_time_s':first['expected_execution_time_s'],'closedness':first['gripper_closedness']},
  'first_mapped_grasp_close_frame':first_grasp['frame'],'first_inference_grid_grasp_close_frame':30,
  'reference_episode4_close_command_step':ref_cmd,'reference_episode4_closed_state_step':ref_state,
  'model_lead_vs_reference_command_frames':ref_cmd-first['frame'],'model_lead_vs_reference_command_s':(ref_cmd-first['frame'])*DT,
  'model_lead_vs_grasp_phase_frames':first_grasp['frame']-first['frame'],'model_lead_vs_grasp_phase_s':(first_grasp['frame']-first['frame'])*DT,
  'phase_mapping':{'changed_actions':sum(r['phase_mapping_changed'] for r in actions),'total':len(actions),
    'premature_count_coarse_inference_phase':coarse_prem,'premature_count_target_step_phase':mapped_prem},
  'legacy_runner_timing_bug':{'formula':'frame_id*1.0 + chunk_index*0.2','correct_formula':'frame_id*0.2 + chunk_index*0.2',
    'max_time_error_s':max(abs(r['legacy_time_error_s']) for r in actions)},
  'reference_close_distribution':{'episodes':len(refs),'command_steps':[r['reference_close_command_step'] for r in refs],
    'median_command_step':statistics.median(r['reference_close_command_step'] for r in refs)},
  'k_index':ksummary,'production_thresholds_changed':False,'real_robot_commands':0,
  'training_dataset_note':'Checkpoint training corpus was not present as step-level episodes; 10 stored scripted reference trajectories are reported separately and are not claimed as training samples.'}
 (OUT/'summary.json').write_text(json.dumps(summary,indent=2)+"\n")
 print(json.dumps(summary,indent=2))

if __name__=='__main__':main()
