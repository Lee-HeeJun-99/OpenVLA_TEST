#!/usr/bin/env python3
"""Compare preserved pre-fix decoupled replay with corrected OFT timing replay."""
import csv,json,sys
from collections import Counter
from pathlib import Path
HERE=Path(__file__).resolve().parent;P12=HERE.parent;OUT=HERE/'results'
BEFORE=P12/'08_gripper_state_cascade_analysis/results/decoupled_policy.csv'
AFTER=OUT/'episode4_oft_corrected_shadow.jsonl'
TIMING=P12/'09_oft_k5_timing_analysis/results/action_timing.csv'

def write(path,rows):
 rows=list(rows)
 with path.open('w',newline='') as f:
  w=csv.DictWriter(f,fieldnames=list(rows[0]) if rows else []);w.writeheader();w.writerows(rows)
def blockers(rows,key):return Counter(b for r in rows for b in json.loads(r[key]))

def main():
 with BEFORE.open() as f:before=list(csv.DictReader(f))
 after=[json.loads(x) for x in AFTER.read_text().splitlines() if x.strip()]
 with TIMING.open() as f:timing=list(csv.DictReader(f))
 bc=blockers(before,'blockers');ac=Counter(b for r in after for b in r['safety_decision']['reason'])
 labels=sorted(set(bc)|set(ac));comparison=[]
 before_accepted=sum(r['accepted']=='True' for r in before);after_accepted=sum(r['safety_decision']['accepted'] for r in after)
 for metric in ['total','accepted','rejected']+labels:
  if metric=='total':a=b=45
  elif metric=='accepted':a,b=before_accepted,after_accepted
  elif metric=='rejected':a,b=45-before_accepted,45-after_accepted
  else:a,b=bc[metric],ac[metric]
  comparison.append({'metric':metric,'before':a,'after_corrected':b,'difference':b-a})
 write(OUT/'before_after.csv',comparison)
 corrected=[]
 for row,t in zip(after,timing):
  ca=row['canonical_action'];corrected.append({'inference_frame':row['inference_frame'],'chunk_index':ca['chunk_index'],
   'target_step':row['target_step'],'target_time_s':row['expected_execution_time_s'],'action_age_sec':row['action_age_sec'],
   'inference_phase':row['inference_phase'],'mapped_phase':row['mapped_phase'],'gripper_closedness':ca['gripper_closedness'],
   'translation_norm_m':t['translation_norm_m'],'translation_velocity_mm_s':t['translation_velocity_mm_s'],
   'translation_acceleration_mm_s2':t['translation_acceleration_mm_s2'],'accepted':row['safety_decision']['accepted'],
   'blockers':json.dumps(row['safety_decision']['reason']),'command_issued':False})
 write(OUT/'action_timing_corrected.csv',corrected)
 krows=[]
 for k in range(5):
  old=[r for r in before if int(r['chunk_index'])==k];new=[r for r in corrected if int(r['chunk_index'])==k]
  ob=blockers(old,'blockers');nb=blockers(new,'blockers')
  krows.append({'chunk_index':k,'total':9,'before_accepted':sum(r['accepted']=='True' for r in old),
   'after_accepted':sum(bool(r['accepted']) for r in new),'after_rejected':sum(not bool(r['accepted']) for r in new),
   'after_reject_rate':sum(not bool(r['accepted']) for r in new)/9,
   'before_acceleration_blocker':ob['translation_acceleration_limit'],'after_acceleration_blocker':nb['translation_acceleration_limit'],
   'after_translation_step_blocker':nb['raw_translation_step_limit'],'after_premature_close':nb['premature_gripper_close']})
 write(OUT/'k_index_summary.csv',krows)
 first=next(r for r in corrected if float(r['gripper_closedness'])>=.7);grasp=next(r for r in corrected if r['mapped_phase']=='grasp_close')
 summary={'classification':'MODEL_EARLY_CLOSE_DOMINANT_AFTER_TIMING_FIX','before':{'accepted':before_accepted,'rejected':45-before_accepted,'blockers':dict(bc)},
  'after':{'accepted':after_accepted,'rejected':45-after_accepted,'blockers':dict(ac)},
  'accepted_change':after_accepted-before_accepted,'acceleration_blocker_change':ac['translation_acceleration_limit']-bc['translation_acceleration_limit'],
  'first_close_candidate_target_step':int(first['target_step']),'reference_close_command_step':26,'first_valid_grasp_close_step':int(grasp['target_step']),
  'early_close_lead_steps':26-int(first['target_step']),'early_close_lead_s':(26-int(first['target_step']))*.2,
  'k_index':krows,'production_thresholds_changed':False,'real_robot_commands':0,
  'next_step':'Keep corrected runtime timing; investigate OFT checkpoint/gripper intent before sequential-K5 real rollout.'}
 (OUT/'summary.json').write_text(json.dumps(summary,indent=2)+"\n")
 print(json.dumps(summary,indent=2))
if __name__=='__main__':main()
