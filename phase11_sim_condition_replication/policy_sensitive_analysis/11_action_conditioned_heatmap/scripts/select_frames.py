#!/usr/bin/env python3
"""Deterministic representative-frame selection from Phase 11 action metrics."""
import csv,json
from collections import defaultdict
from pathlib import Path

H=Path(__file__).resolve().parents[1]; P=H.parent
OUT=H/'01_selected_frames'; OUT.mkdir(parents=True,exist_ok=True)
actions=list(csv.DictReader((P/'06_action_gap/frame_level_action_metrics.csv').open()))
aligned=list(csv.DictReader((P/'03_alignment/aligned_samples.csv').open()))
amap={(r['condition'],r['episode_id'],int(r['baseline_index']),int(r['condition_index'])):r for r in aligned}
selected={}
def add(r,rule,high=False):
 key=(r['model'],r['condition'],r['episode_id'],r['baseline_index'],r['condition_index'])
 x=selected.setdefault(key,{'model':r['model'],'condition':r['condition'],'episode_id':r['episode_id'],'baseline_index':r['baseline_index'],'condition_index':r['condition_index'],'phase':r['phase'],'selection_rules':[],'high_cost':False})
 x['selection_rules'].append(rule);x['high_cost']=x['high_cost'] or high
for key,ss in defaultdict(list).items(): pass
groups=defaultdict(list)
for r in actions: groups[(r['model'],r['condition'])].append(r)
for (model,cond),ss in sorted(groups.items()):
 for metric,label in [('condition_shift_standardized_l2','top_total'),('condition_shift_translation_l2_m','top_translation'),('condition_shift_rotation_l2_rad','top_rotation'),('condition_shift_gripper_abs','top_gripper')]:
  add(max(ss,key=lambda r:float(r[metric])),label,True)
 add(min(ss,key=lambda r:float(r['condition_shift_standardized_l2'])),'near_zero',False)
 # One phase-center example per episode, plus the last descent as pre-grasp.
 for eid in sorted({r['episode_id'] for r in ss}):
  ee=[r for r in ss if r['episode_id']==eid]
  for phase in ('hold','alignment','descent_to_grasp','grasp_close','lift'):
   q=[r for r in ee if r['phase']==phase]
   if q: add(q[len(q)//2],f'phase_center:{phase}',False)
  q=[r for r in ee if r['phase']=='descent_to_grasp']
  if q:add(q[-1],'pre_grasp_last_descent',False)
 # False-close and first persistent false-close diagnostics.
 fc=[r for r in ss if int(r['reference_binary_close'])==0 and int(r['condition_binary_close'])==1]
 if fc:
  add(fc[0],'first_false_close',model=='oft' and cond=='lighting_low')
  for i in range(len(fc)-4):
   idx=[int(x['condition_index']) for x in fc[i:i+5]]
   if idx==list(range(idx[0],idx[0]+5)):
    add(fc[i],'persistent_false_close_n5_start',model=='oft' and cond=='lighting_low');break
rows=[]
for x in selected.values():
 a=amap[(x['condition'],x['episode_id'],int(x['baseline_index']),int(x['condition_index']))]
 x['baseline_image']=a['baseline_image'];x['condition_image']=a['condition_image'];x['selection_rules']=';'.join(sorted(set(x['selection_rules'])));rows.append(x)
rows.sort(key=lambda r:(r['model'],r['condition'],r['episode_id'],int(r['condition_index'])))
with (OUT/'selected_frames.csv').open('w',newline='') as f:
 w=csv.DictWriter(f,fieldnames=rows[0].keys());w.writeheader();w.writerows(rows)
(OUT/'selection_summary.json').write_text(json.dumps({'selected_rows':len(rows),'high_cost_rows':sum(bool(r['high_cost']) for r in rows),'policy':'phase centers plus metric extrema; high-cost uses per-condition extrema and OFT-lighting false-close events'},indent=2))
print(json.dumps({'selected_rows':len(rows),'high_cost_rows':sum(bool(r['high_cost']) for r in rows)},indent=2))
