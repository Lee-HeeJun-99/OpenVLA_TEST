#!/usr/bin/env python3
"""Paired action analysis for actual Phase 11 prediction-only outputs."""
import csv, json
from collections import defaultdict
from pathlib import Path
import numpy as np
import matplotlib.pyplot as plt

ROOT=Path(__file__).resolve().parents[1]
PRED=ROOT/'01_predictions'; OUT=ROOT/'06_action_gap'; FIG=ROOT/'09_figures'
OUT.mkdir(exist_ok=True); FIG.mkdir(exist_ok=True)

def load_episode(folder,cond,eid):
 p=PRED/folder/cond/eid/'samples.jsonl'
 return {int(x['frame_id']):x for x in (json.loads(s) for s in p.read_text().splitlines() if s.strip())}
def vec(d): return np.asarray(d['vector'],float)
def pred(rows,model,index):
 if model=='openvla':
  x=rows.get(index); return None if not x else vec(x['openvla_canonical_action'])
 base=(index//5)*5; x=rows.get(base); chunk=None if not x else x.get('oft_canonical_action_chunk')
 return None if not chunk or index-base>=len(chunk) else vec(chunk[index-base])
def metrics(prefix,d,out,scale):
 out[prefix+'_translation_l2_m']=float(np.linalg.norm(d[:3])); out[prefix+'_rotation_l2_rad']=float(np.linalg.norm(d[3:6]))
 out[prefix+'_gripper_abs']=float(abs(d[6])); out[prefix+'_raw_l2']=float(np.linalg.norm(d)); out[prefix+'_no_gripper_l2']=float(np.linalg.norm(d[:6]))
 out[prefix+'_standardized_l2']=float(np.linalg.norm(d/scale)); out[prefix+'_gripper_contribution_ratio']=float(d[6]**2/max(np.dot(d,d),1e-15))

aligned=list(csv.DictReader((ROOT/'03_alignment/aligned_samples.csv').open()))
stats_root=Path('/home/ubuntu/a0509_vla_linux_field_bundle_20260903/models')
scales={}
for m,p in [('openvla',stats_root/'vanilla_s1_balanced_step8130/dataset_statistics.json'),('oft',stats_root/'oft_mixed480_step28560/dataset_statistics.json')]:
 scales[m]=np.asarray(json.loads(p.read_text())['a0509_sim_cube_pick']['action']['std'],float)
rows=[]; cache={}
for a in aligned:
 cond=a['condition']; eid=a['episode_id']; bi=int(a['baseline_index']); ci=int(a['condition_index'])
 for model,folder in [('openvla','openvla'),('oft','oft_run2')]:
  for c in ('baseline',cond):
   cache.setdefault((folder,c,eid),load_episode(folder,c,eid))
  br=cache[(folder,'baseline',eid)]; cr=cache[(folder,cond,eid)]
  bp=pred(br,model,bi); cp=pred(cr,model,ci)
  if bp is None or cp is None: continue
  refb=vec(br[bi]['planner_canonical_action']); refc=vec(cr[ci]['planner_canonical_action'])
  out={'model':model,'condition':cond,'episode_id':eid,'phase':a['phase'],'baseline_index':bi,'condition_index':ci,
       'oft_chunk_index':bi%5 if model=='oft' else '', 'baseline_gripper':bp[6],'condition_gripper':cp[6],
       'reference_gripper':refc[6],'baseline_binary_close':int(bp[6]>=.5),'condition_binary_close':int(cp[6]>=.5),'reference_binary_close':int(refc[6]>=.5)}
  metrics('condition_shift',cp-bp,out,scales[model]); metrics('baseline_reference_error',bp-refb,out,scales[model]); metrics('condition_reference_error',cp-refc,out,scales[model])
  for j,value in enumerate((cp-bp)/scales[model]): out[f'condition_shift_standardized_d{j}']=float(value)
  rows.append(out)
def write(path,rr):
 with path.open('w',newline='') as f:
  w=csv.DictWriter(f,fieldnames=rr[0].keys()); w.writeheader(); w.writerows(rr)
write(OUT/'frame_level_action_metrics.csv',rows)
nums=[c for c in rows[0] if c.startswith(('condition_shift_','condition_reference_error_','baseline_reference_error_'))]
def aggregate(keys,with_std=False):
 groups=defaultdict(list)
 for r in rows: groups[tuple(r[k] for k in keys)].append(r)
 out=[]
 for key,ss in sorted(groups.items()):
  z={k:v for k,v in zip(keys,key)}
  for n in nums:
   vals=np.asarray([float(r[n]) for r in ss]); z[n+'_mean']=float(vals.mean())
   if with_std: z[n+'_std']=float(vals.std()); z[n+'_count']=len(vals)
  out.append(z)
 return out
ep=aggregate(['model','condition','episode_id']); write(OUT/'episode_level_action_metrics.csv',ep)
phase=aggregate(['model','condition','phase'],True); write(OUT/'phase_level_action_summary.csv',phase)
cs=aggregate(['model','condition'],True); write(OUT/'condition_level_action_summary.csv',cs)
g=[]
groups=defaultdict(list)
for r in rows: groups[(r['model'],r['condition'])].append(r)
for key,s in sorted(groups.items()):
 y=np.asarray([r['reference_binary_close'] for r in s]); q=np.asarray([r['condition_binary_close'] for r in s]); tp=int(((y==1)&(q==1)).sum());tn=int(((y==0)&(q==0)).sum());fp=int(((y==0)&(q==1)).sum());fn=int(((y==1)&(q==0)).sum())
 g.append({'model':key[0],'condition':key[1],'n':len(s),'tp':tp,'tn':tn,'fp':fp,'fn':fn,'accuracy':(tp+tn)/len(s),'precision':tp/max(tp+fp,1),'recall':tp/max(tp+fn,1),'f1':2*tp/max(2*tp+fp+fn,1)})
write(OUT/'gripper_condition_summary.csv',g)
oft_groups=defaultdict(list)
for r in rows:
 if r['model']=='oft': oft_groups[(r['condition'],r['oft_chunk_index'])].append(r)
oft=[]
for key,ss in sorted(oft_groups.items()):
 z={'condition':key[0],'oft_chunk_index':key[1]}
 for n in nums: z[n]=float(np.mean([r[n] for r in ss]))
 oft.append(z)
write(OUT/'oft_chunk_index_summary.csv',oft)
for metric,name in [('condition_shift_standardized_l2','scale_normalized_action_shift'),('condition_shift_raw_l2','raw_action_shift')]:
 conds=sorted({r['condition'] for r in ep}); models=['openvla','oft']; x=np.arange(len(conds)); width=.36
 plt.figure(figsize=(8,4))
 for j,m in enumerate(models):
  vals=[np.mean([r[metric+'_mean'] for r in ep if r['model']==m and r['condition']==c]) for c in conds]
  plt.bar(x+(j-.5)*width,vals,width,label=m)
 plt.xticks(x,conds);plt.ylabel(metric);plt.legend();plt.title(name.replace('_',' ').title());plt.tight_layout();plt.savefig(FIG/(name+'.png'),dpi=160);plt.close()
summary={'status':'COMPLETED_WITH_MODEL_SERVER','aligned_model_rows':len(rows),'openvla_rows':sum(r['model']=='openvla' for r in rows),'oft_expanded_k5_rows':sum(r['model']=='oft' for r in rows),'executed_action':None,'reference_name':'Scripted Reference Command'}
(OUT/'summary.json').write_text(json.dumps(summary,indent=2))
print(json.dumps(summary,indent=2))
