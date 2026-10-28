"""Explain historical denominators, never merge these into the primary experiment."""
import sys,json
from pathlib import Path
import numpy as np
import torch
from analyze import ROOT,LHJ,CK,FEATURE,read,js,csvwrite,metrics,arr
sys.path.insert(0,str(LHJ/'scripts'))
from action_hidden_token_sensitivity import load_action_head,predict_batch
out=Path(sys.argv[1]);head,device,_=load_action_head(CK)
stats=read(CK/'dataset_statistics.json')['a0509_sim_cube_pick']['action']
data={}
for d in ('real','sim'):
    p=ROOT/f'outputs/token_distribution_analysis/5_episodes/features/{d}/feature_manifest.json'
    data[d]={Path(r['source_image']).stem:dict(r,_npz=str(p.parent/r['feature_file'])) for r in read(p)['records']}
rows=[]
for pid,r in data['real'].items():
    a=predict_batch(head,arr(r,FEATURE),stats,device)[0]
    s=np.asarray(data['sim'][pid]['response']['actions'])
    rows.append(dict(pair_id=pid,**metrics(a,s)))
csvwrite(out,'legacy_FP32_real_vs_saved_BF16_sim.csv',rows)
js(out,'legacy_precision_explanation.json',dict(pair_count=len(rows),pipeline='original paired images, real head FP32 replay vs original saved BF16 sim',
    measured_action7_l2=float(np.mean([r['action7_l2'] for r in rows])),reported_rounded_baseline=.370245,
    matches_report_at_6_decimals=round(float(np.mean([r['action7_l2'] for r in rows])),6)==.370245))
print((out/'legacy_precision_explanation.json').read_text(),flush=True)
