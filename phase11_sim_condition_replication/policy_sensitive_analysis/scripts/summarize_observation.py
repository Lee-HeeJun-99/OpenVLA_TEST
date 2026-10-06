#!/usr/bin/env python3
import csv, statistics
from collections import defaultdict
from pathlib import Path
import matplotlib.pyplot as plt

ROOT=Path(__file__).resolve().parents[1]
rows=list(csv.DictReader((ROOT/'04_representation_gap/observation_frame_metrics.csv').open()))
metrics=['pixel_l1','pixel_l2_rmse','ssim','brightness_delta']
def aggregate(keys):
    groups=defaultdict(list)
    for r in rows: groups[tuple(r[k] for k in keys)].append(r)
    out=[]
    for key,ss in sorted(groups.items()):
        x={k:v for k,v in zip(keys,key)}; x['count']=len(ss)
        for m in metrics:
            vals=[float(r[m]) for r in ss]
            x[m+'_mean']=statistics.mean(vals); x[m+'_std']=statistics.pstdev(vals)
        out.append(x)
    return out
def write(path,rows):
    with path.open('w',newline='') as f:
        w=csv.DictWriter(f,fieldnames=rows[0].keys());w.writeheader();w.writerows(rows)
condition=aggregate(['condition']); phase=aggregate(['condition','phase'])
write(ROOT/'04_representation_gap/observation_condition_summary.csv',condition)
write(ROOT/'04_representation_gap/observation_phase_summary.csv',phase)
figdir=ROOT/'09_figures';figdir.mkdir(exist_ok=True)
for metric,title in [('pixel_l1_mean','Condition visual gap (paired pixel L1)'),('ssim_mean','Condition structural similarity (SSIM)')]:
    plt.figure(figsize=(7,4)); plt.bar([r['condition'] for r in condition],[r[metric] for r in condition]);plt.ylabel(metric);plt.title(title);plt.tight_layout();plt.savefig(figdir/(metric+'.png'),dpi=160);plt.close()
phases=sorted(set(r['phase'] for r in phase)); conds=sorted(set(r['condition'] for r in phase))
mat=[[next(r['pixel_l1_mean'] for r in phase if r['condition']==c and r['phase']==p) for p in phases] for c in conds]
plt.figure(figsize=(9,3.5));plt.imshow(mat,aspect='auto',cmap='magma');plt.xticks(range(len(phases)),phases,rotation=25,ha='right');plt.yticks(range(len(conds)),conds);plt.colorbar(label='pixel L1');plt.title('Observation gap: condition × phase');plt.tight_layout();plt.savefig(figdir/'observation_phase_condition_heatmap.png',dpi=160);plt.close()
