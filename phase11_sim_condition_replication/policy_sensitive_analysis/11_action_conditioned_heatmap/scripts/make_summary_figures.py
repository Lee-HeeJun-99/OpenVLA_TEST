#!/usr/bin/env python3
import csv
from pathlib import Path
import numpy as np
from PIL import Image
import matplotlib.pyplot as plt
H=Path(__file__).resolve().parents[1];F=H/'09_figures/summary';F.mkdir(parents=True,exist_ok=True)
inv=list(csv.DictReader((H/'heatmap_inventory.csv').open()));roi=list(csv.DictReader((H/'07_roi_metrics/roi_attribution_metrics.csv').open()));agree=list(csv.DictReader((H/'08_method_agreement/method_agreement.csv').open()))
# OFT lighting false-close: all five chunk gripper intervention maps, shared scale.
r=next(x for x in inv if x['model']=='oft' and x['condition']=='lighting_low' and x['domain']=='condition' and x['method']=='occlusion_local_mean');z=np.load(r['heatmap_file']);maps=z['maps'][:,2];im=Image.open(r['image']).convert('RGB');w,h=im.size;vmax=float(maps.max());fig,ax=plt.subplots(1,5,figsize=(18,4))
for k in range(5):
 heat=np.asarray(Image.fromarray(maps[k]).resize((w,h),Image.Resampling.BILINEAR));ax[k].imshow(im);ax[k].imshow(heat,cmap='magma',alpha=.55,vmin=0,vmax=vmax);ax[k].set_title(f'OFT lighting gripper k={k}');ax[k].axis('off')
fig.tight_layout();fig.savefig(F/'oft_lighting_false_close_chunks.png',dpi=150);plt.close(fig)
# ROI density ratio by condition, OFT gripper/local mean.
conds=['distractor_swap','extra_object','lighting_low'];vals=[]
for c in conds:
 groups={}
 for x in roi:
  if x['model']=='oft' and x['condition']==c and x['domain']=='condition' and x['method']=='occlusion_local_mean' and x['component']=='gripper':groups.setdefault((x['episode_id'],x['frame_index'],x['chunk_index']),{})[x['roi']]=float(x['attribution_density'])
 vals.append(np.mean([q['target_orange']/q['table_background'] for q in groups.values() if q.get('target_orange')==q.get('target_orange') and q.get('table_background',0)>0]))
plt.figure(figsize=(7,4));plt.bar(conds,vals);plt.ylabel('target/background attribution density ratio');plt.title('OFT gripper occlusion sensitivity by condition');plt.tight_layout();plt.savefig(F/'oft_gripper_target_background_ratio.png',dpi=150);plt.close()
# Occlusion replacement-method agreement.
for model in ('openvla','oft'):
 comps=['translation','rotation','gripper','normalized_total'];v=[np.nanmean([float(x['pearson']) for x in agree if x['model']==model and x['component']==c]) for c in comps];plt.figure(figsize=(7,4));plt.bar(comps,v);plt.ylim(-.1,1);plt.ylabel('Pearson correlation');plt.title(f'{model}: local-mean vs Gaussian-blur occlusion');plt.xticks(rotation=20);plt.tight_layout();plt.savefig(F/f'{model}_occlusion_method_agreement.png',dpi=150);plt.close()
print({'summary_figures':4})
