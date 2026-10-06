#!/usr/bin/env python3
import csv,json,re
from collections import defaultdict
from pathlib import Path
import numpy as np
from PIL import Image
from scipy.stats import pearsonr,spearmanr
from scipy.ndimage import binary_opening,binary_closing

H=Path(__file__).resolve().parents[1];P=H.parent
for d in ('06_roi_masks','07_roi_metrics','08_method_agreement'):(H/d).mkdir(exist_ok=True)
selected=list(csv.DictReader((H/'01_selected_frames/selected_frames.csv').open())); actions=list(csv.DictReader((P/'06_action_gap/frame_level_action_metrics.csv').open()))
akey={(r['model'],r['condition'],r['episode_id'],int(r['baseline_index']),int(r['condition_index'])):r for r in actions}
pat=re.compile(r'(.+)__(episode_\d+)__f(\d+)__(baseline|condition)__(local_mean|gaussian_blur)\.npz$')
def write(path,rows):
 with path.open('w',newline='') as f:w=csv.DictWriter(f,fieldnames=rows[0].keys());w.writeheader();w.writerows(rows)
def masks(image):
 rgb=np.asarray(image.convert('RGB')).astype(int); rr,gg,bb=rgb[...,0],rgb[...,1],rgb[...,2]
 # Camera-calibrated color masks; geometry/manual masks are deliberately not invented.
 mm={
  'target_orange':(rr>=150)&(gg>=55)&(gg<=190)&(bb<=90)&(rr>=gg+35),
  # Non-target cube colors overlap robot highlights/reflections in this camera.
  # Keep them empty rather than publish unreliable ROI statistics.
  'blue_cube':np.zeros(rr.shape,dtype=bool),
  'red_cube':np.zeros(rr.shape,dtype=bool),
  'yellow_cube':np.zeros(rr.shape,dtype=bool),
 }
 for k in mm:mm[k]=binary_closing(binary_opening(mm[k],iterations=1),iterations=2)
 union=np.logical_or.reduce(list(mm.values()));mm['table_background']=~union
 return mm
def up(a,w,h):return np.asarray(Image.fromarray(a.astype('float32')).resize((w,h),Image.Resampling.BILINEAR))
inventory=[]; roi=[]; store={}
for p in sorted((H/'05_occlusion').glob('*/*.npz')):
 m=pat.match(p.name); assert m,p;cond,eid,idx,domain,method=m.groups();model=p.parent.name;idx=int(idx)
 cand=[r for r in selected if r['model']==model and r['condition']==cond and r['episode_id']==eid and int(r[domain+'_index'])==idx and r['high_cost'].lower()=='true'];assert cand,(p,cand);r=cand[0];image_path=Path(r[domain+'_image']);im=Image.open(image_path).convert('RGB');w,h=im.size;z=np.load(p);maps=z['maps'];assert np.isfinite(maps).all();assert maps.shape[1]==4
 rec={'model':model,'condition':cond,'episode_id':eid,'frame_index':idx,'phase':r['phase'],'domain':domain,'method':'occlusion_'+method,'chunk_count':maps.shape[0],'components':'translation;rotation;gripper;normalized_total','patch_size':int(z['patch_size']),'stride':int(z['stride']),'image':str(image_path),'heatmap_file':str(p),'valid':True,'executed_action':None};inventory.append(rec);store[(model,cond,eid,idx,domain,method)]=maps
 mm=masks(im)
 np.savez_compressed(H/'06_roi_masks'/f'{cond}__{eid}__f{idx:03d}__{domain}.npz',**{k:v.astype('uint8') for k,v in mm.items()})
 for chunk in range(maps.shape[0]):
  for ci,component in enumerate(('translation','rotation','gripper','normalized_total')):
   heat=up(maps[chunk,ci],w,h);total=max(float(heat.sum()),1e-15);top=heat>=np.quantile(heat,.9);peak=np.unravel_index(np.argmax(heat),heat.shape)
   for name,mask in mm.items():
    mass=float(heat[mask].sum()/total);density=float(heat[mask].mean()) if mask.any() else np.nan
    roi.append({**{k:rec[k] for k in ('model','condition','episode_id','frame_index','phase','domain','method')},'chunk_index':chunk,'component':component,'roi':name,'roi_pixels':int(mask.sum()),'attribution_mass':mass,'attribution_density':density,'top10_overlap':float((top&mask).sum()/max(top.sum(),1)),'pointing_hit':int(bool(mask[peak]))})
write(H/'heatmap_inventory.csv',inventory);write(H/'07_roi_metrics/roi_attribution_metrics.csv',roi)
# Agreement between the two non-black intervention variants.
agree=[]
for key in sorted({k[:-1] for k in store}):
 a=store.get(key+('local_mean',));b=store.get(key+('gaussian_blur',));
 if a is None or b is None:continue
 for ch in range(a.shape[0]):
  for ci,comp in enumerate(('translation','rotation','gripper','normalized_total')):
   x=a[ch,ci].ravel();y=b[ch,ci].ravel();cos=float(x@y/max(np.linalg.norm(x)*np.linalg.norm(y),1e-15));pr=float(pearsonr(x,y).statistic) if np.std(x)>0 and np.std(y)>0 else np.nan;sr=float(spearmanr(x,y).statistic) if np.std(x)>0 and np.std(y)>0 else np.nan
   px=x/max(x.sum(),1e-15);py=y/max(y.sum(),1e-15);mid=(px+py)/2
   mx=px>0;my=py>0;js=.5*np.sum(px[mx]*np.log(px[mx]/np.maximum(mid[mx],1e-15)))+.5*np.sum(py[my]*np.log(py[my]/np.maximum(mid[my],1e-15)))
   agree.append({'model':key[0],'condition':key[1],'episode_id':key[2],'frame_index':key[3],'domain':key[4],'chunk_index':ch,'component':comp,'pearson':pr,'spearman':sr,'cosine_similarity':cos,'js_divergence':float(js)})
write(H/'08_method_agreement/method_agreement.csv',agree)
# Chunk and lighting false-close summaries from condition local-mean target/background mass.
chunk=[]
for key in sorted({(r['model'],r['condition'],r['chunk_index'],r['component'],r['roi']) for r in roi if r['domain']=='condition'}):
 ss=[r for r in roi if (r['model'],r['condition'],r['chunk_index'],r['component'],r['roi'])==key and r['domain']=='condition'];chunk.append({'model':key[0],'condition':key[1],'chunk_index':key[2],'component':key[3],'roi':key[4],'n':len(ss),'mass_mean':float(np.mean([x['attribution_mass'] for x in ss])),'density_mean':float(np.mean([x['attribution_density'] for x in ss]))})
write(H/'07_roi_metrics/oft_chunk_heatmap_summary.csv',[r for r in chunk if r['model']=='oft'])
lf=[r for r in roi if r['model']=='oft' and r['condition']=='lighting_low' and r['domain']=='condition' and r['component']=='gripper'];write(H/'07_roi_metrics/lighting_false_close_heatmap_summary.csv',lf)
# Heatmap-effect/action-shift association at selected samples.
corrrows=[]
for model in ('openvla','oft'):
 for cond in ('distractor_swap','extra_object','lighting_low'):
  ss=[]
  for inv in inventory:
   if inv['model']==model and inv['condition']==cond and inv['domain']=='condition' and inv['method']=='occlusion_local_mean':
    sel=next(x for x in selected if x['model']==model and x['condition']==cond and x['episode_id']==inv['episode_id'] and int(x['condition_index'])==int(inv['frame_index'])); ar=akey[(model,cond,inv['episode_id'],int(sel['baseline_index']),int(sel['condition_index']))]; z=np.load(inv['heatmap_file'])['maps'];ss.append((float(z[:,3].mean()),float(ar['condition_shift_standardized_l2'])))
  x=np.array([z[0] for z in ss]);y=np.array([z[1] for z in ss]);corrrows.append({'model':model,'condition':cond,'n':len(ss),'pearson':float(pearsonr(x,y).statistic) if len(x)>2 and np.std(x)>0 and np.std(y)>0 else np.nan,'spearman':float(spearmanr(x,y).statistic) if len(x)>2 and np.std(x)>0 and np.std(y)>0 else np.nan})
write(H/'08_method_agreement/heatmap_action_shift_correlation.csv',corrrows)
print(json.dumps({'inventory':len(inventory),'roi_rows':len(roi),'agreement_rows':len(agree),'invalid':0},indent=2))
