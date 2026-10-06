#!/usr/bin/env python3
"""Prediction-only occlusion sensitivity on preselected frames; no ROS imports."""
import argparse,base64,csv,io,json,time
from pathlib import Path
from urllib.request import Request,urlopen
import numpy as np
from PIL import Image,ImageFilter
import matplotlib.pyplot as plt

H=Path(__file__).resolve().parents[1]; P=H.parent
ap=argparse.ArgumentParser();ap.add_argument('--model',choices=('openvla','oft'),required=True);ap.add_argument('--server-url',required=True);ap.add_argument('--patch-size',type=int,default=128);ap.add_argument('--stride',type=int,default=96);a=ap.parse_args()
rows=[r for r in csv.DictReader((H/'01_selected_frames/selected_frames.csv').open()) if r['model']==a.model and r['high_cost'].lower()=='true']
stats=Path('/home/ubuntu/a0509_vla_linux_field_bundle_20260903/models')/('vanilla_s1_balanced_step8130' if a.model=='openvla' else 'oft_mixed480_step28560')/'dataset_statistics.json'
scale=np.asarray(json.loads(stats.read_text())['a0509_sim_cube_pick']['action']['std'],float)
out=H/'05_occlusion'/a.model;out.mkdir(parents=True,exist_ok=True); figdir=H/'09_figures'/'occlusion'/a.model;figdir.mkdir(parents=True,exist_ok=True)
def predict(im):
 b=io.BytesIO();im.save(b,format='JPEG',quality=95)
 payload={'instruction':'Pick up the orange cube.','image_jpeg_base64':base64.b64encode(b.getvalue()).decode(),'variant':'openvla_token' if a.model=='openvla' else 'oftplus_h5_vision'}
 req=Request(a.server_url.rstrip('/')+'/predict',data=json.dumps(payload).encode(),headers={'Content-Type':'application/json'},method='POST')
 with urlopen(req,timeout=60) as z:x=json.loads(z.read())
 return np.asarray(x['actions'] if a.model=='oft' else [x['action']],float)
def replacement(im,box,kind,blur):
 z=im.copy(); crop=np.asarray(im.crop(box))
 if kind=='local_mean': fill=Image.fromarray(np.broadcast_to(crop.mean((0,1),keepdims=True).astype('uint8'),crop.shape).copy())
 else: fill=blur.crop(box)
 z.paste(fill,box);return z
inventory=[]
for r in rows:
 for domain in ('baseline','condition'):
  image_path=Path(r[domain+'_image']); stem=f"{r['condition']}__{r['episode_id']}__f{int(r[domain+'_index']):03d}__{domain}"
  im=Image.open(image_path).convert('RGB'); w,h=im.size; ys=list(range(0,h,a.stride));xs=list(range(0,w,a.stride)); blur=im.filter(ImageFilter.GaussianBlur(radius=16)); original=predict(im)
  for kind in ('local_mean','gaussian_blur'):
   dst=out/f'{stem}__{kind}.npz'
   if dst.exists(): continue
   maps=np.zeros((len(original),4,len(ys),len(xs)),np.float32);started=time.time();count=0
   for iy,y in enumerate(ys):
    for ix,x in enumerate(xs):
     box=(x,y,min(x+a.patch_size,w),min(y+a.patch_size,h)); q=predict(replacement(im,box,kind,blur));d=q-original;count+=1
     maps[:,0,iy,ix]=np.linalg.norm(d[:,:3],axis=1);maps[:,1,iy,ix]=np.linalg.norm(d[:,3:6],axis=1);maps[:,2,iy,ix]=np.abs(d[:,6]);maps[:,3,iy,ix]=np.linalg.norm(d/scale,axis=1)
   np.savez_compressed(dst,maps=maps,original_action=original,image_shape=np.array([h,w]),patch_size=a.patch_size,stride=a.stride,components=np.array(['translation','rotation','gripper','normalized_total']))
   vmax=float(maps[:,3].max()); fig,ax=plt.subplots(1,len(original),figsize=(4*len(original),4),squeeze=False)
   for k in range(len(original)):
    heat=np.asarray(Image.fromarray(maps[k,3]).resize((w,h),Image.Resampling.BILINEAR));ax[0,k].imshow(im);ax[0,k].imshow(heat,cmap='magma',alpha=.55,vmin=0,vmax=vmax);ax[0,k].set_title(f'{a.model} {r["condition"]} {domain}\nchunk {k} normalized total');ax[0,k].axis('off')
   fig.tight_layout();fig.savefig(figdir/f'{stem}__{kind}.png',dpi=130);plt.close(fig)
   inventory.append({'model':a.model,'condition':r['condition'],'episode_id':r['episode_id'],'frame_index':r[domain+'_index'],'phase':r['phase'],'domain':domain,'method':'occlusion_'+kind,'chunk_count':len(original),'patch_size':a.patch_size,'stride':a.stride,'inference_count':count+1,'seconds':time.time()-started,'image':str(image_path),'heatmap_file':str(dst),'fixture':False,'executed_action':None})
inv=H/'heatmap_inventory_'+a.model+'.csv'
old=list(csv.DictReader(inv.open())) if inv.exists() else []
allrows=old+inventory
if allrows:
 with inv.open('w',newline='') as f:wri=csv.DictWriter(f,fieldnames=allrows[0].keys());wri.writeheader();wri.writerows(allrows)
print(json.dumps({'model':a.model,'selected_high_cost':len(rows),'new_heatmaps':len(inventory),'robot_commands':0},indent=2))
