#!/usr/bin/env python3
import csv
from pathlib import Path
import numpy as np
from PIL import Image
import matplotlib.pyplot as plt
H=Path(__file__).resolve().parents[1]; out=H/'09_figures/roi_validation';out.mkdir(parents=True,exist_ok=True)
rows=list(csv.DictReader((H/'heatmap_inventory.csv').open()));seen=set()
colors={'target_orange':'orange','blue_cube':'blue','red_cube':'red','yellow_cube':'yellow'}
for r in rows:
 key=(r['condition'],r['episode_id'],r['frame_index'],r['domain']);
 if key in seen:continue
 seen.add(key);im=Image.open(r['image']).convert('RGB');p=H/'06_roi_masks'/f"{r['condition']}__{r['episode_id']}__f{int(r['frame_index']):03d}__{r['domain']}.npz";z=np.load(p)
 fig,ax=plt.subplots(figsize=(8,4.5));ax.imshow(im)
 for name,color in colors.items():
  mask=z[name].astype(bool)
  if mask.any():ax.contour(mask.astype(float),levels=[.5],colors=[color],linewidths=1.4)
 ax.set_title(f"ROI validation: {r['condition']} {r['episode_id']} f{r['frame_index']} {r['domain']}");ax.axis('off');fig.tight_layout();fig.savefig(out/f"{'__'.join(map(str,key))}.png",dpi=130);plt.close(fig)
print({'roi_overlays':len(seen)})
