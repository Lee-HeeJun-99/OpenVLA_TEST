#!/usr/bin/env python3
"""Create deterministic Phase 11 image lists; does not load a model or ROS."""
import json
from pathlib import Path

P11=Path('/home/ubuntu/a0509_vla_linux_field_bundle_20260903/lhj/phase11_sim_condition_replication')
OUT=P11/'policy_sensitive_analysis/02_features/inputs'; OUT.mkdir(parents=True,exist_ok=True)
manifest={}
for cond in ('baseline','lighting_low','extra_object','distractor_swap'):
 images=[]
 for n in range(1,6):
  ep=P11/f'real_dataset/{cond}/episodes/episode_{n:06d}/images/primary'
  images += sorted(ep.glob('*.jpg'))
 path=OUT/f'{cond}_images.txt'; path.write_text('\n'.join(map(str,images))+'\n')
 manifest[cond]={'count':len(images),'list_file':str(path)}
(OUT/'manifest.json').write_text(json.dumps({'instruction':'Pick up the orange cube.','conditions':manifest},indent=2))
print(json.dumps(manifest,indent=2))
