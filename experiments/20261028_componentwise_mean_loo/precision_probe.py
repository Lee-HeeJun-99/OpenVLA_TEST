"""Offline head-only replay probe; never imports robot interfaces."""
import os,sys,json
from pathlib import Path
import numpy as np
import torch
ROOT=Path(__file__).resolve().parents[3]
sys.path.insert(0,str(ROOT/'lhj/scripts'))
from action_hidden_token_sensitivity import load_action_head,unnormalize_np
CK=ROOT/'runtime_state/oft_mixed480_step28560_merged'
head,device,_=load_action_head(CK)
stats=json.loads((CK/'dataset_statistics.json').read_text())['a0509_sim_cube_pick']['action']
p=ROOT/'lhj/phase6_environment_attribution/01_preprocessing/full_forward_features/P0_current_paired_image/real/feature_manifest.json'
r=json.loads(p.read_text())['records'][0]
with np.load(p.parent/r['feature_file']) as z:
    h=z['action_hidden_states.input'];stored=z['action_head.output']
print('device',device,'hidden_shape',h.shape,'stored_norm',stored.reshape(-1)[:7],flush=True)
for name,dtype in [('float32',torch.float32),('bfloat16',torch.bfloat16)]:
    head=head.to(dtype=dtype)
    with torch.inference_mode():pred=head.predict_action(torch.as_tensor(h,device=device,dtype=dtype)).float().cpu().numpy()
    decoded=unnormalize_np(pred.reshape(5,7),stats)
    saved=np.asarray(r['response']['actions'])
    print(json.dumps(dict(dtype=name,max_abs_normalized_error=float(np.max(abs(pred-stored))),max_abs_decoded_error=float(np.max(abs(decoded-saved))),norm_pred=pred.reshape(-1)[:7].tolist())),flush=True)
