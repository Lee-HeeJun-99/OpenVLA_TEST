"""Offline, same-backend BF16 action-head comparison for A/B/C1/C2."""
import csv, json, sys, hashlib
from collections import Counter
from pathlib import Path

import numpy as np
import torch

HERE = Path(__file__).resolve().parent
LHJ = HERE.parents[1]
ROOT = LHJ.parent
OLD = LHJ / 'experiments/20261028_componentwise_mean_loo/results_001'
CK = ROOT / 'runtime_state/oft_mixed480_step28560_merged'
BASE = LHJ / 'phase6_environment_attribution/01_preprocessing/full_forward_features'
CONDITIONS = {'A':'P0_current_paired_image','B':'P5_brightness_match_sim_to_real'}
KEY = 'action_hidden_states.input'

sys.path.insert(0, str(LHJ / 'scripts'))
from action_hidden_token_sensitivity import load_action_head, unnormalize_np

def read(path): return json.loads(Path(path).read_text())
def write_csv(path, rows):
    with path.open('w', newline='') as f:
        fields=list(dict.fromkeys(k for r in rows for k in r))
        w = csv.DictWriter(f, fieldnames=fields); w.writeheader(); w.writerows(rows)
def get_records(condition, domain):
    p = BASE / CONDITIONS[condition] / domain / 'feature_manifest.json'
    records = read(p)['records']
    result = {Path(r['source_image']).stem: (r, p.parent / r['feature_file']) for r in records}
    assert len(records) == len(result) == 225
    return result
def load(record, key):
    with np.load(record[1]) as z: return z[key].copy()
def metric(real, sim):
    d = np.asarray(real, np.float64) - np.asarray(sim, np.float64)
    assert d.shape == (5,7) and np.isfinite(d).all()
    return dict(translation_l2=float(np.linalg.norm(d[:,:3],axis=1).mean()),
                rotation_l2=float(np.linalg.norm(d[:,3:6],axis=1).mean()),
                gripper_abs=float(np.abs(d[:,6]).mean()))
def aggregate(rows, keys):
    groups = {}
    for r in rows: groups.setdefault(tuple(r[k] for k in keys), []).append(r)
    out=[]
    for group, rs in groups.items():
        row=dict(zip(keys,group));row['pair_count']=len(rs)
        for k in ('pixel_mse','translation_l2','rotation_l2','gripper_abs','feature_gap_before','feature_gap_after'):
            if k in rs[0]: row[k]=float(np.mean([r[k] for r in rs]))
        out.append(row)
    return out

def main():
    out=HERE/'results_001';out.mkdir(exist_ok=True)
    assert not any(out.iterdir()), 'Results directory must be empty'
    assert not torch.cuda.is_available(), 'This run is explicitly recorded as CPU BF16'
    torch.set_num_threads(4)
    head, device, headpath = load_action_head(CK)
    head=head.to(dtype=torch.bfloat16).eval().requires_grad_(False)
    stats=read(CK/'dataset_statistics.json')['a0509_sim_cube_pick']['action']
    assert stats['mask']==[True]*6+[False]
    def predict(h):
        assert h.shape==(1,35,4096) and np.isfinite(h).all()
        with torch.inference_mode():
            norm=head.predict_action(torch.as_tensor(h,device=device,dtype=torch.bfloat16)).reshape(5,7).float().cpu().numpy()
        return norm, unnormalize_np(norm,stats)
    pairs=read(ROOT/'outputs/token_distribution_analysis/5_episodes/paired_inputs/paired_manifest.json')['pairs']
    ids=[p['pair_id'] for p in pairs];eps=[p['episode_id'] for p in pairs]
    assert len(ids)==len(set(ids))==225 and len(set(eps))==5
    records={c:{d:get_records(c,d) for d in ('real','sim')} for c in CONDITIONS}
    for c in CONDITIONS:
        for d in ('real','sim'): assert set(records[c][d])==set(ids)
    cached={r['condition']:r for r in csv.DictReader((OLD/'overall_metrics.csv').open()) if r['method']=='none'}
    pixel={}
    for r in csv.DictReader((OLD/'cached_observation_metrics.csv').open()): pixel[r['condition'],r['pair_id']]=float(r['pixel_mse_model_rgb'])
    h={};actions={};replay=[]
    for c in CONDITIONS:
        h[c]={};actions[c]={}
        for d in ('real','sim'):
            hs=[];aa=[]
            for pid in ids:
                rec=records[c][d][pid];x=load(rec,KEY)
                assert x.shape==(1,35,4096) and x.dtype==np.float32
                norm,act=predict(x);saved=load(rec,'action_head.output').reshape(5,7)
                replay.append(float(np.max(abs(norm-saved))))
                hs.append(x[0]);aa.append(act)
            h[c][d]=np.stack(hs);actions[c][d]=np.stack(aa)
            print('replay',c,d,len(aa),'max error',max(replay),flush=True)
    rows=[]
    for c in CONDITIONS:
        for i,pid in enumerate(ids):
            rows.append(dict(condition=c,pair_id=pid,episode_id=eps[i],pixel_mse=pixel[c,pid],
                             **metric(actions[c]['real'][i],actions[c]['sim'][i])))
    old_vectors=np.load(OLD/'mean_correction_vectors.npz')
    folds=[];vectors={};checks=[]
    for ep in sorted(set(eps)):
        train=np.array([i for i,e in enumerate(eps) if e!=ep]);test=np.array([i for i,e in enumerate(eps) if e==ep])
        assert not set(train)&set(test) and len(train)+len(test)==225
        delta=np.mean(h['A']['real'][train].astype(np.float64)-h['A']['sim'][train].astype(np.float64),axis=0)
        assert delta.shape==(35,4096) and np.isfinite(delta).all()
        old=old_vectors['A_'+ep]
        sign_error=float(np.max(np.abs(delta+old)))
        assert sign_error<1e-12
        vectors[ep]=delta
        folds.append(dict(held_out_episode=ep,train_pair_ids=[ids[i] for i in train],test_pair_ids=[ids[i] for i in test],
                          train_count=len(train),test_count=len(test),delta_shape=list(delta.shape),
                          sign_error_vs_existing_C1=sign_error))
        for i in test:
            r=h['A']['real'][i:i+1];s=h['A']['sim'][i:i+1]
            _,c1=predict((r-delta).astype(np.float32))
            _,c2=predict((s+delta).astype(np.float32))
            before=float(np.linalg.norm((r-s).astype(np.float64)))
            after=float(np.linalg.norm((r-s-delta).astype(np.float64)))
            for condition,real,sim in [('C1',c1,actions['A']['sim'][i]),('C2',actions['A']['real'][i],c2)]:
                rows.append(dict(condition=condition,pair_id=ids[i],episode_id=ep,pixel_mse=pixel['A',ids[i]],
                                 feature_gap_before=before,feature_gap_after=after,**metric(real,sim)))
        print('fold',ep,len(train),len(test),flush=True)
    for c in ('A','B','C1','C2'):
        assert {r['pair_id'] for r in rows if r['condition']==c}==set(ids)
        assert len([r for r in rows if r['condition']==c])==225
    zero_errors=[]
    for i in (0,46,93,137,182):
        _,z=predict(h['A']['sim'][i:i+1].astype(np.float64))
        zero_errors.append(float(np.max(abs(z-actions['A']['sim'][i]))))
    assert max(zero_errors)==0
    overall=aggregate(rows,['condition']);episodes=aggregate(rows,['condition','episode_id'])
    for r in overall:
        a=next(x for x in overall if x['condition']=='A')
        for k in ('translation_l2','rotation_l2','gripper_abs'):
            r[k+'_change_percent']=100*(r[k]-a[k])/a[k]
    for r in episodes:
        a=next(x for x in episodes if x['condition']=='A' and x['episode_id']==r['episode_id'])
        for k in ('translation_l2','rotation_l2','gripper_abs'):
            r[k+'_change_percent']=100*(r[k]-a[k])/a[k]
    write_csv(out/'metrics.csv',overall);write_csv(out/'episode_metrics.csv',episodes)
    write_csv(out/'observation_metrics.csv',rows)
    (out/'folds.json').write_text(json.dumps(folds,indent=2)+'\n')
    np.savez_compressed(out/'correction_vectors.npz',**vectors)
    (out/'validation.json').write_text(json.dumps(dict(backend='CPU',dtype='bfloat16',batch_size=1,
        torch_version=torch.__version__,checkpoint=headpath,checkpoint_sha256=hashlib.sha256(Path(headpath).read_bytes()).hexdigest(),
        replay_max_normalized_error=max(replay),replay_exact_count=sum(x==0 for x in replay),replay_count=len(replay),
        zero_vector_max_decoded_error=max(zero_errors),pixel_identity_C1_C2_A=all(r['pixel_mse']==pixel['A',r['pair_id']] for r in rows if r['condition'] in ('C1','C2')),
        head_frozen=not any(p.requires_grad for p in head.parameters()),sign_max_error=max(f['sign_error_vs_existing_C1'] for f in folds)),indent=2)+'\n')
    print(json.dumps(overall,indent=2),flush=True)

if __name__=='__main__':main()
