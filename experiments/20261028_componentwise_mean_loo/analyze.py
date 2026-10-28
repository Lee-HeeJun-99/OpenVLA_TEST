"""Cached, offline component-wise A/B/C comparison and condition-specific mean LOO.

No ROS, model server, backbone inference, training, phase or transfer correction.
Run --stage cached first; --stage head only after inspecting its independent audit.
"""
import argparse,csv,hashlib,json,subprocess,sys
from collections import Counter
from pathlib import Path
import numpy as np
from PIL import Image,ImageDraw

HERE=Path(__file__).resolve().parent
ROOT=HERE.parents[2]
LHJ=ROOT/'lhj'
CK=ROOT/'runtime_state/oft_mixed480_step28560_merged'
FEATURE='action_hidden_states.input'
CONDITIONS={'A':'P0_current_paired_image','B':'P5_brightness_match_sim_to_real','C':'P4_letterbox_224'}
BASE=LHJ/'phase6_environment_attribution/01_preprocessing/full_forward_features'
PAIR=ROOT/'outputs/token_distribution_analysis/5_episodes/paired_inputs/paired_manifest.json'
METRICS=('translation_l2','rotation_l2','gripper_abs','action7_l2','translation_squared','rotation_squared','gripper_squared')

def read(p):return json.loads(Path(p).read_text())
def js(out,name,value):
    (out/name).write_text(json.dumps(value,indent=2,ensure_ascii=False,allow_nan=False)+'\n')
def csvwrite(out,name,rows):
    keys=list(dict.fromkeys(k for r in rows for k in r))
    with (out/name).open('w',newline='') as f:
        w=csv.DictWriter(f,fieldnames=keys);w.writeheader();w.writerows(rows)
def sha(p):
    h=hashlib.sha256()
    with Path(p).open('rb') as f:
        for b in iter(lambda:f.read(1<<20),b''):h.update(b)
    return h.hexdigest()
def manifests(c):
    result={}
    for d in ('real','sim'):
        p=BASE/CONDITIONS[c]/d/'feature_manifest.json';m=read(p)
        rs={Path(r['source_image']).stem:dict(r,_npz=str(p.parent/r['feature_file'])) for r in m['records']}
        assert len(rs)==len(m['records'])==225
        result[d]=rs
    return result
def arr(record,key):
    with np.load(record['_npz']) as z:return z[key].copy()
def actions(record):
    a=np.asarray(record['response']['actions'],dtype=np.float64)
    assert a.shape==(5,7) and np.isfinite(a).all()
    return a
def metrics(real,sim):
    d=np.asarray(real,dtype=np.float64)-np.asarray(sim,dtype=np.float64)
    assert d.shape==(5,7) and np.isfinite(d).all()
    return dict(translation_l2=float(np.linalg.norm(d[:,:3],axis=-1).mean()),
        rotation_l2=float(np.linalg.norm(d[:,3:6],axis=-1).mean()),
        gripper_abs=float(abs(d[:,6]).mean()),action7_l2=float(np.linalg.norm(d,axis=-1).mean()),
        translation_squared=float((d[:,:3]**2).sum(-1).mean()),
        rotation_squared=float((d[:,3:6]**2).sum(-1).mean()),gripper_squared=float((d[:,6]**2).mean()))
def aggregate(rows,keys):
    groups={}
    for r in rows:groups.setdefault(tuple(r[k] for k in keys),[]).append(r)
    result=[]
    for values,rs in groups.items():
        common=[k for k in rs[0] if (k in METRICS or k.startswith('pixel_mse')) and all(k in r for r in rs)]
        x=dict(zip(keys,values));x['pair_count']=len(rs)
        x.update({k:float(np.mean([r[k] for r in rs])) for k in common})
        result.append(x)
    return result
def unit_rgb(tensor,processor):
    # Fixed inverse of the recorded backbone Normalize; NOT min/max visualization scaling.
    images=[]
    for branch in range(2):
        p=processor['tvf_normalize_params'][branch]
        mean=np.asarray(p['mean'])[:,None,None];std=np.asarray(p['std'])[:,None,None]
        images.append((tensor[0,3*branch:3*branch+3].astype(np.float64)*std+mean).transpose(1,2,0))
    return images

def cached(out):
    out.mkdir(parents=True,exist_ok=False)
    pairs=read(PAIR)['pairs'];ids=[p['pair_id'] for p in pairs]
    assert len(ids)==len(set(ids))==225
    counts=Counter(p['episode_id'] for p in pairs);assert len(counts)==5
    stats=read(CK/'dataset_statistics.json');processor=read(CK/'preprocessor_config.json')
    sources=[PAIR,CK/'dataset_statistics.json',CK/'a0509_training_config.json',CK/'preprocessor_config.json',
        CK/'action_head--28560_checkpoint.pt',
        LHJ/'scripts/action_hidden_token_sensitivity.py',
        LHJ/'phase6_environment_attribution/scripts/phase6_prepare_condition_images.py',
        LHJ/'phase6_environment_attribution/scripts/phase6_preprocessing_audit.py',
        ROOT/'runtime/openvla-oft/prismatic/models/action_heads.py',
        ROOT/'runtime/openvla-oft/prismatic/extern/hf/modeling_prismatic.py',
        ROOT/'runtime/openvla-oft/experiments/robot/openvla_utils.py',
        LHJ/'phase5_policy_relevant_completion/scripts/loo_correction_ablation.py']
    audit=dict(branch=subprocess.check_output(['git','branch','--show-current'],cwd=LHJ,text=True).strip(),
        commit=subprocess.check_output(['git','rev-parse','HEAD'],cwd=LHJ,text=True).strip(),
        git_status=subprocess.check_output(['git','status','--short'],cwd=LHJ,text=True),
        checkpoint=str(CK),dataset_key='a0509_sim_cube_pick',dataset_statistics=stats,
        processor=processor,training_config=read(CK/'a0509_training_config.json'),
        pair_count=225,episode_counts=dict(counts),feature=FEATURE,chunk_size=5,action_dim=7,
        inference_required=[],head_replay_required=['A_none','B_none','C_none','A_mean_LOO','C_mean_LOO'],
        cached_reaggregation_available=['A','B','C'],physical_commands=0,
        source_hashes={str(p):sha(p) for p in sources},source_code=str(Path(__file__)),source_code_sha256=sha(__file__))
    (out/'snapshots').mkdir()
    for p in sources:
        if p.suffix in ('.json','.py'):(out/'snapshots'/((p.parent.name+'__'+p.name))).write_bytes(p.read_bytes())
    js(out,'audit.json',audit);js(out,'pair_manifest.json',read(PAIR))
    rows=[];availability={};sample_tensors={};image_audit={};model_cards=[]
    (out/'representative').mkdir()
    canvas=Image.new('RGB',(224*4,270*3+160),(255,255,255));draw=ImageDraw.Draw(canvas)
    for col,domain in enumerate(('real','sim')):
        image=Image.open(pairs[0][domain]['paired_image']).convert('RGB');thumb=image.copy();thumb.thumbnail((420,120))
        canvas.paste(thumb,(col*448,25));draw.text((col*448+5,5),f'Original paired {domain}: {image.size}',fill='black')
        image.save(out/'representative'/f'original_{domain}.png')
    for ci,c in enumerate(CONDITIONS):
        data=manifests(c);availability[c]={}
        for domain in ('real','sim'):
            rs=data[domain];assert set(rs)==set(ids)
            availability[c][domain]=dict(records=len(rs),feature_files_present=sum(Path(r['_npz']).exists() for r in rs.values()),
                input_files_present=sum(Path(r['source_image']).exists() for r in rs.values()),
                shapes=dict(Counter(str(r['features'][FEATURE]['shape']) for r in rs.values())))
            for r in rs.values():
                assert r['instruction']=='Pick up the orange cube.'
                assert r['response']['variant']=='oftplus_h5_vision' and r['response']['chunk_size']==5
        for j,p in enumerate(pairs):
            pid=p['pair_id'];r=data['real'][pid];s=data['sim'][pid]
            rt=arr(r,'vision_backbone.input');st=arr(s,'vision_backbone.input')
            assert rt.shape==st.shape==(1,6,224,224)
            rr=unit_rgb(rt,processor);ss=unit_rgb(st,processor)
            row=dict(condition=c,legacy_condition=CONDITIONS[c],method='none',pair_id=pid,episode_id=p['episode_id'],
                **metrics(actions(r),actions(s)),
                pixel_mse_model_rgb=float(np.mean((rr[0]-ss[0])**2)),
                pixel_mse_model_rgb_branch2=float(np.mean((rr[1]-ss[1])**2)))
            ri=Image.open(r['source_image']).convert('RGB');si=Image.open(s['source_image']).convert('RGB')
            assert ri.size==si.size
            row['pixel_mse_condition_jpeg']=float(np.mean(((np.asarray(ri,dtype=np.float64)-np.asarray(si,dtype=np.float64))/255)**2))
            rows.append(row)
            if j==0:
                image_audit[c]=dict(real_input=str(r['source_image']),sim_input=str(s['source_image']),
                    image_sizes=dict(real=ri.size,sim=si.size),recorded_model_tensor_shape=list(rt.shape),
                    inverse_normalization='Fixed stored processor means/stds, no per-image normalization; PNG only clips to [0,1] and rounds to uint8. MSE uses un-clipped floating RGB.',
                    brightness_gain_sim_to_real=None)
                original_real=Image.open(p['real']['paired_image']).convert('RGB');original_sim=Image.open(p['sim']['paired_image']).convert('RGB')
                l=lambda im:np.asarray(im,dtype=np.float64).dot([.2126,.7152,.0722])/255
                image_audit[c]['brightness_gain_sim_to_real']=float(l(original_real).mean()/max(l(original_sim).mean(),1e-6)) if c=='B' else None
                sample_tensors[c+'_real']=rt;sample_tensors[c+'_sim']=st
                for col,(domain,branch,img) in enumerate([('real',0,rr[0]),('sim',0,ss[0]),('real',1,rr[1]),('sim',1,ss[1])]):
                    pil=Image.fromarray(np.rint(np.clip(img,0,1)*255).astype('uint8'))
                    canvas.paste(pil,(col*224,160+ci*270+25));draw.text((col*224+3,160+ci*270+4),f'{c} {domain} backbone {branch+1}',fill='black')
                    pil.save(out/'representative'/f'{c}_{domain}_model_branch{branch+1}.png')
                ri.save(out/'representative'/f'{c}_real_condition_input.png');si.save(out/'representative'/f'{c}_sim_condition_input.png')
        print('cached',c,'225 pairs',flush=True)
    np.savez_compressed(out/'representative/model_input_tensors.npz',**sample_tensors)
    canvas.save(out/'representative_inputs.png')
    js(out,'availability.json',availability);js(out,'input_processing_evidence.json',image_audit)
    csvwrite(out,'cached_observation_metrics.csv',rows)
    csvwrite(out,'cached_episode_metrics.csv',aggregate(rows,['condition','episode_id']))
    csvwrite(out,'cached_overall_metrics.csv',aggregate(rows,['condition']))
    # Keep the earlier raw-paired-image baseline separate: Phase6 P0 was JPEG re-encoded.
    old={}
    for domain in ('real','sim'):
        p=ROOT/f'outputs/token_distribution_analysis/5_episodes/features/{domain}/feature_manifest.json'
        old[domain]={Path(r['source_image']).stem:r for r in read(p)['records']}
    oldrows=[dict(pair_id=pid,**metrics(actions(old['real'][pid]),actions(old['sim'][pid]))) for pid in ids]
    csvwrite(out,'legacy_original_baseline.csv',oldrows)
    js(out,'legacy_baseline_provenance.json',dict(original_paired_saved_action7_l2=float(np.mean([r['action7_l2'] for r in oldrows])),
        phase6_P0_saved_action7_l2=float(np.mean([r['action7_l2'] for r in rows if r['condition']=='A'])),
        old_mean_report='Phase5 reports FP32 head-recomputed real vs saved BF16 sim (0.370245), not Phase6 JPEG re-encoded P0 saved output (0.370203); no forced numeric reconciliation.',
        baseline_source_images_differ=True))

def replay(out):
    import torch
    sys.path.insert(0,str(LHJ/'scripts'))
    from action_hidden_token_sensitivity import load_action_head,unnormalize_np
    assert (out/'cached_overall_metrics.csv').exists(),'Run cached stage first'
    assert not (out/'head_validation.json').exists(),'Never overwrite a replay'
    pairs=read(PAIR)['pairs'];ids=[p['pair_id'] for p in pairs];meta={p['pair_id']:p for p in pairs}
    eps=sorted(set(p['episode_id'] for p in pairs))
    head,device,headpath=load_action_head(CK)
    assert str(device).startswith('cuda'),'Explicit CUDA execution required; never silently change replay backend'
    head=head.to(dtype=torch.bfloat16);head.requires_grad_(False)
    torch.backends.cuda.matmul.allow_tf32=False
    stats=read(CK/'dataset_statistics.json')['a0509_sim_cube_pick']['action']
    def predict(h):
        # Single-observation batch, identical to cached full-forward request batch.
        with torch.inference_mode():
            a=head.predict_action(torch.as_tensor(h,device=device,dtype=torch.bfloat16)).reshape(5,7).float().cpu().numpy()
        return a,unnormalize_np(a,stats)
    norm_atol=1e-6;decoded_atol=1e-8 # strict replay identity, set before all-sample audit
    all_actions={};validation=[];hiddens={}
    for c in CONDITIONS:
        data=manifests(c);all_actions[c]={};hiddens[c]={}
        for domain in ('real','sim'):
            pred=[];hs=[]
            for pid in ids:
                r=data[domain][pid];h=arr(r,FEATURE);hs.append(h)
                a,d=predict(h);pred.append(d);saved_norm=arr(r,'action_head.output').reshape(5,7)
                validation.append(dict(condition=c,domain=domain,pair_id=pid,
                    max_normalized_abs_error=float(np.max(abs(a-saved_norm))),
                    max_decoded_abs_error=float(np.max(abs(d-actions(r))))))
            all_actions[c][domain]=np.stack(pred)
            hiddens[c][domain]=np.concatenate(hs,axis=0)
        print('replayed',c,'450 cached heads',flush=True)
    passed=all(r['max_normalized_abs_error']<=norm_atol and r['max_decoded_abs_error']<=decoded_atol for r in validation)
    js(out,'head_validation.json',dict(passed=passed,samples=len(validation),dtype='bfloat16',batch_size=1,
        normalized_atol=norm_atol,decoded_atol=decoded_atol,max_normalized_abs_error=max(r['max_normalized_abs_error'] for r in validation),
        max_decoded_abs_error=max(r['max_decoded_abs_error'] for r in validation),
        torch_version=torch.__version__,cuda=torch.version.cuda,device=torch.cuda.get_device_name(0),head_checkpoint=headpath,
        fixed_weights=True,head_sha256=sha(headpath),code_sha256=sha(__file__)))
    csvwrite(out,'head_replay_validation_per_observation.csv',validation)
    np.savez_compressed(out/'replayed_uncorrected_actions.npz',**{f'{c}_{d}':a for c,ds in all_actions.items() for d,a in ds.items()})
    if not passed:
        print('REPLAY VALIDATION FAILED: correction interpretation blocked',flush=True);return
    pixel={}
    with (out/'cached_observation_metrics.csv').open() as f:
        for r in csv.DictReader(f):pixel[r['condition'],r['pair_id']]={k:float(v) for k,v in r.items() if k.startswith('pixel_mse')}
    rows=[];corrected={};folds=[];deltas={};rounding=[]
    for c in CONDITIONS:
        for i,pid in enumerate(ids):
            rows.append(dict(condition=c,method='none',pair_id=pid,episode_id=meta[pid]['episode_id'],fold=meta[pid]['episode_id'],
                **metrics(all_actions[c]['real'][i],all_actions[c]['sim'][i]),**pixel[c,pid]))
        if c not in ('A','C'):continue
        corrected[c]=np.empty((225,5,7))
        for ep in eps:
            train=[i for i,pid in enumerate(ids) if meta[pid]['episode_id']!=ep]
            test=[i for i,pid in enumerate(ids) if meta[pid]['episode_id']==ep]
            assert len(train)+len(test)==225 and not(set(train)&set(test))
            # Original features are exact BF16 values stored as FP32. Estimate elementwise
            # in FP64, then only quantize at injection to original BF16 action head.
            delta=np.mean(hiddens[c]['sim'][train].astype(np.float64)-hiddens[c]['real'][train].astype(np.float64),axis=0)
            deltas[f'{c}_{ep}']=delta
            folds.append(dict(condition=c,held_out_episode=ep,train_pair_ids=[ids[i] for i in train],test_pair_ids=[ids[i] for i in test],
                train_episode_ids=[e for e in eps if e!=ep],train_count=len(train),test_count=len(test),mean_weighting='observation-weighted; not episode-weighted',
                delta_shape=list(delta.shape),delta_l2=float(np.linalg.norm(delta))))
            for i in test:
                injected=hiddens[c]['real'][i].astype(np.float64)+delta
                _,a=predict(injected[None]);corrected[c][i]=a
                rows.append(dict(condition=c,method='mean_LOO',pair_id=ids[i],episode_id=ep,fold=ep,
                    **metrics(a,all_actions[c]['sim'][i]),**pixel[c,ids[i]]))
            print('mean LOO',c,ep,'train',len(train),'test',len(test),flush=True)
    for c,method in [('A','none'),('B','none'),('C','none'),('A','mean_LOO'),('C','mean_LOO')]:
        selected=[r for r in rows if r['condition']==c and r['method']==method]
        assert len(selected)==225 and {r['pair_id'] for r in selected}==set(ids)
        assert len({r['pair_id'] for r in selected})==225
        assert all(r['fold']==r['episode_id'] for r in selected)
    overall=aggregate(rows,['condition','method']);episodes=aggregate(rows,['condition','method','episode_id'])
    csvwrite(out,'observation_metrics.csv',rows);csvwrite(out,'episode_metrics.csv',episodes);csvwrite(out,'overall_metrics.csv',overall)
    js(out,'folds.json',folds);np.savez_compressed(out/'mean_correction_vectors.npz',**deltas)
    np.savez_compressed(out/'corrected_actions.npz',**corrected)
    effects=[]
    for level,source in [('overall',overall),('episode',episodes)]:
        for c in ('A','C'):
            for a in [r for r in source if r['condition']==c and r['method']=='mean_LOO']:
                b=next(r for r in source if r['condition']==c and r['method']=='none' and r.get('episode_id')==a.get('episode_id'))
                for m in METRICS:
                    effects.append(dict(level=level,condition=c,episode_id=a.get('episode_id','ALL'),metric=m,
                        before=b[m],after=a[m],absolute_reduction=b[m]-a[m],reduction_percent=100*(b[m]-a[m])/b[m] if abs(b[m])>1e-9 else None))
    csvwrite(out,'correction_effects.csv',effects)
    table1=[{k:r[k] for k in ('condition','pixel_mse_model_rgb','translation_l2','rotation_l2','gripper_abs')} for r in overall if r['method']=='none']
    table2=[{k:r[k] for k in ('condition','method','translation_l2','rotation_l2','gripper_abs')} for r in overall if r['condition'] in ('A','C')]
    csvwrite(out,'paper_table1.csv',table1);csvwrite(out,'paper_table2.csv',table2)
    js(out,'integrity.json',dict(unique_pairs_per_condition_method=225,total_folds=10,held_out_once=True,
        baseline_recomputed_both_domains=True,no_test_features_in_delta=True,condition_specific_shifts=True,
        no_backbone_inference=True,no_training=True,no_robot_commands=True))
    print(json.dumps(overall,indent=2),flush=True)

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--stage',choices=['cached','head'],required=True);p.add_argument('--output',type=Path,required=True)
    a=p.parse_args();cached(a.output) if a.stage=='cached' else replay(a.output)
