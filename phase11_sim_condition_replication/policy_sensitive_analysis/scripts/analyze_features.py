#!/usr/bin/env python3
"""Phase 11 paired representation and train/validation low-rank analysis."""
import csv,json
from collections import defaultdict
from pathlib import Path
import numpy as np
import matplotlib.pyplot as plt
from scipy.stats import spearmanr, pearsonr

ROOT=Path(__file__).resolve().parents[1]; FEAT=ROOT/'02_features'; REP=ROOT/'04_representation_gap'; POL=ROOT/'05_policy_sensitive'; FIG=ROOT/'09_figures'
for p in (REP,POL,FIG): p.mkdir(exist_ok=True)
LAYERS={'openvla':['vision_backbone.output','projector.output'],'oft':['vision_backbone.output','projector.output','action_hidden_states.input']}
PRIMARY={'openvla':'projector.output','oft':'action_hidden_states.input'}
KS=[1,2,4,8,16,32,64,128]

def write(path,rows):
 with path.open('w',newline='') as f:
  w=csv.DictWriter(f,fieldnames=rows[0].keys());w.writeheader();w.writerows(rows)
def index(model,cond):
 root=FEAT/model/cond; x=json.loads((root/'feature_manifest.json').read_text()); out={}
 for r in x['records']: out[str(Path(r['source_image']).resolve())]=root/r['feature_file']
 return out
def pooled(path,layer):
 with np.load(path,allow_pickle=True) as z:
  a=np.asarray(z[layer],dtype=np.float32)
 return a.reshape(-1,a.shape[-1]).mean(0).astype(np.float64)
def basis_pca(x,k):
 z=x-x.mean(0); gram=z@z.T; val,u=np.linalg.eigh(gram); order=np.argsort(val)[::-1]; u=u[:,order]; val=val[order]
 keep=np.where(val>1e-12)[0][:k]
 return np.empty((x.shape[1],0)) if not len(keep) else (z.T@u[:,keep])/np.sqrt(val[keep])[None,:]
def basis_cov(x,y,k):
 c=(x-x.mean(0)).T@(y-y.mean(0)); u,s,_=np.linalg.svd(c,full_matrices=False); return u[:,:min(k,u.shape[1],int((s>1e-12).sum()))]
def score(x,b): return np.zeros(len(x)) if b.shape[1]==0 else np.linalg.norm(x@b,axis=1)

aligned=list(csv.DictReader((ROOT/'03_alignment/aligned_samples.csv').open()))
actions=list(csv.DictReader((ROOT/'06_action_gap/frame_level_action_metrics.csv').open()))
akey={(r['model'],r['condition'],r['episode_id'],int(r['baseline_index']),int(r['condition_index'])):r for r in actions}
indices={(m,c):index(m,c) for m in LAYERS for c in ('baseline','lighting_low','extra_object','distractor_swap')}
frame=[]; primary={m:[] for m in LAYERS}
for a in aligned:
 cond=a['condition']; eid=a['episode_id']; bi=int(a['baseline_index']);ci=int(a['condition_index']); bp=str(Path(a['baseline_image']).resolve());cp=str(Path(a['condition_image']).resolve())
 for model,layers in LAYERS.items():
  ar=akey[(model,cond,eid,bi,ci)]
  for layer in layers:
   b=pooled(indices[(model,'baseline')][bp],layer); c=pooled(indices[(model,cond)][cp],layer); d=c-b
   row={'model':model,'layer':layer,'condition':cond,'episode_id':eid,'phase':a['phase'],'baseline_index':bi,'condition_index':ci,'feature_l2':float(np.linalg.norm(d)),'feature_cosine_distance':float(1-np.dot(b,c)/(max(np.linalg.norm(b)*np.linalg.norm(c),1e-15))),'relative_shift':float(np.linalg.norm(d)/max(np.linalg.norm(b),1e-15)),'action_shift_standardized_l2':float(ar['condition_shift_standardized_l2'])}
   frame.append(row)
   if layer==PRIMARY[model]: primary[model].append((row,d,np.array([float(ar[f'condition_shift_standardized_d{j}']) for j in range(7)])))
write(REP/'feature_frame_metrics.csv',frame)
summary=[]
for key in sorted({(r['model'],r['layer'],r['condition']) for r in frame}):
 ss=[r for r in frame if (r['model'],r['layer'],r['condition'])==key]
 for metric in ('feature_l2','feature_cosine_distance','relative_shift'):
  pass
 summary.append({'model':key[0],'layer':key[1],'condition':key[2],'n':len(ss),**{m+'_mean':float(np.mean([r[m] for r in ss])) for m in ('feature_l2','feature_cosine_distance','relative_shift')}})
write(REP/'feature_condition_summary.csv',summary)

rng=np.random.default_rng(20261001); sweep=[]; policy_frames=[]
for model,items in primary.items():
 X=np.stack([x[1] for x in items]); Y=np.stack([x[2] for x in items]); meta=[x[0] for x in items]; target=np.linalg.norm(Y,axis=1)
 tr=np.array([r['episode_id']!='episode_000005' for r in meta]); va=~tr
 center=X[tr].mean(0); Z=X-center
 for k in KS:
  bases={'random':np.linalg.qr(rng.normal(size=(X.shape[1],min(k,X.shape[1]))))[0], 'pca_train':basis_pca(X[tr],k), 'train_selected':basis_cov(X[tr],Y[tr],k), 'oracle_validation':basis_cov(X[va],Y[va],k)}
  for method,b in bases.items():
   s=score(Z[va],b); y=target[va]; pr=pearsonr(s,y).statistic if np.std(s)>0 and np.std(y)>0 else np.nan; sr=spearmanr(s,y).statistic if np.std(s)>0 and np.std(y)>0 else np.nan
   sweep.append({'model':model,'layer':PRIMARY[model],'requested_k':k,'effective_k':b.shape[1],'method':method,'split':'validation_episode_5','n':int(va.sum()),'score_mean':float(s.mean()),'pearson_action_shift':float(pr),'spearman_action_shift':float(sr),'oracle':method.startswith('oracle')})
 bsel=basis_cov(X[tr],Y[tr],7); selected=score(Z,bsel); full=np.linalg.norm(Z,axis=1)
 for i,r in enumerate(meta): policy_frames.append({'model':model,'layer':PRIMARY[model],'condition':r['condition'],'episode_id':r['episode_id'],'phase':r['phase'],'split':'validation' if va[i] else 'train','train_selected_effective_k':bsel.shape[1],'global_centered_gap':float(full[i]),'train_selected_policy_score':float(selected[i]),'action_shift_standardized_l2':float(target[i])})
write(POL/'low_rank_sweep.csv',sweep)
write(POL/'policy_sensitive_frame_metrics.csv',policy_frames)
ps=[]
for key in sorted({(r['model'],r['condition'],r['split']) for r in policy_frames}):
 ss=[r for r in policy_frames if (r['model'],r['condition'],r['split'])==key]
 ps.append({'model':key[0],'condition':key[1],'split':key[2],'n':len(ss),'policy_score_mean':float(np.mean([r['train_selected_policy_score'] for r in ss])),'global_gap_mean':float(np.mean([r['global_centered_gap'] for r in ss])),'action_shift_mean':float(np.mean([r['action_shift_standardized_l2'] for r in ss]))})
write(POL/'policy_sensitive_condition_summary.csv',ps)
for model in LAYERS:
 ss=[r for r in sweep if r['model']==model and r['method'] in ('random','pca_train','train_selected')]
 plt.figure(figsize=(8,4))
 for method in ('random','pca_train','train_selected'):
  q=[r for r in ss if r['method']==method]; plt.plot([r['requested_k'] for r in q],[r['spearman_action_shift'] for r in q],marker='o',label=method)
 plt.xscale('log',base=2);plt.xlabel('requested rank k');plt.ylabel('validation Spearman');plt.title(f'{model}: representation score vs action shift');plt.legend();plt.tight_layout();plt.savefig(FIG/f'{model}_low_rank_k_curve.png',dpi=160);plt.close()
report={'status':'COMPLETED_WITH_FEATURES','feature_pairs':len(frame),'primary_layers':PRIMARY,'split':{'train':['episode_000001','episode_000002','episode_000003','episode_000004'],'validation':['episode_000005']},'warning':'OpenVLA action-hidden hook is unavailable; projector.output is an upstream proxy. Covariance-selected policy subspace has at most action dimension 7, so requested k>7 saturates.'}
(POL/'summary.json').write_text(json.dumps(report,indent=2));print(json.dumps(report,indent=2))
