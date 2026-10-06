#!/usr/bin/env python3
"""Read-only Phase 11 integrity, phase alignment, and observation-gap audit."""
from __future__ import annotations

import csv, hashlib, json, math
from collections import Counter, defaultdict
from pathlib import Path
from typing import Any

import numpy as np
from PIL import Image
from scipy.ndimage import gaussian_filter

P11 = Path(__file__).resolve().parents[2]
ROOT = P11 / "policy_sensitive_analysis"
DATA = P11 / "real_dataset"
CONDS = ["baseline", "lighting_low", "extra_object", "distractor_swap"]
PHASES = ["hold", "alignment", "descent_to_grasp", "grasp_close", "lift", "final_hold"]

def read_jsonl(p): return [json.loads(x) for x in p.read_text().splitlines() if x.strip()]
def digest_file(p):
    h=hashlib.sha256()
    with p.open('rb') as f:
        for b in iter(lambda:f.read(1<<20),b''): h.update(b)
    return h.hexdigest()
def write_csv(p, rows):
    p.parent.mkdir(parents=True, exist_ok=True)
    if not rows: p.write_text(''); return
    keys=[]
    for r in rows:
        for k in r:
            if k not in keys: keys.append(k)
    with p.open('w',newline='',encoding='utf-8') as f:
        w=csv.DictWriter(f,fieldnames=keys); w.writeheader(); w.writerows(rows)
def image_path(ep,row):
    q=ep/str(row.get('image',''))
    if q.exists(): return q
    return ep/'images'/'primary'/Path(str(row.get('image',''))).name
def layout(md):
    x=md.get('block_layout') or {}
    return x.get('real_layout_mm') or (x.get('layout') or {}).get('real_layout_mm') or {}
def route(md): return md.get('route_poses_mm_deg') or {}
def flat_action(r):
    a=r.get('action')
    return np.asarray(a,float) if isinstance(a,list) and len(a)==7 else None
def pose(r):
    e=r.get('end_effector_pose') or {}
    p=e.get('position_m'); return np.asarray(p,float) if isinstance(p,list) and len(p)>=3 else None
def phase_progress(rows):
    groups=defaultdict(list)
    for j,r in enumerate(rows): groups[r.get('planner_phase','unknown')].append(j)
    out={}
    for phase,ids in groups.items():
        for rank,j in enumerate(ids): out[j]=0.0 if len(ids)==1 else rank/(len(ids)-1)
    return out,groups
def nearest_align(left,right):
    lp,lg=phase_progress(left); rp,rg=phase_progress(right); out=[]
    for li,l in enumerate(left):
        ph=l.get('planner_phase','unknown'); candidates=rg.get(ph,[])
        if not candidates: continue
        ri=min(candidates,key=lambda j:abs(rp[j]-lp[li]))
        out.append((li,ri,abs(rp[ri]-lp[li])))
    return out
def ssim(a,b):
    a=a.astype(np.float32); b=b.astype(np.float32)
    c1=(.01*255)**2; c2=(.03*255)**2
    ma=gaussian_filter(a,1.5); mb=gaussian_filter(b,1.5)
    va=gaussian_filter(a*a,1.5)-ma*ma
    vb=gaussian_filter(b*b,1.5)-mb*mb
    vab=gaussian_filter(a*b,1.5)-ma*mb
    return float(np.mean(((2*ma*mb+c1)*(2*vab+c2))/((ma*ma+mb*mb+c1)*(va+vb+c2))))

def main():
    for d in ['00_audit','03_alignment','04_representation_gap','09_figures','final_report']:
        (ROOT/d).mkdir(parents=True,exist_ok=True)
    episodes={}; audit=[]; excluded=[]
    for cond in CONDS:
      for i in range(1,6):
        eid=f'episode_{i:06d}'; ep=DATA/cond/'episodes'/eid
        steps=read_jsonl(ep/'steps.jsonl') if (ep/'steps.jsonl').exists() else []
        acts=read_jsonl(ep/'steps_with_actions.jsonl') if (ep/'steps_with_actions.jsonl').exists() else []
        md=json.loads((ep/'metadata.json').read_text()) if (ep/'metadata.json').exists() else {}
        imgs=sorted((ep/'images'/'primary').glob('*.jpg'))
        hashes=[digest_file(x) for x in imgs]
        ts=[r.get('timestamp_ns') for r in steps if isinstance(r.get('timestamp_ns'),(int,float))]
        phases=[r.get('planner_phase','unknown') for r in steps]
        shape_ok=all(flat_action(r) is not None for r in acts)
        complete=bool(steps and len(imgs)==len(steps)==len(acts) and len(set(hashes))==len(hashes) and all(b>a for a,b in zip(ts,ts[1:])) and shape_ok and md.get('success') is True)
        episodes[(cond,eid)]={'ep':ep,'steps':steps,'actions':acts,'md':md,'imgs':imgs}
        row={'condition':cond,'episode_id':eid,'path':str(ep),'image_count':len(imgs),'step_count':len(steps),'action_count':len(acts),'timestamp_monotonic':all(b>a for a,b in zip(ts,ts[1:])),'duplicate_image_count':len(hashes)-len(set(hashes)),'instruction':md.get('instruction') or (steps[0].get('instruction') if steps else None),'phase_counts':json.dumps(Counter(phases),sort_keys=True),'gripper_first_close_step':next((j for j,r in enumerate(steps) if float(r.get('gripper_closedness',0))>=.5),None),'action_shape_valid':shape_ok,'metadata_success':md.get('success'),'planned_pose_samples':md.get('planned_pose_samples'),'feedback_pose_samples':md.get('feedback_pose_samples'),'layout_source':md.get('block_layout_source'),'image_set_sha256':hashlib.sha256(''.join(hashes).encode()).hexdigest(),'episode_complete':complete}
        audit.append(row)
        if not complete: excluded.append({'condition':cond,'episode_id':eid,'reason':'episode_file_integrity'})
    pairs=[]; aligned=[]
    for cond in CONDS[1:]:
      for i in range(1,6):
        eid=f'episode_{i:06d}'; b=episodes[('baseline',eid)]; c=episodes[(cond,eid)]
        bl,cl=layout(b['md']),layout(c['md']); br,cr=route(b['md']),route(c['md'])
        orange_equal=bl.get('orange')==cl.get('orange') if bl and cl else None
        routes=[]
        for k in ('alignment','grasp','lift'):
            if k in br and k in cr: routes.append(float(np.max(np.abs(np.asarray(br[k],float)-np.asarray(cr[k],float)))))
        route_max=max(routes) if routes else None
        same_layout=(bl==cl) if bl and cl else None
        distract_ok=None
        if cond=='distractor_swap' and bl and cl:
            distract_ok=orange_equal and bl.get('yellow')==cl.get('blue') and bl.get('blue')==cl.get('yellow')
        maps=nearest_align(b['steps'],c['steps'])
        exact=(len(b['steps'])==len(c['steps']) and [r.get('planner_phase') for r in b['steps']]==[r.get('planner_phase') for r in c['steps']] and route_max is not None and route_max<1e-9)
        valid_base=next(x['episode_complete'] for x in audit if x['condition']=='baseline' and x['episode_id']==eid)
        valid_case=next(x['episode_complete'] for x in audit if x['condition']==cond and x['episode_id']==eid)
        reference_ok=(route_max is not None and route_max<1e-6 and orange_equal is True and ((cond=='distractor_swap' and distract_ok is True) or (cond!='distractor_swap' and same_layout is True)))
        status='VALID_EXACT_PAIR' if valid_base and valid_case and reference_ok and exact else ('VALID_PHASE_ALIGNABLE' if valid_base and valid_case and reference_ok and maps else ('INVALID_INCOMPLETE' if not(valid_base and valid_case) else 'INVALID_REFERENCE_MISMATCH'))
        pair={'condition':cond,'episode_id':eid,'status':status,'baseline_steps':len(b['steps']),'condition_steps':len(c['steps']),'orange_xy_equal':orange_equal,'layout_exact_equal':same_layout,'distractor_swap_only':distract_ok,'route_pose_max_abs_diff':route_max,'aligned_count':len(maps),'dropped_baseline':len(b['steps'])-len(maps),'interpolated_count':sum(1 for _,_,d in maps if d>1e-12),'max_phase_progress_residual':max((d for _,_,d in maps),default=None),'lighting_manifest_reconstructed':cond=='lighting_low'}
        pairs.append(pair)
        if status.startswith('VALID'):
          for li,ri,pres in maps:
            lr=b['steps'][li]; rr=c['steps'][ri]; lp=pose(lr); rp=pose(rr)
            aligned.append({'condition':cond,'episode_id':eid,'baseline_index':li,'condition_index':ri,'phase':lr.get('planner_phase'),'baseline_phase_progress':phase_progress(b['steps'])[0][li],'condition_phase_progress':phase_progress(c['steps'])[0][ri],'phase_progress_residual':pres,'reference_pose_translation_residual_m':None if lp is None or rp is None else float(np.linalg.norm(lp-rp)),'baseline_timestamp':lr.get('timestamp'),'condition_timestamp':rr.get('timestamp'),'timestamp_residual_sec':None if lr.get('timestamp') is None or rr.get('timestamp') is None else float(rr['timestamp']-lr['timestamp']),'baseline_image':str(image_path(b['ep'],lr)),'condition_image':str(image_path(c['ep'],rr))})
        else: excluded.append({'condition':cond,'episode_id':eid,'reason':status})
    write_csv(ROOT/'00_audit/paired_episode_integrity.csv',audit+pairs)
    write_csv(ROOT/'03_alignment/aligned_samples.csv',aligned)
    valid=[p for p in pairs if p['status'].startswith('VALID')]
    (ROOT/'00_audit/excluded_samples.json').write_text(json.dumps(excluded,indent=2))
    (ROOT/'00_audit/final_valid_pairs.json').write_text(json.dumps({'valid_pair_count':len(valid),'pairs':valid},indent=2))
    candidate={'status':'RECONSTRUCTED_CANDIDATE_NOT_SOURCE_MANIFEST','condition':'lighting_low','episodes':[{'episode_id':f'episode_{i:06d}','path':str((DATA/'lighting_low/episodes'/f'episode_{i:06d}').resolve())} for i in range(1,6)],'warning':'Condition identity is inferred from directory placement; no source condition manifest was found.'}
    (ROOT/'00_audit/lighting_low_manifest_candidate.json').write_text(json.dumps(candidate,indent=2))
    lines=['# Phase 11 paired episode integrity','',f'- Episode directories audited: {len(audit)}',f'- Valid pairs: {len(valid)}/15',f'- Excluded pairs: {15-len(valid)}','- Stored action provenance: Scripted Reference Command; not measured action and not Ground Truth Action.','- `lighting_low` source manifest was absent; a candidate manifest was reconstructed from episode paths only.','', '| Condition | Episode | Status | Baseline/Case steps | Route max diff |','|---|---|---|---:|---:|']
    for p in pairs: lines.append(f"| {p['condition']} | {p['episode_id']} | {p['status']} | {p['baseline_steps']}/{p['condition_steps']} | {p['route_pose_max_abs_diff']} |")
    (ROOT/'00_audit/paired_episode_integrity.md').write_text('\n'.join(lines)+'\n')
    # Observation metrics after phase alignment.
    obs=[]
    for r in aligned:
        a=np.asarray(Image.open(r['baseline_image']).convert('RGB')); b=np.asarray(Image.open(r['condition_image']).convert('RGB'))
        if a.shape!=b.shape: b=np.asarray(Image.fromarray(b).resize((a.shape[1],a.shape[0]),Image.Resampling.BILINEAR))
        af=a.astype(np.float32)/255.; bf=b.astype(np.float32)/255.; d=af-bf
        ga=np.dot(a[...,:3],[0.299,0.587,0.114]); gb=np.dot(b[...,:3],[0.299,0.587,0.114])
        obs.append({**{k:r[k] for k in ('condition','episode_id','baseline_index','condition_index','phase')},'pixel_l1':float(np.mean(np.abs(d))),'pixel_l2_rmse':float(np.sqrt(np.mean(d*d))),'ssim':ssim(ga,gb),'brightness_baseline':float(np.mean(ga)/255),'brightness_condition':float(np.mean(gb)/255),'brightness_delta':float((np.mean(gb)-np.mean(ga))/255)})
    write_csv(ROOT/'04_representation_gap/observation_frame_metrics.csv',obs)
    summary=[]
    for key in sorted(set((r['condition'],r['episode_id']) for r in obs)):
        ss=[r for r in obs if (r['condition'],r['episode_id'])==key]
        summary.append({'condition':key[0],'episode_id':key[1],'count':len(ss),**{m+'_mean':float(np.mean([x[m] for x in ss])) for m in ('pixel_l1','pixel_l2_rmse','ssim','brightness_delta')}})
    write_csv(ROOT/'04_representation_gap/observation_episode_summary.csv',summary)
    (ROOT/'03_alignment/summary.json').write_text(json.dumps({
        'valid_pairs':len(valid),'aligned_samples':len(aligned),
        'method':'same phase plus nearest normalized within-phase progress',
        'raw_index_only':False,'excluded_pairs':15-len(valid)
    },indent=2))
    print(json.dumps({'episodes':len(audit),'valid_pairs':len(valid),'excluded_pairs':15-len(valid),'aligned_samples':len(aligned),'observation_samples':len(obs)},indent=2))
if __name__=='__main__': main()
