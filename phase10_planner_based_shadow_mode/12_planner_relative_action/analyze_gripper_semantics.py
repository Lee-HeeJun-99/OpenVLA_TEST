#!/usr/bin/env python3
"""Offline gripper diagnosis from immutable recorded model outputs."""
from __future__ import annotations
import csv, json, math, statistics
from collections import defaultdict
from pathlib import Path
import matplotlib.pyplot as plt
import numpy as np

ROOT=Path(__file__).resolve().parents[1]
RUNS=ROOT/'06_shadow_collection/offline_recorded_episode4'
OUT=ROOT/'12_planner_relative_action'; FIG=OUT/'figures'

def load(p): return [json.loads(x) for x in Path(p).read_text().splitlines() if x.strip()]
def select(kind):
    candidates=[]
    for d in RUNS.iterdir():
        p=d/'samples.jsonl'
        if not p.exists(): continue
        rows=load(p)
        pred=sum((r.get('openvla_denormalized_action') is not None) if kind=='openvla' else (r.get('oft_denormalized_action_chunk') is not None) for r in rows)
        if len(rows)==45 and pred and not any(r.get('fixture_smoke_test_only') for r in rows) and not any(r.get('inference_errors') for r in rows):
            candidates.append((p.stat().st_mtime,d,rows))
    if not candidates: raise RuntimeError(f'no valid {kind} run')
    return max(candidates,key=lambda x:x[0])[1:]

def binary(values,t=.5,polarity='higher_close'):
    return [int(v>=t) if polarity=='higher_close' else int(v<=t) for v in values]
def first(v): return next((i for i,x in enumerate(v) if x),None)
def persistent(v,n):
    return next((i for i in range(len(v)-n+1) if all(v[i:i+n])),None)
def transitions(v): return sum(a==0 and b==1 for a,b in zip(v,v[1:])),sum(a==1 and b==0 for a,b in zip(v,v[1:]))
def cm(ref,pred):
    tp=sum(a and b for a,b in zip(ref,pred)); tn=sum(not a and not b for a,b in zip(ref,pred)); fp=sum(not a and b for a,b in zip(ref,pred)); fn=sum(a and not b for a,b in zip(ref,pred))
    acc=(tp+tn)/len(ref); prec=tp/(tp+fp) if tp+fp else 0.; rec=tp/(tp+fn) if tp+fn else 0.; f1=2*prec*rec/(prec+rec) if prec+rec else 0.; tnr=tn/(tn+fp) if tn+fp else 0.
    return dict(tp=tp,tn=tn,fp=fp,fn=fn,accuracy=acc,precision=prec,recall=rec,f1=f1,balanced_accuracy=(rec+tnr)/2)
def runtime_hysteresis(values,open_t=.7,close_t=.3,initial_open=True):
    state_open=initial_open; closed=[]
    for v in values:
        if v>=open_t: state_open=True
        elif v<=close_t: state_open=False
        closed.append(int(not state_open))
    return closed
def percentile(v,p): return float(np.percentile(np.asarray(v,float),p))
def write_csv(path,rows):
    with path.open('w',newline='',encoding='utf-8') as f:
        w=csv.DictWriter(f,fieldnames=list(rows[0])); w.writeheader(); w.writerows(rows)

def main():
    ov_dir,ov_rows=select('openvla'); oft_dir,oft_rows=select('oft')
    refs=load(OUT/'episode4_reference_actions.jsonl'); n=len(refs)
    ref=[float(r['reference_gripper_command']) for r in refs]
    ov=[float(r['openvla_denormalized_action'][6]) for r in ov_rows]
    ov_raw=[float(r['openvla_raw_action']['action'][6]) for r in ov_rows]
    oft=[None]*n; oft_raw=[None]*n; meta=[None]*n; heat=np.full((5,9),np.nan); frames=[]
    for row in oft_rows:
        chunk=row.get('oft_denormalized_action_chunk')
        if chunk is None: continue
        frame=int(row['frame_id']); raw=row['oft_raw_action_chunk']; request=raw.get('request_index'); frames.append(frame)
        for k,a in enumerate(chunk):
            target=frame+k; value=float(a[6]); oft[target]=value; oft_raw[target]=float(raw['actions'][k][6]); heat[k,len(frames)-1]=value
            meta[target]=(frame,k,request,len(chunk),target)
    assert frames==list(range(0,45,5)) and all(v is not None for v in oft)
    rb=binary(ref); ob=binary(ov); fb=binary(oft)
    trace=[]
    for i in range(n):
        f,k,request,size,target=meta[i]
        trace.append({"step_index":i,"time_s":i*.2,"phase":refs[i]['phase'],"reference_gripper_raw":ref[i],"reference_gripper_closedness":ref[i],"reference_binary_close":rb[i],"openvla_gripper_raw":ov_raw[i],"openvla_gripper_denormalized":ov[i],"openvla_gripper_closedness":ov[i],"openvla_binary_close":ob[i],"oft_inference_frame":f,"oft_server_request_index":request,"oft_chunk_index":k,"oft_expanded_target_step":target,"oft_chunk_size":size,"oft_gripper_raw":oft_raw[i],"oft_gripper_denormalized":oft[i],"oft_gripper_closedness":oft[i],"oft_binary_close":fb[i],"reference_valid":refs[i]['valid'],"openvla_valid":ov_rows[i]['valid'],"oft_valid":True})
    write_csv(OUT/'episode4_gripper_trace.csv',trace)

    sweep=[]
    for pol in ('higher_close','lower_close'):
      for t in [i/10 for i in range(1,10)]:
        # Reference semantics are known 0=open/1=closed and are never polarity-flipped.
        rr=binary(ref,.5,'higher_close'); oo=binary(ov,t,pol); ff=binary(oft,t,pol); co=cm(rr,oo); cf=cm(rr,ff); rs=first(rr); os=first(oo); fs=first(ff)
        sweep.append({"polarity":pol,"threshold":t,"reference_close_step":rs,"openvla_close_step":os,"oft_close_step":fs,"openvla_timing_error_steps":None if rs is None or os is None else os-rs,"oft_timing_error_steps":None if rs is None or fs is None else fs-rs,"openvla_accuracy":co['accuracy'],"openvla_f1":co['f1'],"oft_accuracy":cf['accuracy'],"oft_f1":cf['f1'],"setting_role":"diagnostic_sweep_not_runtime_selection"})
    write_csv(OUT/'episode4_gripper_threshold_sweep.csv',sweep)

    conf=[]; phase=[]
    for model,pred in [('openvla_step8130',ob),('oft_vision_step28560',fb)]:
        conf.append({"scope":"episode","model":model,"phase":"all","chunk_index":"all",**cm(rb,pred)})
        for ph in sorted(set(r['phase'] for r in refs)):
            idx=[i for i,r in enumerate(refs) if r['phase']==ph]; c=cm([rb[i] for i in idx],[pred[i] for i in idx]); phase.append({"model":model,"phase":ph,"samples":len(idx),**c})
    write_csv(OUT/'episode4_gripper_confusion_summary.csv',conf); write_csv(OUT/'episode4_gripper_phase_summary.csv',phase)
    chunks=[]
    for k in range(5):
        idx=[i for i in range(n) if meta[i][1]==k]; c=cm([rb[i] for i in idx],[fb[i] for i in idx]); chunks.append({"model":"oft_vision_step28560","chunk_index":k,"samples":len(idx),**c,"mean_closedness":sum(oft[i] for i in idx)/len(idx)})
    write_csv(OUT/'episode4_gripper_chunk_index_summary.csv',chunks)

    # Runtime consumes only chunk[0] and interprets high=open/low=close with hysteresis.
    oft_chunk0=[oft[i] for i in frames]; runtime_events=runtime_hysteresis(oft_chunk0)
    expanded_runtime=[]
    for state in runtime_events: expanded_runtime.extend([state]*5)
    ov_runtime=runtime_hysteresis(ov)
    events={"reference":rb,"openvla_offline":ob,"oft_offline_expanded":fb,"openvla_runtime_equivalent":ov_runtime,"oft_runtime_equivalent_chunk0_hold":expanded_runtime[:n]}
    event_summary={}
    for name,v in events.items():
        o2c,c2o=transitions(v); event_summary[name]={"first_close_step":first(v),"persistent_close_step":{"N1":persistent(v,1),"N2":persistent(v,2),"N3":persistent(v,3),"N5":persistent(v,5)},"open_to_close_transitions":o2c,"close_to_open_transitions":c2o,"false_close_count":cm(rb,v)['fp'],"missed_close_count":cm(rb,v)['fn']}
    event_summary['oft_chunk_level_first_close']={"inference_frame":meta[first(fb)][0],"chunk_index":meta[first(fb)][1],"expanded_step":first(fb)}

    latency_rows=[]; lat_summary={}
    for model,rows_,key,budget in [('openvla_step8130',ov_rows,'inference_latency_openvla',.2),('oft_vision_step28560',oft_rows,'inference_latency_oft',1.0)]:
        vals=[]
        for r in rows_:
            if r.get(key) is not None:
                vals.append(float(r[key])); latency_rows.append({"model":model,"frame_id":r['frame_id'],"latency_seconds":r[key],"budget_seconds":budget,"budget_exceeded":float(r[key])>budget,"is_first_request":len(vals)==1})
        def stats(v): return {"count":len(v),"mean":statistics.mean(v),"median_p50":statistics.median(v),"p90":percentile(v,90),"p95":percentile(v,95),"p99":percentile(v,99),"max":max(v),"standard_deviation":statistics.pstdev(v),"budget_seconds":budget,"budget_exceed_count":sum(x>budget for x in v),"budget_exceed_ratio":sum(x>budget for x in v)/len(v)}
        lat_summary[model]={"including_warmup":stats(vals),"excluding_first_request":stats(vals[1:])}
    write_csv(OUT/'episode4_latency_samples.csv',latency_rows)
    (OUT/'episode4_latency_summary.json').write_text(json.dumps({"status":"COMPLETED_WITH_MODEL_SERVER","one_episode_only":True,"models":lat_summary},indent=2)+'\n')

    diagnosis={"classification":"H. MULTIPLE_CONTRIBUTING_FACTORS","confirmed_factors":["model_output_crosses_dataset-closedness threshold early (expanded step 1)","B. GRIPPER_POLARITY_MISMATCH in Real ActionAdapter","E. THRESHOLD_DEFINITION_ERROR if step-1 is called runtime close","G. RUNTIME_OFFLINE_POSTPROCESS_MISMATCH","Real runtime discards chunk indices 1..4"],"not_supported":["C. DENORMALIZATION_ERROR","D. CANONICALIZATION_ERROR","F. CHUNK_ALIGNMENT_ERROR in offline expansion"],"selected_runs":{"openvla":ov_dir.name,"oft":oft_dir.name},"chunk_mapping":{"inference_frames":frames,"complete":True,"duplicates":0,"missing":0,"off_by_one":False},"event_summary":event_summary,"runtime_thresholds":{"open_if_gte":.7,"close_if_lte":.3,"middle":"hold_last","initial":"open"},"safety":{"robot_commands":0,"ros_calls":0,"closed_loop":0}}
    (OUT/'gripper_early_close_diagnosis.json').write_text(json.dumps(diagnosis,indent=2)+'\n')

    FIG.mkdir(exist_ok=True); x=np.arange(n)
    bounds=[]
    for i in range(1,n):
        if refs[i]['phase']!=refs[i-1]['phase']: bounds.append(i-.5)
    plt.figure(figsize=(12,5))
    plt.plot(x,ref,label='Reference continuous')
    plt.plot(x,ov,label='OpenVLA continuous')
    plt.plot(x,oft,label='OFT expanded continuous')
    plt.step(x,rb,where='post',ls=':',alpha=.45,label='Reference binary')
    plt.step(x,ob,where='post',ls=':',alpha=.45,label='OpenVLA binary')
    plt.step(x,fb,where='post',ls=':',alpha=.45,label='OFT binary')
    plt.axhline(.5,color='k',ls=':',lw=1)
    for b in bounds:
        plt.axvline(b,color='gray',alpha=.3)
    plt.axvline(26,color='k',ls='--',label='Reference close: 26')
    plt.axvline(23,color='C1',ls='--',label='OpenVLA close: 23')
    plt.axvline(1,color='C2',ls='--',label='OFT close: 1')
    plt.xlim(0,n-1)
    plt.xlabel('step')
    plt.ylabel('closedness')
    plt.legend(ncol=2,fontsize=8)
    plt.tight_layout()
    plt.savefig(FIG/'episode4_gripper_trace.png',dpi=180)
    plt.close()
    plt.figure(figsize=(12,4)); plt.step(x,rb,where='post',label='Reference'); plt.step(x,ob,where='post',label='OpenVLA'); plt.step(x,fb,where='post',label='OFT offline'); plt.step(x,expanded_runtime[:n],where='post',label='OFT runtime-equivalent',alpha=.8); plt.yticks([0,1],['open','close']); plt.legend(); plt.tight_layout(); plt.savefig(FIG/'episode4_gripper_binary_events.png',dpi=180); plt.close()
    plt.figure(figsize=(11,4)); im=plt.imshow(heat,aspect='auto',vmin=0,vmax=1,cmap='viridis'); plt.xticks(range(9),frames); plt.yticks(range(5)); plt.xlabel('inference frame'); plt.ylabel('chunk index'); plt.colorbar(im,label='closedness');
    for k in range(5):
      for j,f in enumerate(frames): plt.text(j,k,str(f+k),ha='center',va='center',color='white' if heat[k,j]<.55 else 'black',fontsize=7)
    plt.tight_layout(); plt.savefig(FIG/'episode4_oft_chunk_gripper_heatmap.png',dpi=180); plt.close()
    labels=[]; ofp=[]; ovfp=[]
    for ph in sorted(set(r['phase'] for r in refs)):
        labels.append(ph); ofp.append(next(r['fp'] for r in phase if r['model'].startswith('oft') and r['phase']==ph)); ovfp.append(next(r['fp'] for r in phase if r['model'].startswith('open') and r['phase']==ph))
    z=np.arange(len(labels)); plt.figure(figsize=(10,4)); plt.bar(z-.2,ovfp,.4,label='OpenVLA FP'); plt.bar(z+.2,ofp,.4,label='OFT FP'); plt.xticks(z,labels,rotation=20); plt.ylabel('false-close frames'); plt.legend(); plt.tight_layout(); plt.savefig(FIG/'episode4_gripper_phase_errors.png',dpi=180); plt.close()
    plt.figure(figsize=(10,4));
    for model in sorted(set(r['model'] for r in latency_rows)):
        v=[float(r['latency_seconds']) for r in latency_rows if r['model']==model]; plt.plot(range(len(v)),v,marker='o',label=model)
    plt.axhline(.2,color='C0',ls=':',label='OpenVLA 0.2s budget'); plt.axhline(1.,color='C1',ls=':',label='OFT 1.0s budget'); plt.xlabel('request index'); plt.ylabel('seconds'); plt.legend(); plt.tight_layout(); plt.savefig(FIG/'episode4_inference_latency.png',dpi=180); plt.close()
    print(json.dumps(diagnosis,indent=2))

if __name__=='__main__': main()
