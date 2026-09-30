#!/usr/bin/env python3
"""Reference-command-relative metrics for recorded Episode 4 predictions."""
from __future__ import annotations
import argparse, csv, json, math
from collections import defaultdict
from pathlib import Path


def rows(path): return [json.loads(x) for x in Path(path).read_text().splitlines() if x.strip()]
def norm(v): return math.sqrt(sum(x*x for x in v))
def cosine_distance(a,b):
    na,nb=norm(a),norm(b)
    return None if na < 1e-12 or nb < 1e-12 else 1-sum(x*y for x,y in zip(a,b))/(na*nb)
def mean(values):
    values=[v for v in values if v is not None]
    return sum(values)/len(values) if values else None
def rmse(values): return math.sqrt(mean([v*v for v in values])) if values else None


def metric(model, frame, chunk, phase, pred, ref):
    te=[pred[i]-ref[i] for i in range(3)]; re=[pred[i]-ref[i] for i in range(3,6)]
    return {
        "model":model,"frame_id":frame,"chunk_index":chunk,"phase":phase,
        "translation_l1_m":sum(abs(x) for x in te),"translation_l2_m":norm(te),
        "translation_direction_cosine_distance":cosine_distance(pred[:3],ref[:3]),
        "translation_magnitude_error_m":abs(norm(pred[:3])-norm(ref[:3])),
        "rotation_l1_rad":sum(abs(x) for x in re),"rotation_l2_rad":norm(re),
        "rotation_convention":"collector_rpy_assumption",
        "gripper_absolute_error":abs(pred[6]-ref[6]),
        "gripper_reference_closed":int(ref[6]>=0.5),"gripper_predicted_closed":int(pred[6]>=0.5),
        "gripper_correct":int((pred[6]>=0.5)==(ref[6]>=0.5)),
        "reference_status":"REFERENCE_COMMAND_NOT_MEASURED_GROUND_TRUTH",
    }


def main():
    ap=argparse.ArgumentParser(); ap.add_argument('--reference',type=Path,required=True); ap.add_argument('--openvla',type=Path,required=True); ap.add_argument('--oft',type=Path,required=True); ap.add_argument('--output-dir',type=Path,required=True); a=ap.parse_args()
    ref=rows(a.reference); ov=rows(a.openvla); oft=rows(a.oft); result=[]
    for row in ov:
        i=int(row['frame_id']); pred=row.get('openvla_denormalized_action')
        if pred is not None and i < len(ref): result.append(metric('openvla_step8130',i,0,ref[i]['phase'],pred,ref[i]['canonical_action']))
    for row in oft:
        start=int(row['frame_id']); chunk=row.get('oft_denormalized_action_chunk')
        if chunk is None: continue
        for k,pred in enumerate(chunk):
            i=start+k
            if i < len(ref): result.append(metric('oft_vision_step28560',start,k,ref[i]['phase'],pred,ref[i]['canonical_action']))
    a.output_dir.mkdir(parents=True,exist_ok=True)
    fields=list(result[0]);
    with (a.output_dir/'episode4_model_reference_metrics.csv').open('w',newline='',encoding='utf-8') as f:
        w=csv.DictWriter(f,fieldnames=fields); w.writeheader(); w.writerows(result)
    grouped=defaultdict(list)
    for r in result: grouped[(r['model'],r['phase'])].append(r)
    summaries=[]
    for (model,phase),g in sorted(grouped.items()):
        summaries.append({"model":model,"phase":phase,"samples":len(g),
            "translation_mae_l1_m":mean([x['translation_l1_m'] for x in g]),
            "translation_rmse_l2_m":rmse([x['translation_l2_m'] for x in g]),
            "rotation_mae_l1_rad":mean([x['rotation_l1_rad'] for x in g]),
            "rotation_rmse_l2_rad":rmse([x['rotation_l2_rad'] for x in g]),
            "gripper_mae":mean([x['gripper_absolute_error'] for x in g]),
            "gripper_accuracy":mean([x['gripper_correct'] for x in g])})
    with (a.output_dir/'episode4_phase_summary.csv').open('w',newline='',encoding='utf-8') as f:
        w=csv.DictWriter(f,fieldnames=list(summaries[0])); w.writeheader(); w.writerows(summaries)
    overall={}
    for model in sorted({r['model'] for r in result}):
        g=[r for r in result if r['model']==model]
        overall[model]={"samples":len(g),"translation_l1_mean_m":mean([x['translation_l1_m'] for x in g]),
            "translation_l2_rmse_m":rmse([x['translation_l2_m'] for x in g]),
            "rotation_l1_mean_rad":mean([x['rotation_l1_rad'] for x in g]),
            "rotation_l2_rmse_rad":rmse([x['rotation_l2_rad'] for x in g]),
            "gripper_mae":mean([x['gripper_absolute_error'] for x in g]),
            "gripper_accuracy":mean([x['gripper_correct'] for x in g]),
            "translation_cosine_distance_mean":mean([x['translation_direction_cosine_distance'] for x in g])}
    (a.output_dir/'episode4_metric_summary.json').write_text(json.dumps({"status":"COMPLETED_WITH_MODEL_SERVER","comparison":"reference-command-relative error","rotation_convention":"collector_rpy_assumption","overall":overall},indent=2)+'\n')
    print(json.dumps(overall,indent=2))

if __name__=='__main__': main()
