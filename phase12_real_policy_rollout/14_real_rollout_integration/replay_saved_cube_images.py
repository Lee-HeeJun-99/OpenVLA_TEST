"""Replay existing model-input JPEGs to the local model. No ROS or robot IO."""
import argparse
import base64
import csv
import hashlib
import json
import math
from pathlib import Path
import time
from urllib.request import Request,urlopen

import numpy as np
from scipy.spatial.transform import Rotation
from verify_live_model_server import validate_identity

ROOT=Path(__file__).resolve().parent
SAMPLES=[('right','20261106_110547_arm_only_servo_episode','camera_000.jpg'),
         ('center_lower','20261106_111406_arm_only_servo_episode','camera_000.jpg'),
         ('left','20261106_125319_arm_only_servo_episode','camera_000.jpg'),
         ('center_lower_moveline','20261106_132009_arm_only_moveline_episode','camera_000_model_input.jpg')]


def compare(a,b):
    a=np.asarray(a,dtype=float);b=np.asarray(b,dtype=float)
    return dict(translation_difference_mm=float(np.linalg.norm(a[:3]-b[:3])*1000),
        rotation_delta_difference_deg=math.degrees((Rotation.from_rotvec(a[3:6])*Rotation.from_rotvec(b[3:6]).inv()).magnitude()),
        gripper_abs_difference=float(abs(a[6]-b[6])))


def replay(output,repeats=5):
    if repeats<2:raise ValueError('at_least_two_repeats_for_same_input_control')
    output.mkdir(parents=True,exist_ok=False)
    def save(name,value):(output/name).write_text(json.dumps(value,indent=2,allow_nan=False))
    with urlopen('http://127.0.0.1:8766/health',timeout=5) as response:health=json.load(response)
    validate_identity(health,'openvla')
    if health.get('status')!='ready':raise RuntimeError('model_not_ready')
    save('model_health_before.json',health)
    manifest=[];raws={key:[] for key,_,_ in SAMPLES};records=[]
    for label,trial,image in SAMPLES:
        folder=ROOT/'real_trials'/trial;path=folder/image
        first=json.loads((folder/'predictions.jsonl').open().readline())
        pre=json.loads((folder/'preflight.json').read_text())
        manifest.append(dict(label=label,source_image=str(path),image_sha256=hashlib.sha256(path.read_bytes()).hexdigest(),
            original_raw_action=first['raw_action'],original_start_pose_error_deg=pre.get('start_pose_error_deg'),
            cube_location_label_source='MANUAL_VISUAL_INSPECTION_NOT_METRIC_POSITION',
            confounders='Gripper state/covers, pendant, people and other objects differ; not cube-only controlled A/B.'))
    save('image_manifest.json',manifest)
    with (output/'predictions.jsonl').open('x') as f:
        for repeat in range(repeats):
            # Interleave positions instead of running one position entirely first.
            for item in manifest:
                jpeg=Path(item['source_image']).read_bytes()
                if hashlib.sha256(jpeg).hexdigest()!=item['image_sha256']:raise RuntimeError('input_changed')
                request=Request('http://127.0.0.1:8766/predict',data=json.dumps(dict(
                    image_jpeg_base64=base64.b64encode(jpeg).decode(),instruction='Pick up the orange cube.')).encode(),
                    headers={'Content-Type':'application/json'})
                t0=time.monotonic()
                with urlopen(request,timeout=30) as response:prediction=json.load(response)
                action=prediction['action']
                if len(action)!=7 or not all(math.isfinite(v) for v in action):raise ValueError('invalid_action')
                row=dict(label=item['label'],repeat=repeat,image_sha256=item['image_sha256'],raw_action=action,
                    inference_latency_s=time.monotonic()-t0,command_requested=False,command_issued=False,
                    live_camera_freshness_applicable=False)
                f.write(json.dumps(row)+'\n');f.flush()
                raws[item['label']].append(action);records.append(row)
    per_image=[]
    for item in manifest:
        actions=raws[item['label']];mean=np.mean(actions,axis=0).tolist()
        control=[compare(a,b) for a in actions for b in actions]
        per_image.append(dict(label=item['label'],repeat_count=len(actions),mean_raw_action=mean,
            mean_translation_mm=(np.array(mean[:3])*1000).tolist(),mean_rotation_rotvec_rad=mean[3:6],
            gripper=mean[6],unique_raw_actions=len({tuple(a) for a in actions}),
            same_image_repeat_difference_max={k:max(x[k] for x in control) for k in control[0]},
            difference_from_original=compare(mean,item['original_raw_action'])))
    pairs=[]
    for i,a in enumerate(per_image):
        for b in per_image[i+1:]:pairs.append(dict(image_a=a['label'],image_b=b['label'],**compare(a['mean_raw_action'],b['mean_raw_action'])))
    with (output/'action_comparison.csv').open('w',newline='') as f:
        writer=csv.DictWriter(f,fieldnames=list(pairs[0]));writer.writeheader();writer.writerows(pairs)
    with urlopen('http://127.0.0.1:8766/health',timeout=5) as response:after=json.load(response)
    validate_identity(after,'openvla')
    if any(after.get(k)!=health.get(k) for k in ['checkpoint','dataset_key','variant','preprocessing']):
        raise RuntimeError('model_identity_changed_during_replay')
    save('model_health_after.json',after)
    summary=dict(status='SAVED_IMAGE_PREDICTION_REPLAY_COMPLETE',samples=len(manifest),predictions=len(records),
        per_image=per_image,pairwise_comparisons=pairs,
        physical_commands=0,transport_created=False,robot_connection_required=False,
        interpretation='Whole-image action sensitivity only. Not proof of cube detection, correct direction or cube-only causal effect.',
        cube_world_positions_and_camera_to_base_calibration_verified=False,
        image_preprocessing='Exact saved model-input JPEG bytes, unchanged; server training processor retained.')
    save('replay_summary.json',summary);print(json.dumps(summary,indent=2),flush=True)
    return summary


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',type=Path,default=ROOT/'real_trials'/time.strftime('%Y%m%d_%H%M%S_saved_cube_image_replay'))
    parser.add_argument('--repeats',type=int,default=5)
    args=parser.parse_args();replay(args.output,args.repeats)
