"""Offline recorded prediction geometry audit. No ROS/robot transport."""
import argparse
import json
import math
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
from servo_experimental.controller import interpolate_segment,continuous_zyz
from run_real_rollout import SafetyPipeline
from action_contract import rotation_step_geodesic_rad,compose_world_rotvec_with_doosan_zyz


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--trial',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    records=[json.loads(line) for line in (args.trial/'predictions.jsonl').read_text().splitlines()]
    pipeline=SafetyPipeline('openvla',operator_confirmed_initial_open=False,arm_only=True,
                            translation_endpoint_delta=True)
    replay=[]
    for row in records:
        raw=row['raw_action'];pose=row['current_controller_pose'];timing=row['camera_timing']
        now=row['action_pipeline']['action_age_sec']+timing['inference_start_monotonic']
        new=pipeline.runtime.evaluate(raw,action_id=str(row['step']),
            source_monotonic=timing['inference_start_monotonic'],now_monotonic=now,
            current_position_m=[v/1000 for v in pose[:3]],current_abc_deg=pose[3:],phase='alignment',arm_only=True)
        raw_target,_=compose_world_rotvec_with_doosan_zyz(pose[3:],raw[3:6])
        stored_target=row['action_pipeline']['target_pose_candidate_mm_zyz_deg']
        replay.append(dict(target_number=len(replay)+1,step=row['step'],
            raw_rotvec_norm_deg=math.degrees(np.linalg.norm(raw[3:6])),
            raw_target_geodesic_deg=math.degrees(rotation_step_geodesic_rad(pose[3:],raw_target)),
            recorded_filtered_target_geodesic_deg=math.degrees(rotation_step_geodesic_rad(pose[3:],stored_target[3:])),
            recorded_euler_difference_l2_deg=float(np.linalg.norm(np.array(stored_target[3:])-pose[3:])),
            before_raw_rotation_rejected='raw_rotation_step_limit' in row['action_pipeline']['technical_blockers'],
            after_raw_rotation_rejected='raw_rotation_step_limit' in new['technical_blockers'],
            before_blockers=row['action_pipeline']['technical_blockers'],after_blockers=new['technical_blockers']))
    summary=dict(physical_commands=0,count=len(replay),rotation_limit_deg=4.,
        target_63=replay[62] if len(replay)>62 else None,
        note='Raw model rotvec and filtered target are different quantities; raw gate remains enforced.',
        replay=replay)
    max_reconstruction_error=0.
    for row in records:
        start=np.asarray(row['current_controller_pose'])
        end=np.asarray(row['action_pipeline']['target_pose_candidate_mm_zyz_deg'])
        r0=Rotation.from_euler('ZYZ',start[3:],degrees=True)
        r1=Rotation.from_euler('ZYZ',end[3:],degrees=True)
        previous=start[3:]
        for fraction in np.linspace(0,1,21):
            p,r=interpolate_segment(start[:3],r0,end[:3],r1,float(fraction))
            abc=continuous_zyz(r,previous)
            max_reconstruction_error=max(max_reconstruction_error,
                (Rotation.from_euler('ZYZ',abc,degrees=True).inv()*r).magnitude())
            previous=abc
    summary['target_segment_replay']=dict(segments=len(records),samples_per_segment=21,
        method='LINEAR_POSITION_SLERP_ROTATION',max_reconstruction_error_rad=max_reconstruction_error,
        physical_execution=False,includes_rejected_target_for_analysis_only=True)
    args.output.mkdir(parents=True,exist_ok=False)
    (args.output/'rotation_replay.json').write_text(json.dumps(summary,indent=2))
    print(json.dumps({k:v for k,v in summary.items() if k!='replay'},indent=2))


if __name__=='__main__':main()
