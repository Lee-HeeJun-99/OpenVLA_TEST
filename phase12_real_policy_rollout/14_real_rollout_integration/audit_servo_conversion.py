"""Offline conversion/integration/joint-limit audit; no ROS or runtime changes."""
import argparse
import csv
import hashlib
import json
import math
from pathlib import Path
import time

import numpy as np
from scipy.spatial.transform import Rotation
import yaml
from analyze_servo_timing import read_rows,published_commands,align_feedback,stats
from servo_experimental.controller import ServoShadowController

ROOT=Path(__file__).resolve().parent


def rotation_error(a,b):
    return math.degrees((a*b.inv()).magnitude())


def joint5(row):
    return math.degrees(row['position'][row['names'].index('joint_5')])


def joint_crossing(samples,commands,episode_start,boundary):
    # This boundary is an observed controller alarm value, not a new guard or
    # a claim that the current controller configuration was read/verified.
    cross=next((r['receive'] for r in samples if r['receive']>=episode_start and
                r.get('valid') and joint5(r)<=boundary),None)
    after=[] if cross is None else [r for r in commands if r['published_monotonic']>=cross]
    return dict(observed_alarm_boundary_deg=boundary,
        boundary_current_configuration_verified=False,
        first_crossing_monotonic=cross,
        first_crossing_episode_s=None if cross is None else cross-episode_start,
        publish_returns_after_first_crossing=len(after))


def fixed_goal_probe(limits):
    c=ServoShadowController([400.,0,500,179,-155,3],limits)
    position=[]
    for i in range(150):
        now=i*.05
        # Synthetic source refresh avoids stale-action termination, without
        # moving the goal. NOT a physical command sequence or recorded policy.
        c.set_target([400.8,0,500,179,-155,3],now,now)
        position.append(c.tick(now)['pos'][0]-400.)
    return dict(kind='SYNTHETIC_FIXED_TARGET_CHARACTERIZATION_NOT_ROBOT_RESPONSE',
        requested_delta_mm=.8,max_command_delta_mm=max(position),
        virtual_overshoot_mm=max(position)-.8,final_delta_mm=position[-1],
        last_second_delta_min_mm=min(position[-20:]),last_second_delta_max_mm=max(position[-20:]),
        crossings_of_target=sum((a-.8)*(b-.8)<0 for a,b in zip(position,position[1:])),
        nominal_duration_s=7.5,source_timestamp_refreshed_synthetically=True,
        physical_commands=0)


def audit_trial(path):
    summary=json.loads((path/'trial_summary.json').read_text())
    commands=published_commands(read_rows(path/'servo_commands.jsonl'))
    predictions=read_rows(path/'predictions.jsonl')
    samples=json.loads((path/'jointstate.json').read_text())['samples']
    initial=json.loads((path/'preflight.json').read_text())['initial_pose']
    jt=np.array([r['receive'] for r in samples]);q5=np.array([joint5(r) for r in samples])
    # finish()/duration are adjacent host-monotonic records, not a new ROS time.
    episode_start=summary['stop_requested_monotonic']-summary['duration_s']
    cross=joint_crossing(samples,commands,episode_start,-135.)
    cross['crossing_to_stop_request_s']=(None if cross['first_crossing_monotonic'] is None else
        summary['stop_requested_monotonic']-cross['first_crossing_monotonic'])
    position_errors=[];orientation_errors=[];during_inference=[];decision_times=[]
    for p in predictions:
        current=p['current_controller_pose'];goal=p['action_pipeline']['target_pose_candidate_mm_zyz_deg']
        action=p['action_pipeline']['limited_canonical_action']
        position_errors.append(float(np.linalg.norm(np.array(goal[:3])-np.array(current[:3])-1000*np.array(action[:3]))))
        expected=Rotation.from_rotvec(action[3:6])*Rotation.from_euler('ZYZ',current[3:],degrees=True)
        orientation_errors.append(rotation_error(Rotation.from_euler('ZYZ',goal[3:],degrees=True),expected))
        t0=p['camera_timing']['inference_start_monotonic'];t1=p['camera_timing']['inference_end_monotonic']
        during_inference.append(abs(float(np.interp(t1,jt,q5)-np.interp(t0,jt,q5))))
        decision_times.append(t0+p['action_pipeline']['action_age_sec'])
    previous_position=np.array(initial[:3]);previous_r=Rotation.from_euler('ZYZ',initial[3:],degrees=True)
    previous_v=np.zeros(3);previous_w=np.zeros(3)
    integration_linear=[];integration_rotation=[];steps=[]
    for cmd in commands:
        v=np.array(cmd['translation_velocity_mm_s']);w=np.array(cmd['rotation_velocity_rad_s']);dt=cmd['actual_tick_dt_s']
        p=np.array(cmd['pos'][:3]);r=Rotation.from_euler('ZYZ',cmd['pos'][3:],degrees=True)
        integration_linear.append(float(np.linalg.norm(p-(previous_position+(previous_v+v)*(.5*dt)))))
        integration_rotation.append(rotation_error(r,Rotation.from_rotvec((previous_w+w)*(.5*dt))*previous_r))
        steps.append(rotation_error(r,previous_r))
        previous_position,previous_r,previous_v,previous_w=p,r,v,w
    pairs=align_feedback(predictions,commands)
    result=dict(trial=str(path.resolve()),status=summary['status'],reason=summary.get('reason'),
        prediction_count=len(predictions),publish_return_count=len(commands),
        duration_s=summary['duration_s'],stop_ack=summary.get('stop_ack'),
        joint5_min_deg=float(min(q5)),joint5_last_deg=float(q5[-1]),joint_crossing=cross,
        conversion_reconstruction=dict(max_position_error_mm=max(position_errors),max_rotation_error_deg=max(orientation_errors)),
        interpolation_reconstruction=dict(max_position_error_mm=max(integration_linear),max_rotation_error_deg=max(integration_rotation)),
        rotation_step_deg=stats(steps),commanded_rotation_arc_deg=sum(steps),
        commanded_angular_speed_deg_s=stats([float(np.linalg.norm(c['rotation_velocity_rad_s'])*180/math.pi) for c in commands]),
        joint5_motion_during_inference_deg=stats(during_inference),
        policy_decision_interval_s=stats(np.diff(decision_times)),
        translation_tracking_difference_mm=stats([p['translation_tracking_difference_mm'] for p in pairs]),
        rotation_tracking_difference_deg=stats([p['rotation_tracking_difference_deg'] for p in pairs]),
        ik_solution_space_logged_per_prediction=all('solution_space' in p.get('controller_feedback',{}) for p in predictions),
        interpretation_limits=['No IK/joint path simulation. SO(3) continuity does not verify solution-space continuity.',
            'Controller pose source time is unverified; feedback/command matching is approximate.',
            'Physical paths use different camera observations/policy histories; not a controlled MoveLine-versus-ServoL A/B.',
            'Missing pre-limit guard explains continued dispatch, not why the policy trajectory first approaches that limit.'])
    timeline=[dict(episode_s=r['receive']-episode_start,joint5_deg=joint5(r),
                   joint5_velocity_deg_s=math.degrees(r['velocity'][r['names'].index('joint_5')]))
              for r in samples if r['receive']>=episode_start and r.get('valid')]
    return result,timeline


def audit(output):
    output.mkdir(parents=True,exist_ok=False)
    def save(name,obj):(output/name).write_text(json.dumps(obj,indent=2,allow_nan=False))
    results=[]
    for name in ['20261106_110547_arm_only_servo_episode','20261106_111406_arm_only_servo_episode',
                 '20261106_125319_arm_only_servo_episode']:
        p=ROOT/'real_trials'/name
        result,timeline=audit_trial(p);results.append(result);save(name+'.json',result)
        with (output/(name+'_joint5.csv')).open('w',newline='') as f:
            writer=csv.DictWriter(f,fieldnames=list(timeline[0]));writer.writeheader();writer.writerows(timeline)
    old=ROOT/'real_trials/20261014_arm_only_70s_episode_001'
    old_summary=json.loads((old/'trial_summary.json').read_text())
    old_commands=read_rows(old/'arm_commands.jsonl')
    old_joints=json.loads((old/'jointstate.json').read_text())['samples']
    save('moveline_reference.json',dict(source=str(old),status=old_summary['status'],
        duration_s=old_summary['duration_s'],arm_commands=old_summary['arm_commands'],
        joint5_min_deg=min(joint5(r) for r in old_joints),
        command_start_interval_s=stats(np.diff([r['receipt']['sent_at'] for r in old_commands])),
        caveat='Same model/checkpoint and arm-only scope, but different date/scene, action limits and control history. Not causal A/B.'))
    limits=yaml.safe_load((ROOT.parent/'01_configs/safety_limits.yaml').read_text())
    probe=fixed_goal_probe(limits);save('fixed_goal_characterization.json',probe)
    save('source_provenance.json',{str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in
        [Path(__file__),ROOT/'start_servo_episode_confirmed.py',ROOT/'servo_experimental/controller.py',
         ROOT/'run_arm_only_episode.py',ROOT.parent/'02_safety/safety_pipeline.py']})
    findings=dict(confirmed=[
        'Live servo RuntimeState leaves joint_limits_ok at default True; measured joint limits/target IK not evaluated.',
        'State-code check happens per prediction and catches SAFE_OFF2 only after the controller enters it.',
        'Servo interpolation state persists separately from measured CurrentPosX; no following-error guard.',
        'Measured pose for target construction is read after inference while robot continues moving.',
        'Discrete servo interpolator overshoots/oscillates in a command-free fixed-goal characterization.',
        'Per-prediction CurrentPosX slices first six values and discards controller solution-space value.',
        'Driver servol callback ignores DRFL boolean return; publish return is not robot acceptance.'],
        not_observed=['No extra 1000x scaling in logged target construction.',
            'No repeated raw-delta addition at each 20Hz tick.',
            'Logged command SO(3) increments match the interpolator; no 180-degree jump in command rotations, not an IK-branch guarantee.'],
        not_proven=['Unique root cause of joint5 drift: model output, timing mismatch, tracking, IK/singularity contribution.',
            'The synthetic fixed-goal overshoot caused either physical limit fault.',
            'HOLD false is a stop-system defect rather than a consequence of already being SAFE_OFF2.'],
        fixed_goal_probe=probe,physical_commands=0,runtime_code_changed=False,
        recommendation='Do not continue full-episode tuning before pre-limit protection and interpolator/feedback behavior are addressed.')
    save('audit_findings.json',findings)
    print(json.dumps(dict(output=str(output),trials=results,findings=findings),indent=2))


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',type=Path,default=ROOT/'real_trials'/time.strftime('%Y%m%d_%H%M%S_servo_conversion_audit'))
    audit(parser.parse_args().output)
