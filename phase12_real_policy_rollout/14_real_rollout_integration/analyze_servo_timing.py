"""Offline timing/feedback audit. No ROS import, command transport or config write.

This is NOT a plant simulator. Changing limits changes the closed-loop trajectory;
the recorded poses cannot predict that response. Rest-to-rest calculations are
explicit reference profiles, not the actual moving ServoL boundary conditions.
"""
import argparse
import bisect
import csv
import json
import math
from pathlib import Path
import re
import time

import numpy as np
from scipy.spatial.transform import Rotation


def rest_to_rest_time(distance, velocity, acceleration):
    if not all(math.isfinite(v) for v in (distance,velocity,acceleration)):
        raise ValueError('nonfinite_profile')
    if distance<0 or velocity<=0 or acceleration<=0:raise ValueError('invalid_profile')
    if distance<=velocity*velocity/acceleration:
        return 2*math.sqrt(distance/acceleration)
    return distance/velocity+velocity/acceleration


def pose_difference(command, measured):
    a=np.asarray(command,dtype=float);b=np.asarray(measured,dtype=float)
    if a.shape!=(6,) or b.shape!=(6,) or not np.all(np.isfinite(np.r_[a,b])):
        raise ValueError('invalid_pose')
    linear=float(np.linalg.norm(a[:3]-b[:3]))
    angular=math.degrees((Rotation.from_euler('ZYZ',a[3:],degrees=True)*
                         Rotation.from_euler('ZYZ',b[3:],degrees=True).inv()).magnitude())
    return linear,angular


def stats(values):
    if not len(values):return dict(count=0)
    a=np.asarray(values,dtype=float)
    return dict(count=len(a),min=float(np.min(a)),median=float(np.median(a)),
                p95=float(np.quantile(a,.95)),p99=float(np.quantile(a,.99)),max=float(np.max(a)))


def read_rows(path):
    return [json.loads(line) for line in path.read_text().splitlines() if line.strip()]


def published_commands(rows):
    commands=[]
    for i,row in enumerate(rows[:-1]):
        event=rows[i+1]
        if 'pos' in row and event.get('event')=='PUBLISH_RETURNED_NOT_VENDOR_ACK':
            commands.append(dict(row, published_monotonic=event['published_monotonic'],
                                 published_wall_time=event['logger_wall_time']))
    if any(b['published_monotonic']<=a['published_monotonic'] for a,b in zip(commands,commands[1:])):
        raise ValueError('nonmonotonic_publish_times')
    return commands


def align_feedback(predictions, commands):
    times=[row['published_monotonic'] for row in commands]
    pairs=[]
    for row in predictions:
        # CurrentPosX completed immediately BEFORE pipeline.inspect(). Its exact
        # source/receive time was not logged per sample in the old launcher.
        # This decision time is an upper bound, not an exact measurement time.
        upper=(row['camera_timing']['inference_start_monotonic']+
               row['action_pipeline']['action_age_sec'])
        i=bisect.bisect_right(times,upper)-1
        if i<0:continue  # Never substitute a future command.
        cmd=commands[i]
        linear,angular=pose_difference(cmd['pos'],row['current_controller_pose'])
        target_time=cmd['time'];vel=cmd['vel'];acc=cmd['acc']
        if not math.isfinite(target_time) or target_time<=0:raise ValueError('unsupported_target_time')
        rest_reference=max(rest_to_rest_time(linear,vel[0],acc[0]),
                           rest_to_rest_time(angular,vel[1],acc[1]))
        pairs.append(dict(step=row['step'],feedback_time_upper_bound=upper,
            matched_publish_time=cmd['published_monotonic'],
            decision_to_publish_skew_s=upper-cmd['published_monotonic'],
            translation_tracking_difference_mm=linear,rotation_tracking_difference_deg=angular,
            target_time_s=target_time,linear_velocity_limit_mm_s=vel[0],
            angular_velocity_limit_deg_s=vel[1],linear_acceleration_limit_mm_s2=acc[0],
            angular_acceleration_limit_deg_s2=acc[1],
            speed_only_reference_time_s=max(linear/vel[0],angular/vel[1]),
            exceeds_even_full_target_time_speed_budget=linear>vel[0]*target_time or angular>vel[1]*target_time,
            rest_to_rest_reference_time_s=rest_reference,
            # Triangular rest-to-rest profile that would cover the observed gap.
            # This is a mathematical reference, NOT a permissible robot setting.
            triangular_linear_peak_mm_s=2*linear/target_time,
            triangular_angular_peak_deg_s=2*angular/target_time,
            triangular_linear_acceleration_mm_s2=4*linear/target_time**2,
            triangular_angular_acceleration_deg_s2=4*angular/target_time**2))
    return pairs


def alarm_records(path, start, end):
    lines=path.read_text().splitlines();records=[]
    for i,line in enumerate(lines):
        match=re.search(r'^([0-9.]+).*\bindex\s*:\s*(\d+)',line)
        if not match or not start<=float(match[1])<=end:continue
        params=[]
        for following in lines[i+1:i+9]:
            if 'param :' in following:params.append(following.split('param :',1)[1].strip())
            elif params:break
        records.append(dict(wall_time=float(match[1]),index=int(match[2]),parameters=params))
    return records


def analyze(trial, output, driver_log=None):
    commands=published_commands(read_rows(trial/'servo_commands.jsonl'))
    predictions=read_rows(trial/'predictions.jsonl')
    pairs=align_feedback(predictions,commands)
    if not pairs:raise ValueError('no_usable_command_feedback_pairs')
    output.mkdir(parents=True,exist_ok=False)
    def save(name,value):(output/name).write_text(json.dumps(value,indent=2,allow_nan=False))
    with (output/'matched_feedback.csv').open('w',newline='') as f:
        writer=csv.DictWriter(f,fieldnames=list(pairs[0]));writer.writeheader();writer.writerows(pairs)
    distribution={key:stats([r[key] for r in pairs]) for key in [
        'translation_tracking_difference_mm','rotation_tracking_difference_deg',
        'decision_to_publish_skew_s','speed_only_reference_time_s','rest_to_rest_reference_time_s',
        'triangular_linear_peak_mm_s','triangular_angular_peak_deg_s',
        'triangular_linear_acceleration_mm_s2','triangular_angular_acceleration_deg_s2']}
    # Separate translation and rotation units. None is automatically selected.
    candidates=[]
    for label,vl,vr,al,ar in [('recorded',20,20,20,20),('linear_margin',40,20,40,40),
                              ('acceleration_margin',40,20,80,80),('larger_linear_margin',60,20,120,120)]:
        required=[max(rest_to_rest_time(r['translation_tracking_difference_mm'],vl,al),
                      rest_to_rest_time(r['rotation_tracking_difference_deg'],vr,ar)) for r in pairs]
        candidates.append(dict(label=label,linear_velocity_mm_s=vl,angular_velocity_deg_s=vr,
            linear_acceleration_mm_s2=al,angular_acceleration_deg_s2=ar,
            rest_to_rest_reference_required_time_s=stats(required),
            rest_reference_fits_recorded_target_time_count=sum(t<=r['target_time_s'] for t,r in zip(required,pairs)),
            sample_count=len(pairs),production_approved=False,
            actual_controller_response='NOT_SIMULATED_NOT_IDENTIFIABLE_FROM_RECORDED_REPLAY'))
    summary=json.loads((trial/'trial_summary.json').read_text())
    alarms=[] if driver_log is None else alarm_records(driver_log,commands[0]['published_wall_time'],
                                                       commands[-1]['published_wall_time']+.1)
    alarm_counts={str(i):sum(r['index']==i for r in alarms) for i in sorted({r['index'] for r in alarms})}
    result=dict(source_trial=str(trial.resolve()),published_return_count=len(commands),
        prediction_count=len(predictions),matched_feedback_pair_count=len(pairs),
        distributions=distribution,
        full_target_time_speed_budget_exceeded_count=sum(r['exceeds_even_full_target_time_speed_budget'] for r in pairs),
        actual_trial_status=summary['status'],actual_trial_reason=summary.get('reason'),
        driver_alarm_counts=alarm_counts,production_configuration_changed=False,
        selected_velocity=None,selected_acceleration=None,selected_target_time=None,
        tuning_status='NO_VALIDATED_SETTING_FROM_OFFLINE_LOGS',
        limitations=[
            'Matching time is an upper bound; exact controller pose source/receive timestamp was not logged per prediction.',
            'Nearest previous published command, not a vendor-acknowledged/executed command.',
            'Feedback/controller timing skew is reported; no tolerance invented to hide it.',
            'Speed-budget calculation generously allows the full target time, not its already elapsed remainder.',
            'Rest-to-rest assumes zero initial/final Cartesian velocity; actual streaming robot is moving.',
            'No Jacobian, joint constraints, controller profile, singularity dynamics or actual initial velocity is simulated.',
            'Replaying recorded poses does not predict the changed closed-loop response under different limits.',
            'SAFE_OFF2/joint-limit fault and failed HOLD ACK must not be treated as a speed-tuning success.'],
        physical_commands=0)
    save('timing_analysis.json',result);save('candidate_reference_profiles.json',candidates)
    save('driver_alarms.json',alarms)
    print(json.dumps(dict(output=str(output),pairs=len(pairs),speed_budget_exceeded=result['full_target_time_speed_budget_exceeded_count'],
        distributions=distribution,alarms=alarm_counts,production_configuration_changed=False,
        tuning_status=result['tuning_status'],physical_commands=0),indent=2))
    return result


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--trial',type=Path,required=True)
    parser.add_argument('--driver-log',type=Path)
    parser.add_argument('--output',type=Path,default=Path(__file__).parent/'real_trials'/time.strftime('%Y%m%d_%H%M%S_servo_timing_analysis'))
    args=parser.parse_args();analyze(args.trial,args.output,args.driver_log)
