"""One-factor ServoL timing reference sweep; no ROS, live profile or config edits.

The recorded gaps are fixed inputs to ideal rest-to-rest profiles, NOT a robot
simulation or prediction of tracking error after changing the controller.
"""
import argparse
import csv
import hashlib
import json
import math
from pathlib import Path
import time

import yaml
from analyze_servo_timing import (read_rows, published_commands, align_feedback,
                                 rest_to_rest_time, stats)

ROOT=Path(__file__).resolve().parent
FIELDS=('linear_velocity_mm_s','angular_velocity_deg_s',
        'linear_acceleration_mm_s2','angular_acceleration_deg_s2')


def candidates(base):
    """Explicit one-factor comparisons, plus a labelled combined diagnostic."""
    result=[dict(name='baseline',changed_fields=[],**base)]
    for field in FIELDS:
        for multiplier in (1.5,2.,4.):
            # No rotation-speed increase: do not automatically equate units.
            if field=='angular_velocity_deg_s':continue
            row=dict(base);row[field]*=multiplier
            result.append(dict(name=f'{field}_x{multiplier:g}',changed_fields=[field],**row))
    for multiplier in (2.,4.):
        row=dict(base)
        for field in FIELDS[2:]:row[field]*=multiplier
        result.append(dict(name=f'both_accelerations_x{multiplier:g}',
                           changed_fields=list(FIELDS[2:]),**row))
    return result


def required_times(pairs, profile):
    return [max(rest_to_rest_time(r['translation_tracking_difference_mm'],
                                 profile['linear_velocity_mm_s'],profile['linear_acceleration_mm_s2']),
                rest_to_rest_time(r['rotation_tracking_difference_deg'],
                                 profile['angular_velocity_deg_s'],profile['angular_acceleration_deg_s2']))
            for r in pairs]


def reference_horizon(required, timing_uncertainty, period=.05):
    if not required or period<=0 or timing_uncertainty<0:raise ValueError('invalid_reference_inputs')
    # Finite observed-data envelope only. Not a safety/robust-control bound.
    return math.ceil((max(required)+timing_uncertainty)/period-1e-12)*period


def tune(trials,output):
    if len({p.resolve() for p in trials})!=len(trials):raise ValueError('duplicate_trial')
    if not trials:raise ValueError('no_trials')
    pairs=[]; sources=[]; baseline=None
    for path in trials:
        commands=published_commands(read_rows(path/'servo_commands.jsonl'))
        if not commands:raise ValueError('no_published_commands:'+str(path))
        profile=dict(zip(FIELDS,commands[0]['vel']+commands[0]['acc']))
        if baseline is None:baseline=profile
        if profile!=baseline or any(dict(zip(FIELDS,r['vel']+r['acc']))!=baseline for r in commands):
            raise ValueError('mixed_limits_require_separate_sweep')
        matched=align_feedback(read_rows(path/'predictions.jsonl'),commands)
        if not matched:raise ValueError('no_feedback_pairs:'+str(path))
        pairs.extend(dict(row,trial=str(path.resolve())) for row in matched)
        summary=json.loads((path/'trial_summary.json').read_text())
        sources.append(dict(trial=str(path.resolve()),pair_count=len(matched),
            status=summary['status'],reason=summary.get('reason'),
            hashes={name:hashlib.sha256((path/name).read_bytes()).hexdigest() for name in
                    ['predictions.jsonl','servo_commands.jsonl','trial_summary.json']}))
    uncertainty=max(r['decision_to_publish_skew_s'] for r in pairs)
    results=[];grid=[]
    for profile in candidates(baseline):
        times=required_times(pairs,profile)
        horizon=reference_horizon(times,uncertainty)
        result=dict(profile,rest_to_rest_required_time_s=stats(times),
            observed_pair_count=len(pairs),observed_alignment_skew_max_s=uncertainty,
            finite_sample_reference_horizon_s=horizon,physical_validation='NOT_EXECUTED',
            approved_for_live=False)
        results.append(result)
        for step in range(2,21):
            t=step*.05
            grid.append(dict(name=profile['name'],**{k:profile[k] for k in FIELDS},target_time_s=t,
                ideal_rest_reference_fit_count=sum(x+uncertainty<=t+1e-12 for x in times),
                sample_count=len(times),
                ideal_rest_reference_fit_rate=sum(x+uncertainty<=t+1e-12 for x in times)/len(times),
                actual_tracking_success_rate='NOT_MEASURED'))
    by_name={r['name']:r for r in results}
    def draft(profile_name):
        p=by_name[profile_name]
        return dict(profile_name=profile_name,**{k:p[k] for k in FIELDS},
            target_time_s=p['finite_sample_reference_horizon_s'],
            approved_for_live=False,production_connected=False,
            required_before_physical_test=['JOINT_LIMIT_PREFLIGHT_AND_RUNTIME_GUARD',
                'SAFE_OFF2_NORMAL_OPERATOR_RECOVERY','CURRENT_STOP_PATH_VERIFICATION',
                'FRESH_OPERATOR_AND_HARDWARE_APPROVAL'],
            planning_limiter_changes=False,
            note='Transport-bound candidate only; increasing the planning limiter simultaneously is not authorized by this sweep.')
    plan=dict(stage_0_time_only=draft('baseline'),
        stage_1_acceleration_only_at_stage_0_time=dict(draft('both_accelerations_x2'),
            target_time_s=by_name['baseline']['finite_sample_reference_horizon_s']),
        later_time_reduction_candidate=draft('both_accelerations_x2'),
        one_trial_at_a_time=True,automatic_physical_sweep=False)
    output.mkdir(parents=True,exist_ok=False)
    def save(name,obj):(output/name).write_text(json.dumps(obj,indent=2,allow_nan=False))
    with (output/'reference_sweep.csv').open('w',newline='') as f:
        writer=csv.DictWriter(f,fieldnames=list(grid[0]));writer.writeheader();writer.writerows(grid)
    with (output/'matched_feedback.csv').open('w',newline='') as f:
        writer=csv.DictWriter(f,fieldnames=list(pairs[0]));writer.writeheader();writer.writerows(pairs)
    limits_path=ROOT.parent/'01_configs/safety_limits.yaml'
    save('provenance.json',dict(trials=sources,production_limits=yaml.safe_load(limits_path.read_text()),
        production_limits_sha256=hashlib.sha256(limits_path.read_bytes()).hexdigest(),
        code_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest()))
    save('candidate_profiles.json',results);save('staged_tuning_plan.json',plan)
    summary=dict(status='OFFLINE_REFERENCE_SWEEP_COMPLETE_LIVE_TUNING_BLOCKED',
        trial_count=len(trials),matched_pair_count=len(pairs),
        approximation='Ideal stationary-start/stationary-end reference, no controller/IK/joint/singularity simulation',
        matching_time_is_exact_measurement_timestamp=False,
        caution='Alignment skew is not a bound on controller pose source age. Longer reach time may increase closed-loop lag.',
        baseline=by_name['baseline'],acceleration_candidate=by_name['both_accelerations_x2'],
        staged_plan=plan,production_parameters_changed=False,physical_commands=0,
        joint_limit_fault_resolved=False,stop_ack_failure_resolved=False,
        claim='No claim that any candidate removes 1216 or safely completes a physical episode.')
    save('tuning_summary.json',summary)
    print(json.dumps(summary,indent=2));print('Output:',output)
    return summary


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--trial',type=Path,action='append',required=True)
    parser.add_argument('--output',type=Path,default=ROOT/'real_trials'/time.strftime('%Y%m%d_%H%M%S_servo_offline_tuning'))
    args=parser.parse_args();tune(args.trial,args.output)
