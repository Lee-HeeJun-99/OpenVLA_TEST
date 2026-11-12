"""Non-driving joint inventory, SpeedL forecast replay and stop-failure audit.

No ROS, HTTP, controller setter, publisher or physical transport is created.
"""
import argparse
import bisect
import csv
import hashlib
import json
from pathlib import Path
import subprocess
import xml.etree.ElementTree as ET
import numpy as np
import yaml
from speedl_experimental.controller import SpeedLShadowController
from speedl_experimental.joint_safety import JointRangePredictor, ordered_joint_state, CANONICAL_JOINTS

ROOT = Path(__file__).resolve().parent
BUNDLE = ROOT.parents[2]


def read(p):return json.loads(p.read_text())
def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def lines(p):return [json.loads(x) for x in p.read_text().splitlines() if x.strip()]


def bounds(rows):
    if not rows:return dict(count=0, min_deg=None, max_deg=None)
    q = np.rad2deg(np.array(rows))
    return dict(count=len(q), min_deg=q.min(0).tolist(), max_deg=q.max(0).tolist())


def rows_in(value):
    if isinstance(value, list):
        for v in value:yield from rows_in(v)
    elif isinstance(value, dict):
        if ('names' in value or 'name' in value) and 'position' in value:
            yield value
        else:
            for v in value.values():
                if isinstance(v, (list, dict)):yield from rows_in(v)


def csv_write(p, rows):
    if not rows:return
    keys=list(dict.fromkeys(k for r in rows for k in r))
    with p.open('x', newline='') as f:
        w=csv.DictWriter(f, fieldnames=keys);w.writeheader();w.writerows(rows)


def run(output):
    output.mkdir(parents=True, exist_ok=False)
    def save(n, o):(output/n).write_text(json.dumps(o, indent=2, allow_nan=False))
    config= yaml.safe_load((ROOT/'configs/joint_safety.yaml').read_text())
    urdf=BUNDLE/'validation/a0509_project/urdf/a0509.urdf'
    predictor=JointRangePredictor(urdf, config)
    limits=yaml.safe_load((ROOT.parent/'01_configs/safety_limits.yaml').read_text())
    train_root=BUNDLE/'data/real_world/raw_dataset_oft'
    training_config=read(BUNDLE/'models/vanilla_s1_balanced_step8130/training_config.json')
    expected=Path(training_config['dataset_root'])
    datasets=[]; dataset_values=[]
    # Only the canonical metadata splits: exclude backups, copied condition
    # datasets and duplicate steps/steps_with_actions representations.
    for label, directory in [('local_portable_not_checkpoint_training', train_root),
                             ('checkpoint_training', expected)]:
        if not directory.exists():continue
        for split in ('train', 'val'):
            p=directory/'metadata'/f'{split}.jsonl'
            if not p.exists():continue
            values=[];invalid=0;seen=set()
            for row in lines(p):
                key=(row.get('episode_id'), row.get('source_step_index', row.get('step_index')))
                if key in seen:continue
                seen.add(key)
                q=row.get('joint_position_rad', [])
                # Portable manifest explicitly defines first six ordered joints;
                # remaining entries belong to gripper, not J7/J8 arm joints.
                if len(q)<6 or not np.all(np.isfinite(q[:6])):invalid+=1;continue
                values.append(q[:6])
            actual=sha(p);expected_sha=training_config.get('dataset_hashes', {}).get(f'metadata/{split}.jsonl')
            record=dict(dataset=label, split=split, path=str(p), sha256=actual,
                expected_checkpoint_hash=expected_sha, checkpoint_hash_matches=actual==expected_sha,
                joint_order=list(CANONICAL_JOINTS), unit='radian', invalid_rows=invalid,
                episode_count=len(set(k[0] for k in seen)), **bounds(values))
            datasets.append(record)
            dataset_values.extend(values)
    save('dataset_joint_ranges.json', dict(sources=datasets, checkpoint_training_root=str(expected),
        checkpoint_training_root_available=expected.exists(),
        complete_checkpoint_training_verified=any(d['dataset']=='checkpoint_training' and d['split']=='train' and d['checkpoint_hash_matches'] for d in datasets),
        warning='Local portable metadata is NOT the 12998-sample checkpoint training corpus unless its exact hashes match.'))
    live=[]; all_values=[]; exclusions=[]; trial_rows={}; stop_audit=[]
    for d in sorted((ROOT/'real_trials').iterdir()):
        p=d/'trial_summary.json'
        if not p.exists():continue
        s=read(p)
        physical=s.get('arm_commands',s.get('physical_arm_commands',0)) or s.get('servo_publish_attempts',0)
        if not physical:continue
        paths=[d/'jointstate.json'] if (d/'jointstate.json').exists() else list(d.glob('jointstate_*.json'))
        values=[];samples=[];invalid=0
        for source in paths:
            for row in rows_in(read(source)):
                try:q=ordered_joint_state(row)
                except (ValueError, TypeError):invalid+=1;continue
                values.append(q);samples.append(row)
        if not values:
            exclusions.append(dict(trial=d.name, reason='no_valid_measured_joint_rows'));continue
        all_values.extend(values)
        live.append(dict(trial=d.name, source_files=[str(x) for x in paths], invalid_rows=invalid,
            status=s.get('status'), **bounds(values)))
        trial_rows[d.name]=samples
        if s.get('stop_commands',0):
            stop_audit.append(dict(trial=d.name, request_time=s.get('stop_requested_monotonic'),
                stop_ack=s.get('stop_ack'), stop_response=s.get('stop_response'),
                original_reason=s.get('reason'), stop_error=s.get('stop_error'),
                physical_stop_confirmed=s.get('stop_confirmation',{}).get('confirmed'),
                interpretation='ACK false after robot abnormal state does not uniquely identify root cause; inspect paired alarms. Missing stop proof stays UNKNOWN.'))
    save('real_joint_ranges.json',dict(trials=live, aggregate=bounds(all_values), exclusions=exclusions,
        measurement='Actual received JointState; reordered by name; nominal frames not training commanded Cartesian poses.'))
    tree=ET.parse(urdf).getroot();urdf_rows=[]
    for n in CANONICAL_JOINTS:
        e=tree.find(f"joint[@name='{n}']/limit")
        urdf_rows.append(dict(joint=n, lower_deg=float(np.degrees(float(e.get('lower')))),
            upper_deg=float(np.degrees(float(e.get('upper')))), max_speed_deg_s=float(np.degrees(float(e.get('velocity'))))))
    limits_inventory=dict(conservative_offline_contract=config, urdf=urdf_rows,
        controller_current_lower_deg=None, controller_current_upper_deg=None,
        historical_controller_evidence=dict(joint='joint_5', minimum_deg=-135,
            source='20261106_z_direction_audit_02/driver_alarm_evidence.json', current_session_verified=False),
        distinction='URDF physical/model limits, manual defaults, and active controller safety settings are different sources. No controller writes/readback attempted.')
    save('limits_inventory.json',limits_inventory)
    comparisons=[]
    for label, b in [('local_available_train', None),
                     ('real_execution', bounds(all_values))]:
        if label=='local_available_train':
            ds=next((d for d in datasets if d['dataset']=='local_portable_not_checkpoint_training' and d['split']=='train'),None)
            if ds is None:continue
            b=ds
        if not b['count']:continue
        for i,n in enumerate(CANONICAL_JOINTS):
            lo,hi=b['min_deg'][i], b['max_deg'][i]
            comparisons.append(dict(source=label,joint=n,observed_min_deg=lo,observed_max_deg=hi,
                reference_lower_deg=config['lower_deg'][i],reference_upper_deg=config['upper_deg'][i],
                lower_extension_deg=max(0,config['lower_deg'][i]-lo),
                upper_extension_deg=max(0,hi-config['upper_deg'][i]),
                needed_envelope_lower_deg=min(config['lower_deg'][i],lo),
                needed_envelope_upper_deg=max(config['upper_deg'][i],hi),
                setting_changed=False,reference_is_current_controller=False,
                interpretation='Observed coverage only, not safety approval or proof expansion is required.'))
    csv_write(output/'range_comparison.csv', comparisons)
    save('required_ranges.json',comparisons)
    # Replay recorded policy targets against nearby real joint feedback.
    predictions=[]
    for name in ['20261106_110547_arm_only_servo_episode','20261106_111406_arm_only_servo_episode','20261106_125319_arm_only_servo_episode']:
        samples=sorted([s for s in trial_rows.get(name,[]) if 'receive' in s], key=lambda x:x['receive'])
        if not samples:continue
        times=[s['receive'] for s in samples]
        for p in lines(ROOT/'real_trials'/name/'predictions.jsonl'):
            row=dict(trial=name,step=p.get('step'),command_requested=False,command_issued=False)
            try:
                target=p['action_pipeline']['target_pose_candidate_mm_zyz_deg']; current=p['current_controller_pose']
                source=p['camera_timing']['inference_start_monotonic']
                timing=p.get('controller_feedback')
                if timing and 'response_received_monotonic' in timing:
                    now=timing['response_received_monotonic']
                    row['time_alignment']='POSE_SERVICE_RESPONSE_TIME_NOT_CONTROLLER_MEASUREMENT_TIME'
                else:
                    now=source+p['action_pipeline']['action_age_sec']
                    row['time_alignment']='RECORDED_POLICY_DECISION_TIME_POSE_MEASUREMENT_TIME_UNKNOWN'
                row['controller_measurement_time_verified']=False
                idx=bisect.bisect_right(times,now)-1
                if idx<0:raise ValueError('no_previous_jointstate')
                c=SpeedLShadowController(limits)
                cand=c.candidate(current,target,now=now,source_time=source)
                result=predictor.assess(samples[idx],cand['vel'],now=now,target_pose=target)
                row.update(candidate_velocity=cand['vel'],joint_sample_age_s=now-times[idx],assessment=result)
            except (ValueError, KeyError, TypeError, RuntimeError) as exc:row['error']=str(exc)
            predictions.append(row)
    with (output/'speedl_joint_forecasts.jsonl').open('x') as f:
        for row in predictions:f.write(json.dumps(row,allow_nan=False)+'\n')
    from collections import Counter
    forecast_summary=dict(samples=len(predictions), nominal_geometry_pass=sum(r.get('assessment',{}).get('allowed',False) for r in predictions),
        physical_dispatch_allowed=sum(r.get('assessment',{}).get('physical_dispatch_allowed',False) for r in predictions),
        reasons=dict(Counter(r.get('error') or r.get('assessment',{}).get('reason','UNKNOWN') for r in predictions)),
        rejection_stages=dict(Counter(r.get('assessment',{}).get('rejection_stage','input_or_missing_data') for r in predictions if not r.get('assessment',{}).get('allowed'))),
        mode='Independent first ticks from recorded target/pose; not hypothetical full closed-loop trajectory.',
        controller_limit_change=False,physical_commands=0)
    save('speedl_forecast_summary.json',forecast_summary)
    alarms_path=ROOT/'real_trials/20261106_z_direction_audit_02/driver_alarm_evidence.json'
    alarms=read(alarms_path)
    save('stop_failure_analysis.json',dict(trials=stop_audit,
        joint_limit_alarms=[a for a in alarms if a['index']==9008],
        conclusion='Two failed ServoL runs have J5 limit alarm before stop request, then state10/SAFE_OFF2 and success=false. No evidence uniquely distinguishes already-stopped rejection, SDK false, state/authority or stop defect. No automatic recovery/retry is introduced.',
        new_rule='ACK plus fresh STANDBY plus post-request continuous exact-zero joint window; failure/unknown permanently inhibits this trial.'))
    save('provenance.json',dict(git_head=subprocess.run(['git','rev-parse','HEAD'],cwd=ROOT,capture_output=True,text=True).stdout.strip(),
        code={str(p):sha(p) for p in [Path(__file__),ROOT/'speedl_experimental/joint_safety.py',ROOT/'servo_experimental/stop_evidence.py',ROOT/'configs/joint_safety.yaml']},
        urdf=str(urdf),urdf_sha256=sha(urdf), dataset_config=training_config, physical_commands=0,
        no_controller_changes=True, no_server_requests=True))
    print(json.dumps(dict(output=str(output),dataset_sources=len(datasets),real_trials=len(live),
        real_aggregate=bounds(all_values),forecast=forecast_summary),indent=2))


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--output',required=True,type=Path)
    run(p.parse_args().output)
