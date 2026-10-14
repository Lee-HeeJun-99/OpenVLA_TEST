"""Persist a blocked attempt; no ROS initialization or physical commands."""
import argparse,hashlib,json,math,subprocess,time
from pathlib import Path
HERE=Path(__file__).resolve().parent;REPO=HERE.parents[1]
TARGET=[-182.55277,-1.07182,-41.00585,1.25662,-113.50009,1.49975]


def save(output):
    def write(name,value):(output/name).write_text(json.dumps(value,indent=2))
    def read(name):return json.loads((output/name).read_text())
    git=lambda *args:subprocess.check_output(['git',*args],cwd=REPO,text=True).strip()
    write('git_state.json',dict(branch=git('branch','--show-current'),head=git('rev-parse','HEAD'),status_short=git('status','--short')))
    files=list((HERE/'configs').glob('*.yaml'))+[HERE.parent/'01_configs/safety_limits.yaml']
    write('config_snapshot.json',{str(p.relative_to(REPO)):dict(sha256=hashlib.sha256(p.read_bytes()).hexdigest(),text=p.read_text()) for p in files})
    summary=read('summary.json');joints=read('jointstate_samples.json');row=joints[-1];byname=dict(zip(row['names'],row['position']))
    actual=[math.degrees(byname[f'joint_{i}']) for i in range(1,7)];errors=[a-b for a,b in zip(actual,TARGET)]
    write('start_pose.json',dict(current_deg=actual,target_deg=TARGET,error_deg=errors,max_abs_error_deg=max(map(abs,errors)),pass_existing_tolerance=max(map(abs,errors))<=.5,reset_executed=False))
    write('jointstate_current.json',summary)
    write('hardware_current.json',dict(scalar_responses=read('hardware_getters.json'),authority='UNKNOWN',servo='UNKNOWN',protective_stop='UNKNOWN',emergency_stop='UNKNOWN'))
    write('operator_confirmation.json',dict(status='FRESH_CONFIRMATION_REQUIRED',confirmed_wall_time=None,old_approval_reused=False,operator_present=None,workspace_clear=None,estop_accessible=None,robot_stationary=None))
    write('tcp_execution_acceptance.json',dict(classification='TCP_ZERO_OFFSET_SUPPORTED_NOT_VERIFIED',condition_candidate='ZERO_OFFSET_CONSISTENCY_SUPPORTED',accepted=False,explicit_engineering_approval_received=False,active_tcp_name_not_directly_observed=True,evidence_source='20261014_zero_offset_tcp_validation_01',formal_verified=False))
    write('gripper_mapping.json',dict(classification='GRIPPER_MAPPING_UNVERIFIED',config='configs/gripper_hardware.yaml',polarity_confirmed=False,abort_value=None,physical_commands=0))
    write('initial_gripper_state.json',dict(initial_gripper_state='UNKNOWN',expected_initial_gripper_state='OPEN',match=None))
    for name in ('minimum_motion_result.json','stop_path_result.json'):
        write(name,dict(status='NOT_EXECUTED',physical_commands=0,reason='FRESH_SAFETY_CONFIRMATION_AND_TCP_EXECUTION_ACCEPTANCE_MISSING'))
    write('minimum_motion_request.json',dict(status='NOT_CREATED',requested_delta='BASE X +0.5 mm',physical_commands=0))
    write('fresh_prediction.json',dict(status='NOT_EXECUTED',reason='PHYSICAL_STAGE_NOT_AUTHORIZED; no further prediction-only validation requested',physical_commands=0))
    write('candidate_action_pipeline.json',dict(status='NOT_COMPUTED_FOR_DISPATCH',reason='TCP_EXECUTION_ACCEPTANCE_MISSING',actual_delivered_action=None))
    directory=output/'short_horizon';directory.mkdir(exist_ok=False)
    for name in ('observations.jsonl','predictions.jsonl','commands.jsonl'):(directory/name).touch()
    write('short_horizon/runtime_events.json',[])
    write('short_horizon/trial_summary.json',dict(status='NOT_STARTED',runtime_valid=None,task_success=None,prediction_count=0,command_count=0,gripper_command_count=0,estop_used=None,unexpected_motion=None))
    text=f'''# Physical short-horizon attempt — blocked before dispatch

Current scalar state REAL/AUTO/STANDBY; current readiness RUNTIME_READY, fault null. Post-ready source max {summary['current_session_runtime']['max_source_gap_s']*1000:.3f}ms, receive max {summary['current_session_runtime']['max_receive_gap_s']*1000:.3f}ms; >=100ms events 0. Start pose max error {max(map(abs,errors)):.9f} degrees within0.5; no reset motion. Camera stream and strict current model identity PASS. No new model prediction requested.

Fresh operator presence/workspace/E-stop/stationary and authority/servo/protection confirmations were requested together, but no answer/evidence was received in this turn. Zero-offset consistency engineering acceptance was also requested and remains unapproved; formal TCP classification unchanged. Gripper polarity/abort output/initial physical state remain unknown.

No minimum-motion, physical Stop/Hold test or short-horizon command dispatched. An idle Stop ACK would not alone prove physical stopping response, and no such test was fabricated. All physical command counts0. E-stop usage/unexpected motion not observed by an operator, so null rather than invented NO.

Existing short-horizon config is gripper-disabled and Controller.step rejects close>=0.7. Full-task alone dispatches gripper candidates. Therefore verified mapping alone would still not make the current short-horizon a gripper-inclusive full-policy evaluation. Any later arm-only mode must be labeled SYSTEM_INTEGRATION_TEST, not policy task success. No protocol/config was silently changed.

Final: **PHYSICAL_SHORT_HORIZON_BLOCKED**. Current13mm threshold preserved. Next: provide fresh safety values, explicit TCP engineering acceptance, actual gripper initial state and physically verified IO/abort mapping. Minimum-motion and Stop-path proof must then be completed before a single authorized short-horizon. Automatic batch/retry/home/recovery disabled. No old manual confirmation reused and no physical task SUCCESS/FAILURE/INVALID fabricated.
'''
    (output/'FINAL_PHYSICAL_SHORT_HORIZON_REPORT.md').write_text(text)
    print(json.dumps(dict(classification='PHYSICAL_SHORT_HORIZON_BLOCKED',physical_commands=0,start_pose_max_error_deg=max(map(abs,errors)))))


if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('output',type=Path);save(parser.parse_args().output.resolve())
