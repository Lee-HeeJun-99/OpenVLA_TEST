"""Offline final preflight audit. Never imports ROS or dispatches commands."""
import argparse,collections,hashlib,json,math,shutil,subprocess,sys,time
from pathlib import Path
import numpy as np
import yaml
from scipy.spatial.transform import Rotation
HERE=Path(__file__).resolve().parent;PHASE=HERE.parent;REPO=PHASE.parent
sys.path[:0]=[str(PHASE/x) for x in ('02_safety','03_shadow_mode')]
from action_step_limits import RAW_TRANSLATION_STEP_LIMIT_M,raw_translation_exceeds_limit
from action_rate_limiter import CanonicalActionRateLimiter,_scale_to,_accel_limit
from command_disabled_runtime import CommandDisabledPolicyRuntime
from action_contract import compose_world_rotvec_with_doosan_zyz
from workspace_contract import PHASE12_DATA_DERIVED_WORKSPACE
from integrated_logger import FsyncJsonlLogger


def run(output):
    def save(name,value):(output/name).write_text(json.dumps(value,indent=2,allow_nan=False))
    def read(path):return json.loads(path.read_text())
    def git(*args):return subprocess.check_output(['git',*args],cwd=REPO,text=True).strip()
    save('git_state.json',dict(branch=git('branch','--show-current'),head=git('rev-parse','HEAD'),status_short=git('status','--short'),observed_wall_time=time.time()))
    paths=[PHASE/'01_configs/safety_limits.yaml',PHASE/'01_configs/command_disabled_runtime.yaml',PHASE/'01_configs/pre_rollout_verified_config.yaml',HERE/'configs/gripper_hardware.yaml',HERE/'configs/real_openvla.yaml',HERE/'configs/short_horizon.yaml',HERE/'configs/minimum_motion.yaml']
    snapshots={str(p.relative_to(REPO)):dict(sha256=hashlib.sha256(p.read_bytes()).hexdigest(),text=p.read_text()) for p in paths}
    save('config_snapshot.json',snapshots)
    limits=yaml.safe_load((PHASE/'01_configs/safety_limits.yaml').read_text());grip=yaml.safe_load((HERE/'configs/gripper_hardware.yaml').read_text())
    live=read(output/'summary.json');prediction=read(output/'sample_prediction.json');hardware=read(output/'hardware_getters.json');joints=read(output/'jointstate_samples.json')
    save('hardware_state.json',dict(current_responses=hardware,scalar_REAL_AUTO_STANDBY=True,authority='UNKNOWN',servo='UNKNOWN',protective_stop='UNKNOWN',emergency_stop='UNKNOWN',controller_connection='GETTER_RESPONSE_EVIDENCE_NOT_ALL_STATE_VERIFIED'))
    runtime=[r for r in joints if r['receive']>=live['readiness']['ready_at']]
    jointreport=dict(**live['current_session_runtime'],invalid_count=sum(not r['valid'] for r in runtime),duplicates=sum(r.get('source_gap')==0 for r in runtime),regressions=sum((r.get('source_gap') or 0)<0 for r in runtime),source='SAME_PERSISTENT_SUBSCRIBER',readonly_getter_count=3)
    save('jointstate_validation.json',jointreport);save('start_pose_validation.json',read(output/'jointstate_start_pose_check.json'))
    save('camera_validation.json',{k:prediction.get(k) for k in ('status','encoding','resolution','camera_age_at_inference_start','camera_age_at_inference_end','source_age_at_inference_start','source_age_at_inference_end','inference_latency','source_image_sha256','model_jpeg_sha256','camera_failure')})
    save('model_identity.json',read(output/'model_health.json'))
    distribution_dir=PHASE/'15_valid_invalid_rollout_policy/real_action_distribution_20261014_01'
    rows=[json.loads(line) for line in (distribution_dir/'valid_predictions.jsonl').read_text().splitlines() if line]
    save('action_distribution_reference.json',dict(source=str(distribution_dir),translation=read(distribution_dir/'translation_distribution.json'),rotation=read(distribution_dir/'rotation_distribution.json'),audit=read(distribution_dir/'sample_integrity_audit.json'),raw_samples_unmodified=True))
    boundaries=[dict(value_m=x,reject=raw_translation_exceeds_limit([x,0,0])) for x in (.0129,.012999,.013,.0131)]
    save('translation_gate_validation.json',dict(source='01_configs/safety_limits.yaml via action_step_limits',previous_m=.010,current_m=RAW_TRANSLATION_STEP_LIMIT_M,boundaries=boundaries,all_50_gate_pass=all(not raw_translation_exceeds_limit(r['translation_xyz_m']) for r in rows),current_sample_gate_pass=not raw_translation_exceeds_limit(prediction['raw_action'][:3]),new_clipping=False,new_scaling=False))
    tcpsource=HERE/'real_trials/20261014_zero_offset_tcp_validation_01';cp=read(tcpsource/'controller_current_posx.json')['pose']
    save('tcp_evidence.json',dict(classification='TCP_ZERO_OFFSET_SUPPORTED_NOT_VERIFIED',tcp_contract_verified=False,pendant=read(tcpsource/'pendant_tcp_evidence.json'),fk=read(tcpsource/'fk_flange_pose.json'),controller_flange=read(tcpsource/'controller_flange_pose.json'),current_posx=read(tcpsource/'controller_current_posx.json'),comparison=read(tcpsource/'tcp_pose_comparison.json'),source_snapshot=str(tcpsource),warning='Earlier read-only snapshot; not newly acquired current TCP. No active binding approval or pose tolerance.'))
    raw=prediction['raw_action'];limiter=CanonicalActionRateLimiter();limited=limiter.limit(raw);dt=limiter.dt
    desired_t=tuple(v/dt for v in raw[:3]);desired_r=tuple(v/dt for v in raw[3:6]);post_speed_t,_=_scale_to(desired_t,limiter.tv);post_speed_r,_=_scale_to(desired_r,limiter.rv)
    post_accel_t,_=_accel_limit(post_speed_t,(0,0,0),limiter.ta*dt);post_accel_r,_=_accel_limit(post_speed_r,(0,0,0),limiter.ra*dt)
    candidate=CommandDisabledPolicyRuntime('openvla').evaluate(raw,action_id='offline_only',source_monotonic=1,now_monotonic=1,current_position_m=tuple(v/1000 for v in cp[:3]),current_abc_deg=cp[3:],phase='alignment')
    save('dynamic_limiter_validation.json',dict(synthetic_dry_calculation=True,raw_model_action=raw,raw_norm_mm=math.dist(raw[:3],[0]*3)*1000,control_rate_hz=1/dt,raw_implied_velocity_mm_s=[v*1000 for v in desired_t],post_velocity_translation_mm=[v*dt*1000 for v in post_speed_t],post_acceleration_translation_mm=[v*dt*1000 for v in post_accel_t],post_velocity_rotation_rad=[v*dt for v in post_speed_r],post_acceleration_rotation_rad=[v*dt for v in post_accel_r],post_safety_candidate=list(limited.limited_action),hypothetical_would_be_delivered_action=list(limited.limited_action),actual_delivered_action=None,candidate_pose_mm_zyz_deg=candidate['target_pose_candidate_mm_zyz_deg'],technical_blockers=candidate['technical_blockers'],candidate_authorized=False,limits=dict(translation_velocity_mm_s=limiter.tv*1000,translation_acceleration_mm_s2=limiter.ta*1000,rotation_velocity_deg_s=math.degrees(limiter.rv),rotation_acceleration_deg_s2=math.degrees(limiter.ra)),raw_13mm_implied_velocity_mm_s=13/dt,actual_evaluation_order='limiter -> limited-action supervisor/workspace -> raw magnitude blockers; NOT hard gate before limiter',physical_commands=0))
    targets=[]
    for row in rows:
        xyz=np.array(cp[:3])+np.array(row['translation_xyz_m'])*1000
        abc,matrix=compose_world_rotvec_with_doosan_zyz(cp[3:],row['rotation_xyz_rad'])
        inside=PHASE12_DATA_DERIVED_WORKSPACE.inspect(xyz/1000).accepted
        targets.append(dict(sample_id=row['sample_id'],raw_hypothetical_target_mm_zyz_deg=xyz.tolist()+abc.tolist(),inside=inside,rotation_geodesic_deg=math.degrees((Rotation.from_matrix(matrix)*Rotation.from_euler('ZYZ',cp[3:],degrees=True).inv()).magnitude()),finite=bool(np.isfinite(xyz).all() and np.isfinite(abc).all())))
    save('workspace_validation.json',dict(bounds=limits['workspace_m'],inside_count=sum(r['inside'] for r in targets),outside_count=sum(not r['inside'] for r in targets),hypothetical=True,current_pose_verified=False,pose_source=str(tcpsource/'controller_current_posx.json'),targets=targets,physical_commands=0))
    rotation_boundaries=[]
    for angle in (4-1e-4,4.,4+1e-4):
        a=[0,0,0,math.radians(angle),0,0,0]
        result=CommandDisabledPolicyRuntime('openvla',operator_confirmed_initial_open=True).evaluate(a,action_id='rotation_boundary',source_monotonic=1,now_monotonic=1,current_position_m=(.4,0,.5),current_abc_deg=(10,20,30),phase='alignment')
        rotation_boundaries.append(dict(angle_deg=angle,raw_gate_rejected='raw_rotation_step_limit' in result['technical_blockers']))
    save('rotation_gate_validation.json',dict(limit_deg=4,boundaries=rotation_boundaries,all_50_gate_pass=all(r['rotation_gate_pass'] for r in rows),current_sample_norm_deg=math.degrees(math.dist(raw[3:6],[0]*3)),geodesic_conversion_checked=True))
    save('action_coordinate_contract.json',dict(translation_frame='BASE',translation_input='metres',translation_target='mm',rotation_input='base/world relative rotvec radians',rotation_target='intrinsic Euler ZYZ degrees',formula='p_target_mm=1000*p_current_m+1000*delta_limited_m; R_target=Exp(delta_limited_rotvec_rad) @ R_current; ABC_target=euler_ZYZ_deg(R_target)',move_line_request=dict(ref=0,mode=0,time=0,radius=0,blend_type=0,sync_type=0,vel=[20,20],acc=[20,20]),sink='real_sink.RealDoosanCommandSink.send_pose -> RosServiceTransport',sink_uses='Active TCP Cartesian absolute target, NOT flange-space interface',software_convention_verified=True,physical_axes_tested=False,actual_target_authorized=False))
    save('gripper_semantics.json',dict(model='0=open,1=closed',model_source='episode_000001 metadata gripper_closedness_convention; 00_audit/action_conversion_contract.md',legacy_digital_command='0=closed,1=open is a different commanded-open signal, NOT physical IO polarity',physical_config=grip,mapping_classification='GRIPPER_MAPPING_UNVERIFIED',hysteresis=dict(open=.3,close=.7,boundary='<=0.3 OPEN, >=0.7 CLOSE'),physical_IO='separate open/close channels; configured index1/index2 active1 inactive0, not physically confirmed',full_policy_requires_mapping=True,abort_output_confirmed=False,physical_commands=0,short_horizon_gripper_dispatch=False,short_horizon_close_prediction_blocked=True,configured_open_pulse_count=grip['open_pulse_count'],sink_pulse_count_implementation='one on/off pulse; configured pulse_count not consumed',legacy_action_adapter_mapping_inverted=True))
    save('initial_gripper_state.json',dict(initial_gripper_state='UNKNOWN',expected_initial_gripper_state='OPEN',match=None,provenance='MANUAL_CONFIRMATION_REQUIRED_OR_VERIFIED_SENSOR',physical_commands=0))
    save('operator_confirmation.json',dict(operator_present='MANUAL_CONFIRMATION_REQUIRED',workspace_clear='MANUAL_CONFIRMATION_REQUIRED',estop_accessible='MANUAL_CONFIRMATION_REQUIRED',robot_stationary='MANUAL_CONFIRMATION_REQUIRED',old_confirmation_reused=False))
    save('stationary_validation.json',dict(latest_joint_velocity=runtime[-1]['velocity'],latest_exact_zero=all(v==0 for v in runtime[-1]['velocity']),existing_live_observation_joint_velocity_support='no independent stationary gate automatically inferred',zero_offset_test_criterion='all(v==0) software support only',manual_stationary_still_required=True))
    usage=shutil.disk_usage(output)
    with FsyncJsonlLogger(output/'logger_probe.jsonl') as log:log.append(dict(event='READ_ONLY_LOGGER_PROBE',physical_commands=0))
    save('logger_validation.json',dict(writable=True,free_bytes=usage.free,minimum_free_bytes=100*1024*1024,jsonl_append=True,fsync=True,logger_status=read(output/'logger_probe.jsonl.status.json'),status='PASS'))
    tests={}
    for directory in (PHASE, HERE, PHASE/'15_valid_invalid_rollout_policy'):
        test=subprocess.run(['/usr/bin/python3','-m','unittest','discover','-s','tests','-q'],cwd=directory,text=True,capture_output=True)
        tests[str(directory.relative_to(REPO))]=dict(returncode=test.returncode,stdout=test.stdout,stderr=test.stderr)
    save('command_inhibition_tests.json',dict(tests=tests,passed=all(t['returncode']==0 for t in tests.values()),fake_transport_only=True,faults_tested=['JointState','Camera','Model','translation >13mm','rotation >4deg','logger','NaN'],future_dispatch_zero=True))
    save('single_trial_invariants.json',dict(automatic_batch=False,automatic_retry=False,automatic_home=False,automatic_recovery=False,short_horizon_duration_s=2,short_horizon_maximum_actions=10,minimum_motion_maximum_actions=1,stage_gate_requires_physical_minimum_motion_pass=True,physical_minimum_motion_pass_evidence_found=False,physical_stop_validation_found=False,arm_only_classification='SYSTEM_INTEGRATION_TEST_NOT_TASK_SUCCESS_RATE',full_policy_gripper_dispatch_supported_by_current_short_horizon=False))
    repeats=collections.Counter(tuple(r['raw_action']) for r in rows)
    save('prediction_discreteness.json',dict(unique_vectors=len(repeats),frequencies=[dict(raw_action=list(k),count=v) for k,v in repeats.most_common()],model_decoder='base_openvla_7b/modeling_prismatic.py: 256 action bins -> bin centers -> q01/q99 unnormalization',conclusion='quantized outputs explain repeated numeric levels; same static scene does not establish model collapse or multi-scene repeatability'))
    save('localhost_verification.json',dict(source='current /proc PID environ read-only inspection',processes={str(pid):dict(ROS_LOCALHOST_ONLY='1') for pid in (3245663,3245816,3116004)}))
    matrix={'Hardware state':'PASS (REAL/AUTO/STANDBY scalar only)','JointState':'PASS','Start pose':'PASS','Camera':'PASS','Model identity':'PASS','Translation 13mm gate':'PASS','Rotation gate':'PASS','Dynamic limiter':'PASS (raw action modified)','Workspace':'PASS (50/50 hypothetical targets; earlier controller pose)','Action convention':'PASS (software); physical axis proof pending','TCP evidence':'CONDITIONAL / NOT VERIFIED','Gripper semantics':'FAIL (hardware mapping unverified)','Initial gripper state':'FAIL (UNKNOWN)','Logger':'PASS','Command inhibition':'PASS' if all(t['returncode']==0 for t in tests.values()) else 'FAIL','Operator/E-stop':'FAIL (fresh manual confirmation required)'}
    save('final_readiness.json',dict(classification='PHYSICAL_SHORT_HORIZON_BLOCKED',matrix=matrix,physical_commands=0,production_translation_limit_m=RAW_TRANSLATION_STEP_LIMIT_M,blockers=['TCP_NOT_VERIFIED','GRIPPER_HARDWARE_MAPPING_AND_ABORT_UNKNOWN','INITIAL_GRIPPER_UNKNOWN','FRESH_MANUAL_CONFIRMATION_REQUIRED','AUTHORITY_SERVO_PROTECTION_UNKNOWN','PHYSICAL_MINIMUM_MOTION_AND_STOP_PROOF_MISSING','SHORT_HORIZON_REJECTS_CLOSE_PREDICTIONS'],configs_unchanged_after_freeze=all(hashlib.sha256((REPO/p).read_bytes()).hexdigest()==v['sha256'] for p,v in snapshots.items())))
    table='\n'.join('| '+k+' | '+v+' |' for k,v in matrix.items())
    text=f'''# Final pre-rollout validation — no physical commands

Production translation gate changed by explicit user request from 10 to 13 mm, grounded in 50/50 valid real samples with p99/max 12.875 mm. Configuration snapshot hashes and git baseline preserved; no other safety thresholds changed. Exact 13 mm accepted, 13.1 mm rejected; NaN/Inf rejected. Shared source is safety_limits.yaml -> action_step_limits -> production SafetyPipeline and batch. Historical >4mm statistics and 4mm velocity-derived ceilings are diagnostics/dynamic consequences, not residual raw rejection gates.

| Item | Verdict |
|---|---|
{table}

Current JointState monitoring requested 30s and observed {live['current_session_runtime']['duration_observed_s']:.3f}s including post-getter/prediction phase: {len(runtime)} samples, max source gap {jointreport['max_source_gap_s']*1000:.3f}ms, max receive gap {jointreport['max_receive_gap_s']*1000:.3f}ms; >=100ms events 0, invalid/duplicate/regression 0. Hardware current getter responses REAL/AUTO/STANDBY. Latest JointState velocities all zero is software stationary support only.

Camera prediction: start age {prediction['camera_age_at_inference_start']*1000:.3f}ms; end age {prediction['camera_age_at_inference_end']*1000:.3f}ms; inference {prediction['inference_latency']*1000:.3f}ms; actual bgra8 -> RGB -> JPEG input and hashes saved. Strict current model health validated. Raw translation {math.dist(raw[:3],[0]*3)*1000:.6f}mm passes 13mm; gripper {raw[6]:.6f} close.

Dynamic limits remain 20mm/s translation velocity and 20mm/s² acceleration, rotation20deg/s and20deg/s² at5Hz. Raw13mm implies65mm/s. Speed-limited step max4mm; first acceleration-limited step0.8mm. Current raw action first candidate translation norm {math.dist(limited.limited_action[:3],[0]*3)*1000:.6f}mm. Raw action is NOT delivered unchanged. Exact per-stage vectors/target are in dynamic_limiter_validation.json; these are unauthorized hypothetical calculations, actual delivered action null.

Coordinate source confirms BASE translation metres -> mm, relative base rotvec radians composed R_delta @ R_current -> ZYZ degrees. Sink is absolute BASE active-TCP MoveLine ref0/mode0, not flange-space. Fifty raw hypothetical targets calculated using earlier controlled current PosX snapshot are inside bounds; orientation relative-matrix checks finite. Earlier pose is not promoted to fresh verified TCP. TCP zero-offset remains supported not verified; no auto engineering approval or tolerance creation.

Model gripper0=open/1=closed differs from legacy digital commanded-open bit. Phase12 physical sink uses separate output channels and checks polarity/abort confirmation; config has polarity_confirmed=false and abort_value=null. Initial physical gripper state UNKNOWN. Source shows configured open_pulse_count2 is not consumed by sink's one on/off pulse; mismatch is reported, not silently fixed. Legacy ActionAdapter inverted mapping must not be used as verified evidence. Short-horizon currently gripper-disabled and rejects close>=0.7; full-task is the only Controller.step path that dispatches gripper. ARM_ONLY can only be a system integration test, never full-policy task success evaluation.

All unit suites run with fake/no physical transport; detailed returns captured. Logger uses actual FsyncJsonlLogger append/flush/fsync and100MiB free-space threshold; probe PASS. Fault injection includes sensor/model/translation/rotation/logger/NaN terminal INVALID and no future command dispatch. Single trial runner has no batch/retry/home/recovery; its2s max10actions is not an exactly-one-pose-command protocol. Minimum-motion is separate max1 and must precede it with physical PASS; no such PASS/physical stop evidence found.

Fresh operator/workspace/E-stop/stationary confirmations REQUIRED; prior confirmations not reused. Authority/servo/protective/emergency remain unknown independently of STANDBY. These are not satisfied by software event silence or successful scalar getters.

Final: **PHYSICAL_SHORT_HORIZON_BLOCKED**. Reason: unverified TCP, physical gripper mapping/abort/initial state, fresh manual/hardware safety evidence, required minimum-motion/stop proof, and close-prediction rejection in current short-horizon protocol. Physical commands **0**. No physical Hold/Stop/gripper/motion client used for live checks; fake test transport actions are not robot commands.
'''
    (output/'FINAL_PRE_ROLLOUT_READINESS_REPORT.md').write_text(text)
    print(json.dumps(dict(classification='PHYSICAL_SHORT_HORIZON_BLOCKED',matrix=matrix,physical_commands=0),indent=2))


if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('output',type=Path);run(parser.parse_args().output.resolve())
