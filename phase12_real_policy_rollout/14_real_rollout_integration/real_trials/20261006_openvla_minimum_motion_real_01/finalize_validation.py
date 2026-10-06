import json,math,statistics
from pathlib import Path
ROOT=Path(__file__).resolve().parent
s=json.loads((ROOT/'live_shadow_summary.json').read_text());audit=json.loads((ROOT/'tcp_source_audit.json').read_text())
rows=[json.loads(l) for l in (ROOT/'live_shadow.jsonl').read_text().splitlines()];rows=[r for r in rows if r['event']=='PREDICTION']
maxage=max(r['camera_age_s'] for r in rows) if rows else None
camera=dict(status='FAIL_VALIDATION_ABORTED_BEFORE_30_SECONDS',observed_prediction_count=len(rows),observed_duration_s=s['shadow_duration_s'],
    camera_failure_count=s['blockers'].get('camera_failure',0),max_camera_age_s=maxage,threshold_s=.5,threshold_changed=False,
    observed_frame_freshness='PASS_WITHIN_RECORDED_INTERVAL' if maxage is not None and maxage<.5 else 'FAIL',
    abort_reason=s['watchdog_or_abort_reason'])
tcp=dict(status='CURRENT_TCP_UNKNOWN',active_tcp_name=None,verified_flange_to_tcp=None,source='FK_ESTIMATED_FLANGE',
    zero_offset_applied=False,measured_tcp_available=False,controller_evidence=None,
    explanation='Actual a0509 robot_description ends at link_6, only world/base fixed joint. Observed left_TCP/right_TCP belong to SO101 scene, not current Doosan controller tool. /doosan/current_pose has no publishers.',getters_called=0)
hardware=dict(status='UNKNOWN_FAIL_CLOSED',operator_present='UNCONFIRMED',estop_accessible='UNCONFIRMED',
    workspace_clear='UNCONFIRMED',protective_stop_clear='UNCONFIRMED',robot_mode_authority='UNCONFIRMED',motion_authorized=False)
joint=dict(pre_window=s['pre_shadow_gate'],status='FAIL_DURING_SHADOW',abort_reason=s['watchdog_or_abort_reason'])
minimum=dict(status='BLOCKED_TCP_CONTRACT_UNKNOWN',also_blocked_by=['JointState watchdog fault','Hardware/operator UNKNOWN','30s camera validation aborted'],
    physical_pose_command_count=0,gripper_command_enabled=False,gripper_pulse_enabled=False,requested_delta_m=None,
    protocol_delta_m=[.0005,0,0],observed_delta_m=None,ack=None,unexpected_motion='NOT_ASSESSED_NO_PHYSICAL_STAGE',
    short_horizon='NOT_EXECUTED',full_task='NOT_EXECUTED',model_action_used=False,motion_authorized=False)
for name,data in [('camera_final_validation.json',camera),('tcp_contract.json',tcp),('jointstate_pre_motion.json',joint),('hardware_gate.json',hardware),('minimum_motion_result.json',minimum)]:
    with (ROOT/name).open('x') as f:json.dump(data,f,indent=2)
with (ROOT/'minimum_motion.jsonl').open('x') as f:f.write(json.dumps(dict(event='NOT_EXECUTED',**minimum))+'\n')
(ROOT/'MINIMUM_MOTION_REAL_REPORT.md').write_text(f'''# Minimum-motion real precheck — blocked, no physical stage

Start HEAD ebb1665d20bff17846567f12b2afb4c9cf5a618d, branch lhj-research, initially clean. No driver restart, mode/tool change or getter call.

Camera recorded {len(rows)} real GPU predictions in {s['shadow_duration_s']:.3f}s. camera_failure=0, observed maximum selected age {maxage:.6f}s (<0.5). However JointState watchdog fault stopped the run before the mandatory30s; camera final-validation FAIL/INCOMPLETE, not promoted to motion approval. Existing fast BGRA→RGB/JPEG and health-before-new-frame path unchanged. Original logs preserved.

JointState pre20s gate PASS (2000 samples99.998Hz,maxsource11.42ms/maxreceive16.15ms). During Shadow, recent continuity failed: captured late source gaps0.1900s and0.1300s and receive gaps0.1912s/0.1320s. Initial3.06s events are separately retained, not confused with this late recent-window failure. Independent watchdog stopped predictions; no physical Hold was invoked because diagnostic sink has no command capability.

TCP automatic inventory: actual /dsr01/robot_description robot=a0509 has base_link,link_1…link_6,world and only world_fixed static joint. No Doosan flange→tool/TCP transform verified. Source a0509.urdf tool0 block is commented out. /doosan/current_pose has no publisher. left_TCP/right_TCP and so101 gripper frames are OTHER robot/scene frames; never reused as A0509 controller TCP. TF source/header/receive times, publisher endpoints, all frame transforms and topic/service types saved in tcp_source_audit.json. TF alone cannot prove controller active tool. Historical Tool_v1/offset0 is not current evidence. Current classification CURRENT_TCP_UNKNOWN.

Hardware/operator confirmation not received: workspace clear,local operator,E-stop access,protective stop,mode/authority UNKNOWN. These were not inferred from the user's request. Gripper physical commands/pulses DISABLED. No setter or query of unstable service performed.

Final BLOCKED_TCP_CONTRACT_UNKNOWN with additional JointState/hardware/camera-completion blocks. Minimum motion NOT_EXECUTED; requested target/delta=null (protocol reference only +X0.5mm), observed delta=null, ACK=null, physical direction/displacement not assessed. No API-success-only PASS artifact created. Short-horizon/full-task NOT_EXECUTED. All physical robot/motion/gripper/Home/trajectory/Hold/Stop/real-rollout calls0. Safety limits unchanged.

Required next evidence: current active TCP/tool name, current flange→TCP6D offset with units/convention/reference, pendant current pose/base frame/time/stationary confirmation, plus current local safety confirmations. Read-only fresh validation must pass again before any separate minimum-motion approval. No software auto-recovery or root-cause expansion undertaken.
''')
print(json.dumps(dict(camera=camera,tcp=tcp['status'],joint=joint['status'],minimum=minimum['status'],physical_pose_commands=0),indent=2))
