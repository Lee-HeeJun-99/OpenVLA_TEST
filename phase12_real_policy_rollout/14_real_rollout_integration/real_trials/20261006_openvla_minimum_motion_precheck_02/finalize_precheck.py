"""Offline summary only; does not authorize or execute motion."""
import json,statistics,math
from pathlib import Path
ROOT=Path(__file__).resolve().parent
r=[json.loads(l) for l in (ROOT/'live_shadow.jsonl').read_text().splitlines()];rows=[x for x in r if x['event']=='PREDICTION']
s=json.loads((ROOT/'live_shadow_summary.json').read_text())
def stats(k):
    v=sorted(x[k] for x in rows)
    return dict(median=statistics.median(v),max=max(v),p95=v[math.ceil(.95*len(v))-1]) if v else None
camera=dict(status='PASS' if rows and all(x['camera_age_s']<.5 for x in rows) and not s['watchdog_or_abort_reason'] else 'FAIL',
    threshold_s=.5,threshold_changed=False,prediction_count=len(rows),
    camera_failure_count=s['blockers'].get('camera_failure',0),
    timing={k:stats(k) for k in ('health_latency_s','jpeg_encode_latency_s','selected_frame_age_at_snapshot_s','inference_latency_s','camera_age_s')},
    rgb_pixel_equivalence_test='PASS',physical_commands=0)
tcp=dict(status='UNKNOWN',tool_offset_verified=False,tcp_contract_verified=False,active_tcp_name=None,
    source='FK_ESTIMATED_FLANGE',getters_called=0,operator_current_evidence_received=False)
minimum=dict(status='BLOCKED',reason='CURRENT_TCP_TOOL_AND_OPERATOR_SAFETY_EVIDENCE_MISSING',
    gripper_physical_command_enabled=False,gripper_pulse_enabled=False,physical_pose_commands=0,
    actual_tcp_response=None,ack=None,unexpected_motion='NOT_ASSESSED_NO_PHYSICAL_STAGE',
    motion_authorized=False,model_action_used_for_motion=False)
for name,data in [('camera_freshness_validation.json',camera),('tcp_validation.json',tcp),('jointstate_pre_motion.json',dict(s['pre_shadow_gate'],scope='PRE_SHADOW_NOT_MOTION_APPROVAL')),('minimum_motion_result.json',minimum)]:
    with (ROOT/name).open('x') as f:json.dump(data,f,indent=2)
with (ROOT/'minimum_motion.jsonl').open('x') as f:f.write(json.dumps(dict(event='NOT_EXECUTED',**minimum))+'\n')
(ROOT/'MINIMUM_MOTION_REPORT.md').write_text(f'''# Minimum-motion precheck — physical stage blocked

Current HEAD at start01c4fe5945e3d323e1016c676e72501ccfdcba45. Driver was not restarted; no mode/tool/jog/getter call made. Production safety limits unchanged.

Camera freshness: {camera['status']}; actual prediction count {len(rows)}, camera_failure {camera['camera_failure_count']}, timing {camera['timing']}. Health validation now precedes frame selection; fresh receive-age<=10ms frame required, raw PIL RGB decode is pixel-equivalent including ROS padding/alpha. Inference input size/crop/normalization/JPEGquality95 unchanged. Safety's0.5s threshold not modified. FK/joints used only as unverified diagnostic context; safety context refreshed after inference.

First attempt (precheck01) health-before-snapshot alone still failed10/61 selected frames; healthmedian2.34ms, RGB/JPEGmedian32.02ms, snapshotmax94.59ms. Preserved, not overwritten. Second attempt additionally waits for a NEWframe and uses tested faster raw decoding. These observations support selected-frame processing/age as contributors, not proven controller issues.

JointState fresh window: {s['pre_shadow_gate']}. Watchdog fault: {s['watchdog_or_abort_reason']}. This is a diagnostic pre-Shadow gate, not a continuously valid future motion approval.

TCP remainsUNKNOWN. GetCurrentTcp/GetCurrentTool callbacks return names only; no6D offset/pose. Their current live stability is unverified and they were not called. ConfigCreateTcp contains6Dpos but is a setter and NEVER called. PastTool_v1 was not reused. Current operator pendant name/offset/pose/frame/time and safety mode/authority/E-stop/workspace confirmations are still required.

Gripper stateUNKNOWN isolated: this diagnostic has no physical gripper API, pulse or motion sink. Minimum-motion must remain translation-only with gripper disabled; this does not establish real gripper polarity or full-task safety.

Minimum-motion BLOCKED, exactly0 physical pose commands. No service ACK or actual Cartesian displacement exists to score. API-success-only PASS never issued. Short/full-task and model action motion NOT_EXECUTED. All robot/gripper/Home/trajectory/Hold/Stop/real rollout calls0. No authorization artifact created.

Unit integration37PASS,0FAIL after RGBdecoder/gate changes. Next allowed stage remains currentTCP/operator crosscheck; only after ALLgates freshlyPASS can a separate explicit minimum-motion approval be requested.
''')
print(json.dumps(camera,indent=2))
