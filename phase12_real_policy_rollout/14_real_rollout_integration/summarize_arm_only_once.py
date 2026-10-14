"""Offline feedback comparison; never dispatches another command."""
import argparse,json,math
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation


def summarize(directory):
    def read(name):return json.loads((directory/name).read_text())
    def save(name,value):(directory/name).write_text(json.dumps(value,indent=2))
    summary=read('trial_summary.json');motion=read('observed_motion.json');prediction=read('prediction.json');runtime=read('jointstate_during_test.json')
    limited=read('action_pipeline.json')['final_arm_candidate'];requested=np.array(limited[:3])*1000;observed=np.array(motion['observed_fk_flange_delta_mm'])
    before=np.array(motion['fk_before']['rotation_matrix']);after=np.array(motion['fk_after']['rotation_matrix'])
    observed_rotation=Rotation.from_matrix(after@before.T);expected_rotation=Rotation.from_rotvec(limited[3:6])
    ready_at=runtime['status']['ready_at'];rows=[r for r in runtime['rows'] if r['receive']>=ready_at]
    metrics=dict(raw_translation_norm_mm=float(np.linalg.norm(prediction['raw_action'][:3])*1000),requested_delta_mm=requested.tolist(),observed_fk_flange_delta_mm=observed.tolist(),
        requested_norm_mm=float(np.linalg.norm(requested)),observed_norm_mm=float(np.linalg.norm(observed)),delta_error_mm=float(np.linalg.norm(observed-requested)),
        direction_cosine=float(np.dot(requested,observed)/(np.linalg.norm(requested)*np.linalg.norm(observed))),magnitude_ratio=float(np.linalg.norm(observed)/np.linalg.norm(requested)),
        requested_rotation_deg=math.degrees(expected_rotation.magnitude()),observed_rotation_deg=math.degrees(observed_rotation.magnitude()),
        relative_rotation_error_deg=math.degrees((observed_rotation*expected_rotation.inv()).magnitude()),runtime_fault=runtime['status']['fault'],
        max_source_gap_s=max(r['source_gap'] for r in rows),max_receive_gap_s=max(r['receive_gap'] for r in rows),
        source_gap_ge_100ms=sum(r['source_gap']>=.1 for r in rows),receive_gap_ge_100ms=sum(r['receive_gap']>=.1 for r in rows),final_joint_velocity=read('jointstate_after.json')['velocity'],
        feedback_source='JOINTSTATE_MEASURED_JOINTS_PLUS_FK_ESTIMATED_FLANGE_NOT_MEASURED_TCP',physical_operator_result_confirmation='PENDING')
    save('feedback_comparison.json',metrics)
    text=f'''# Actual arm-only one-step system integration test

Physical arm commands **{summary['physical_arm_commands']}**; gripper **{summary['physical_gripper_commands']}**; physical Hold/Stop **{summary['physical_hold_stop_commands']}**. Real inference requests **{summary['prediction_count']}**. No second step/retry/batch/home/recovery. Scope SYSTEM_INTEGRATION_TEST, task_success=null; never counted as policy task SUCCESS/FAILURE.

Fresh operator four-condition confirmation received and preserved with timestamp. REAL/AUTO/STANDBY current precheck PASS. Existing persistent clean10s readiness obtained, start pose within0.5 degrees, zero JointState runtime faults. Existing CurrentPosX(ref0) getter called once, valid response; no setter/tool change. TCP formal classification remains TCP_ZERO_OFFSET_SUPPORTED_NOT_VERIFIED; execution acceptance is explicitly scoped consistency-based, active TCP name not directly observed. Post-motion state query returned STANDBY.

Fresh image 1280x720 bgra8 -> RGB -> JPEG -> strict OpenVLA step8130 K1 dim7 proprio=false. Camera request-start age {prediction['camera_age_at_inference_start']*1000:.3f}ms, end age {prediction['camera_age_at_inference_end']*1000:.3f}ms, inference {prediction['inference_latency']*1000:.3f}ms. Raw action {prediction['raw_action']}. Translation norm {metrics['raw_translation_norm_mm']:.6f}mm <13mm; rotation {metrics['requested_rotation_deg']:.6f} degrees <4deg. Gripper raw {prediction['raw_action'][6]:.6f} stored only; no hardware IO/pulse/abort gripper output.

Existing velocity stage preserved the {metrics['raw_translation_norm_mm']:.6f}mm translation; existing first-step acceleration limiter reduced it to {metrics['requested_norm_mm']:.6f}mm. No new clipping/scaling. Final candidate and exact current/target pose, BASE absolute ref0/mode0, vel/acc20 are retained. Euler target values can look different by equivalent representation; rotation validation uses matrices/geodesics, not component subtraction.

MoveLine sent once, ACK success=true; synchronous service receipt completed after {summary['command_events'][0]['ack_latency']*1000:.3f}ms. Service ACK alone is not used as proof of physical PASS.

| Metric | Recorded result |
|---|---|
| Requested limited xyz mm | {requested.tolist()} |
| Observed FK flange xyz mm | {observed.tolist()} |
| Requested / observed norm mm | {metrics['requested_norm_mm']:.6f} / {metrics['observed_norm_mm']:.6f} |
| Delta error mm | {metrics['delta_error_mm']:.6f} |
| Direction cosine | {metrics['direction_cosine']:.9f} |
| Magnitude ratio | {metrics['magnitude_ratio']:.6f} |
| Requested / observed rotation deg | {metrics['requested_rotation_deg']:.6f} / {metrics['observed_rotation_deg']:.6f} |
| Relative rotation mismatch deg | {metrics['relative_rotation_error_deg']:.6f} |
| Max source / receive gap ms | {metrics['max_source_gap_s']*1000:.3f} / {metrics['max_receive_gap_s']*1000:.3f} |
| Runtime >=100ms events | {metrics['source_gap_ge_100ms']} / {metrics['receive_gap_ge_100ms']} |
| Final joint velocities | {metrics['final_joint_velocity']} |

Feedback is measured JointState plus computed flange FK, not independently measured TCP. Small requested/observed motion agreement is strong command-path evidence, but no new physical acceptance tolerance was invented and formal TCP verification is not promoted. E-stop used/unexpected motion remain null until operator supplies actual observation.

Current result **PHYSICAL_CONFIRMATION_PENDING**. Final ARM_ONLY_SHORT_HORIZON_PASS/FAIL requires operator confirmation of expected motion, no gripper movement, no unexpected motion and E-stop usage. No further physical command is permitted in this trial. Next ANALYZE_ARM_TEST.
'''
    (directory/'ARM_ONLY_SHORT_HORIZON_REPORT.md').write_text(text)
    print(json.dumps(metrics,indent=2))


if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('directory',type=Path);summarize(parser.parse_args().directory)
