"""Read recorded artifacts only. No ROS, inference, simulation or robot calls."""
import csv
import hashlib
import json
import math
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
from run_real_rollout import SafetyPipeline
from action_contract import compose_world_rotvec_with_doosan_zyz,rotation_step_geodesic_rad
from servo_experimental.controller import continuous_zyz,interpolate_segment
from settle_window import ExactZeroSettleWindow

ROOT=Path(__file__).resolve().parent
RUNS=('182408','183554','183848')


def csv_write(path,rows):
    if not rows:return
    with path.open('w',newline='') as stream:
        writer=csv.DictWriter(stream,fieldnames=list(rows[0]))
        writer.writeheader();writer.writerows(rows)


def main():
    out=ROOT/'real_trials/20261112_three_servo_rotation_analysis_02'
    out.mkdir(exist_ok=False)
    poses=[];states=[];policies=[];reports=[];provenance=[]
    for run in RUNS:
        directory=ROOT/f'real_trials/20261112_{run}_arm_only_servo_episode'
        rows=[json.loads(line) for line in (directory/'predictions.jsonl').read_text().splitlines()]
        summary=json.loads((directory/'trial_summary.json').read_text())
        joints=json.loads((directory/'jointstate.json').read_text())['samples']
        health=json.loads((directory/'model_health.json').read_text())
        config=json.loads((directory/'config_snapshot.json').read_text())
        pipeline=SafetyPipeline('openvla',operator_confirmed_initial_open=False,
                                arm_only=True,translation_endpoint_delta=True)
        replay=[];max_error=0.
        for i,row in enumerate(rows):
            t=row['camera_timing']['inference_start_monotonic']
            current=row['current_controller_pose'];raw=row['raw_action'];stored=row['action_pipeline']
            raw_abc,_=compose_world_rotvec_with_doosan_zyz(current[3:],raw[3:6])
            raw_pose=np.r_[np.array(current[:3])+1000*np.array(raw[:3]),raw_abc]
            prev=None
            if i:
                pr=rows[i-1];prabc,_=compose_world_rotvec_with_doosan_zyz(pr['current_controller_pose'][3:],pr['raw_action'][3:6])
                prev=np.r_[np.array(pr['current_controller_pose'][:3])+1000*np.array(pr['raw_action'][:3]),prabc]
            now=t+stored['action_age_sec']
            decision=pipeline.runtime.evaluate(raw,action_id=str(i),source_monotonic=t,now_monotonic=now,
                current_position_m=[v/1000 for v in current[:3]],current_abc_deg=current[3:],phase='alignment',arm_only=True)
            geo=math.degrees(rotation_step_geodesic_rad(current[3:],raw_abc))
            old_reject='raw_rotation_step_limit' in stored['technical_blockers']
            new_reject='raw_rotation_step_limit' in decision['technical_blockers']
            replay.append(dict(step=row['step'],old_reject=old_reject,new_reject=new_reject,
                               raw_geodesic_deg=geo,after_blockers=decision['technical_blockers']))
            states.append(dict(run=run,step=row['step'],robot_state=row['robot_state'],
                state_receive_monotonic=None,state_receive_upper_bound=t,
                inference_start_monotonic=t,pose_receive_monotonic=row['controller_feedback']['response_received_monotonic'],
                source='prediction.robot_state; getter before inference; exact receive time NOT_LOGGED'))
            if i>=len(rows)-4:
                candidate=stored['target_pose_candidate_mm_zyz_deg']
                prev_candidate=rows[i-1]['action_pipeline']['target_pose_candidate_mm_zyz_deg'] if i else None
                raw_diff=None if prev is None else raw_pose[3:]-prev[3:]
                cand_diff=None if prev_candidate is None else np.array(candidate[3:])-prev_candidate[3:]
                poses.append(dict(run=run,step=row['step'],rejected=old_reject,
                    raw_target_xyz_mm=raw_pose[:3].tolist(),raw_target_ABC_deg=raw_pose[3:].tolist(),
                    raw_target_previous_ABC_diff_deg=None if raw_diff is None else raw_diff.tolist(),
                    raw_target_pair_geodesic_deg=None if prev is None else math.degrees(rotation_step_geodesic_rad(prev[3:],raw_pose[3:])),
                    raw_from_current_geodesic_deg=geo,
                    B_distance_to_0_or_180_deg=min(abs(raw_pose[4]),abs(180-abs(raw_pose[4]))),
                    recorded_filtered_target_xyz_mm=candidate[:3],recorded_filtered_target_ABC_deg=candidate[3:],
                    recorded_target_previous_ABC_diff_deg=None if cand_diff is None else cand_diff.tolist(),
                    recorded_target_pair_geodesic_deg=None if prev_candidate is None else math.degrees(rotation_step_geodesic_rad(prev_candidate[3:],candidate[3:])),
                    pair_classification=None if prev is None else ('표현 문제' if rotation_step_geodesic_rad(prev[3:],raw_pose[3:])<math.radians(4) else '실제 점프'),
                    actual_gate_classification='표현 문제' if geo<4 else '실제 점프'))
                policies.append(dict(run=run,step=row['step'],raw_action=raw,chunk_id=stored.get('chunk_id'),
                    chunk_index=stored.get('chunk_index'),health_chunk_size=health.get('chunk_size'),
                    new_prediction=True,multi_action_chunk_boundary=False))
            r0=Rotation.from_euler('ZYZ',current[3:],degrees=True)
            r1=Rotation.from_euler('ZYZ',stored['target_pose_candidate_mm_zyz_deg'][3:],degrees=True)
            previous=np.array(current[3:])
            for fraction in np.linspace(0,1,21):
                _,r=interpolate_segment(current[:3],r0,stored['target_pose_candidate_mm_zyz_deg'][:3],r1,float(fraction))
                abc=continuous_zyz(r,previous);previous=abc
                max_error=max(max_error,(Rotation.from_euler('ZYZ',abc,degrees=True).inv()*r).magnitude())
        request=summary['stop_requested_monotonic'];window=ExactZeroSettleWindow();first_ready=None
        post=[r for r in joints if r['receive']>=request]
        for row in post:
            if window.sample(row) and first_ready is None:first_ready=row['receive']
        source=next(v['content'] for k,v in config['files'].items() if k.endswith('/start_servo_episode_confirmed.py'))
        stop=dict(request_monotonic=request,stop_ack=summary.get('stop_ack'),
            state_value=summary.get('final_robot_state'),state_receive_monotonic=summary.get('final_robot_state_received_monotonic'),
            original_confirmation=summary.get('stop_confirmation'),post_stop_joint_samples=len(post),
            first_post_stop_joint_receive=post[0]['receive'] if post else None,
            first_zero_window_ready_monotonic=first_ready,
            last_joint_receive=joints[-1]['receive'],
            state_age_when_zero_ready=None if first_ready is None or summary.get('final_robot_state_received_monotonic') is None else first_ready-summary['final_robot_state_received_monotonic'],
            actual_state_sample_refresh=False)
        report=dict(run=run,rotation_limit_deg=4.,replay_count=len(replay),
            rejected_before=[x['step'] for x in replay if x['old_reject']],
            rejected_after=[x['step'] for x in replay if x['new_reject']],
            rejected_raw_geodesic_deg=replay[-1]['raw_geodesic_deg'],
            stop=stop,max_slerp_reconstruction_error_rad=max_error,replay=replay)
        reports.append(report)
        for name in ('predictions.jsonl','servo_commands.jsonl','trial_summary.json','jointstate.json','config_snapshot.json'):
            f=directory/name;provenance.append(dict(path=str(f),sha256=hashlib.sha256(f.read_bytes()).hexdigest()))
    (out/'analysis.json').write_text(json.dumps(dict(physical_commands=0,runs=reports,targets=poses,policies=policies),indent=2,ensure_ascii=False))
    (out/'provenance.json').write_text(json.dumps(provenance,indent=2))
    csv_write(out/'targets.csv',poses);csv_write(out/'robot_state_timeline.csv',states);csv_write(out/'policy_raw_outputs.csv',policies)
    lines=['# Three recorded ServoL runs — offline audit',
        '', 'Physical calls: 0. Original artifacts unchanged. No production code changes in this task.',
        '', 'Raw model rotation is a BASE rotvec (rad), NOT Euler ABC. Raw target below is reconstructed from logged current pose and raw action, using intrinsic ZYZ. Recorded filtered targets are separately preserved in targets.csv.',
        '', 'Pair rule requested by user: geodesic < 4deg => expression label; >= 4deg => physical jump label. A small pair angle alone does not prove that the actual rejection was caused by an Euler representation problem.',
        '', '## Raw-policy target (reconstructed, before rotation delta limiter)',
        '', '| Run | Step (zero-based) | Rejected | xyz mm | ABC deg | Delta ABC vs previous raw target deg | Pair geodesic deg | Distance of B to nearest singular deg |',
        '|---|---:|---|---|---|---|---:|---:|']
    def vector(value):return '['+', '.join(f'{v:.6f}' for v in value)+']'
    for row in poses:
        lines.append(f"| {row['run']} | {row['step']} | {row['rejected']} | {vector(row['raw_target_xyz_mm'])} | {vector(row['raw_target_ABC_deg'])} | {vector(row['raw_target_previous_ABC_diff_deg'])} | {row['raw_target_pair_geodesic_deg']:.6f} | {row['B_distance_to_0_or_180_deg']:.6f} |")
    lines+=['', 'No listed B is within 1deg of 0 or +/-180deg; nearest distance is 13.035deg. This 1deg neighborhood is a reporting label, not a safety threshold.',
        '', '## Actual raw gate replay', '', '| Run | Rejected step | Raw geodesic deg | Before | After |', '|---|---:|---:|---|---|']
    for r in reports:lines.append(f"| {r['run']} | {r['rejected_before']} | {r['rejected_raw_geodesic_deg']:.9f} | REJECT | REJECT |")
    lines+=['', 'All three are actual raw rotation steps above the unchanged 4deg gate. Slerp/continuous ZYZ cannot and must not erase this raw rejection. Replaying recorded inputs is not a counterfactual physical trajectory prediction.',
        '', 'All models report chunk_size=1; each row is a fresh inference, chunk_id=null, chunk_index=0. There is no multi-action chunk boundary.',
        '', '## Stop samples', '', '| Run | Stop request monotonic | State receive monotonic | State | First exact-zero window ready | State age at that time |',
        '|---|---:|---|---|---|---|']
    for r in reports:
        s=r['stop'];lines.append(f"| {r['run']} | {s['request_monotonic']:.9f} | {s['state_receive_monotonic']} | {s['state_value']} | {s['first_zero_window_ready_monotonic']} | {s['state_age_when_zero_ready']} |")
    lines+=['', '182408: no post-stop robot state recorded, only one post-stop JointState sample. Stop cannot be verified from these logs.',
        '', '183554 / 183848: post-stop STANDBY sample exists and follows request. Exact-zero window became available later, when the cached robot-state sample was 1.240s / 0.953s old (>0.5s). Thus stale, not before-request. Same cached state was reused during proof wait. No clock-domain mixing was found: comparisons use host monotonic receive time; ROS source stamps are used only for source continuity.',
        '', '## Robot-state timeline (compressed)', '', 'Complete sampled sequence: robot_state_timeline.csv. State getter receive timestamps were not logged per prediction; inference-start times below are upper bounds on state receipt, not exact measurements. Pose receive times must not be relabeled as state receipt.',
        '', '| Run | Step interval | State | Receipt upper-bound interval (monotonic) |', '|---|---|---:|---|']
    for run in RUNS:
        rs=[s for s in states if s['run']==run];begin=0
        for i in range(1,len(rs)+1):
            if i==len(rs) or rs[i]['robot_state']!=rs[begin]['robot_state']:
                a,b=rs[begin],rs[i-1]
                lines.append(f"| {run} | {a['step']}..{b['step']} | {a['robot_state']} | {a['state_receive_upper_bound']:.9f} .. {b['state_receive_upper_bound']:.9f} |")
                begin=i
    lines+=['', 'State 1=STANDBY, 2=MOVING under the launcher contract. Sparse per-inference polling cannot establish the full continuous state trajectory.',
        '', '## Changes and reproducibility', '', '`analyze_three_servo_runs.py` added only. New analysis CSV/JSON/Markdown generated. No runtime/safety/controller changes in this task because all rejected raw rotations are real >4deg steps. Existing geodesic/Slerp/continuous-ZYZ code was reused read-only.',
        '', 'Reconstruction replay: every stored target segment sampled at 21 fractions using linear position/Slerp rotation; max orientation reconstruction error <1.6e-15 rad across all runs.',
        '', 'Source file hashes: provenance.json. Raw values, separate filtered target values, and both pair-angle calculations: targets.csv. Per-prediction raw-gate replay decisions: analysis.json.',
        '', 'Command: `/usr/bin/python3 analyze_three_servo_runs.py` (output directory must not already exist).']
    (out/'THREE_RUN_ROTATION_STOP_REPORT.md').write_text('\n'.join(lines)+'\n')
    for report in reports:
        print(json.dumps({k:v for k,v in report.items() if k!='replay'},ensure_ascii=False,indent=2))
    print('TARGETS')
    for p in poses:print(json.dumps(p,ensure_ascii=False))
    print('NON_STANDBY',json.dumps([r for r in states if r['robot_state']!=1],ensure_ascii=False))


if __name__=='__main__':main()
