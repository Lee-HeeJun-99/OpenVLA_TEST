"""Recorded-input generator replay, NOT a vendor/robot physics simulation."""
import bisect
import csv
import difflib
import hashlib
import json
import math
from pathlib import Path
import numpy as np
import yaml
from servo_experimental.controller import ServoShadowController
from servo_experimental.reference_replay import (
    pose_distance,install_reference,DiagnosticGuardLimits,OfflineStreamBoundary)

ROOT=Path(__file__).resolve().parent
TRIAL=ROOT/'real_trials/20261112_190404_arm_only_servo_episode'


def write_csv(path,rows):
    if not rows:return
    with path.open('w',newline='') as f:
        writer=csv.DictWriter(f,fieldnames=list(rows[0]));writer.writeheader();writer.writerows(rows)


def read_inputs():
    predictions=[json.loads(s) for s in (TRIAL/'predictions.jsonl').read_text().splitlines()]
    commands=[json.loads(s) for s in (TRIAL/'servo_commands.jsonl').read_text().splitlines()]
    commands=[c for c in commands if 'pos' in c]
    frozen=json.loads((TRIAL/'config_snapshot.json').read_text())['files']
    def snapshot(suffix):return next(v['content'] for k,v in frozen.items() if k.endswith(suffix))
    limits=yaml.safe_load(snapshot('/01_configs/safety_limits.yaml'))
    transport=yaml.safe_load(snapshot('/configs/servo_episode.yaml'))['servo_transport']
    initial=json.loads((TRIAL/'preflight.json').read_text())['initial_pose']
    return predictions,commands,limits,transport,initial,frozen


def replay(mode,guarded,inputs,following_mm=50.,following_deg=8.,seed=None,budget_check=True):
    predictions,commands,limits,transport,initial,_=inputs
    by_source={p['camera_timing']['inference_start_monotonic']:p for p in predictions}
    received=[p['controller_feedback']['received'] for p in predictions]
    tracker=ServoShadowController(initial,limits,transport=transport)
    if seed is not None:
        tracker.v=np.array(seed['translation_velocity_mm_s'])
        tracker.w=np.array(seed['rotation_velocity_rad_s'])
    tracker.last_tick=commands[0]['requested_monotonic']-commands[0]['actual_tick_dt_s']
    published=[];stop=[]
    # Explicit OFFLINE diagnostic references: existing raw 50mm/8deg numbers,
    # NOT approved tracking tolerances. Sensitivity sweeps are reported too.
    guard=OfflineStreamBoundary(initial,DiagnosticGuardLimits(1e-6,1e-6,following_mm,following_deg,
        limits['max_action_age_sec']),lambda pose:published.append(pose),lambda:stop.append('FAKE_STOP'),
        check_action_budget=budget_check)
    rows=[];segments=[];active=None;last_pose=list(initial);failure=None
    for command in commands:
        now=command['requested_monotonic'];source=command['source_monotonic'];p=by_source[source]
        action=p['action_pipeline']['limited_canonical_action']
        if source!=active:
            active=source
            segment_start=list(last_pose)
            if mode=='original':
                anchor=list(p['current_controller_pose'])
                target=p['action_pipeline']['target_pose_candidate_mm_zyz_deg']
            else:
                anchor,target=install_reference(tracker,action,p['current_controller_pose'],mode)
            try:tracker.set_target(target,source,now)
            except (RuntimeError,ValueError) as exc:
                failure=dict(reason=str(exc),step=p['step'],time=now,stage='set_target')
                break
            if guarded:guard.begin_segment(p['step'],action)
            segments.append(dict(step=p['step'],stream_start=segment_start,
                actual_pose=p['current_controller_pose'],target=target,target_anchor=anchor,
                action_translation_mm=float(np.linalg.norm(action[:3])*1000),
                action_rotation_deg=float(np.linalg.norm(action[3:6])*180/math.pi),
                start_target_translation_mm=pose_distance(segment_start,target)[0],
                start_target_rotation_deg=pose_distance(segment_start,target)[1],
                actual_start_translation_mm=pose_distance(segment_start,p['current_controller_pose'])[0],
                actual_start_rotation_deg=pose_distance(segment_start,p['current_controller_pose'])[1],
                stream_translation_arc_mm=0.,stream_rotation_arc_deg=0.,
                published_count=0))
        try:row=tracker.tick(now)
        except (RuntimeError,ValueError) as exc:
            failure=dict(reason=str(exc),step=p['step'],time=now,stage='tick')
            break
        pose=row['pos']
        if guarded:
            fi=bisect.bisect_right(received,now)-1
            feedback=initial if fi<0 else predictions[fi]['current_controller_pose']
            feedback_time=now if fi<0 else received[fi]
            try:guard.inspect_and_publish(pose,now,feedback,feedback_time)
            except RuntimeError:
                failure=dict(reason=guard.reason,step=p['step'],time=now,event=guard.events[-1] if guard.events else None)
                # Demonstrate permanent inhibition with a deliberate extra
                # callback attempt; it must not append a fake publication.
                before=len(published)
                try:guard.inspect_and_publish(pose,now+.05,feedback,feedback_time)
                except RuntimeError:pass
                assert len(published)==before
                break
        delta_t,delta_r=pose_distance(last_pose,pose)
        segment=segments[-1]
        segment['stream_translation_arc_mm']+=delta_t
        segment['stream_rotation_arc_deg']+=delta_r
        segment['published_count']+=1
        segment['last_pose']=list(pose)
        record_t,record_r=pose_distance(pose,command['pos'])
        rows.append(dict(step=p['step'],now=now,pose=pose,recorded_pose=command['pos'],
            recorded_difference_mm=record_t,recorded_difference_deg=record_r,
            incremental_translation_mm=delta_t,incremental_rotation_deg=delta_r))
        last_pose=list(pose)
    for s in segments:
        s['rotation_arc_action_ratio']=s['stream_rotation_arc_deg']/max(s['action_rotation_deg'],1e-12)
        s['translation_arc_action_ratio']=s['stream_translation_arc_mm']/max(s['action_translation_mm'],1e-12)
    summary=dict(mode=mode,guarded=guarded,rows=len(rows),segments=len(segments),
        failure=failure,fake_stop_requests=len(stop),additional_commands_after_abort=0,
        maximum_rotation_arc_action_ratio=max(s['rotation_arc_action_ratio'] for s in segments),
        maximum_translation_arc_action_ratio=max(s['translation_arc_action_ratio'] for s in segments),
        last_segment_rotation_arc_deg=segments[-1]['stream_rotation_arc_deg'],
        last_segment_action_rotation_deg=segments[-1]['action_rotation_deg'],
        max_recorded_pose_difference_mm=max((r['recorded_difference_mm'] for r in rows),default=0),
        max_recorded_pose_difference_deg=max((r['recorded_difference_deg'] for r in rows),default=0),
        following_limit_mm=following_mm,following_limit_deg=following_deg,
        action_budget_check=budget_check,
        original_dynamics_and_transport_limits_unchanged=True)
    return summary,rows,segments


def proposal_diff(mode):
    old='''def update(tracker, limited_action, actual_pose, source, now):
    # Original: target from actual; tracker continues commanded state.
    anchor = actual_pose
    target = delta_target(anchor, limited_action)
    tracker.set_target(target, source, now)
'''
    new=f'''def update(tracker, limited_action, actual_pose, source, now):
    # OFFLINE proposal {mode}; guard checks candidate BEFORE fake publication.
    # Existing hard gates/workspace checks must validate the NEW target too.
    anchor, target = install_reference(tracker, limited_action, actual_pose, "{mode}")
    tracker.set_target(target, source, now)
    guard.begin_segment(source, limited_action)

def candidate_tick(now, feedback, feedback_received):
    candidate = tracker.tick(now)
    # Fake callbacks only in this experiment. Fault latches inhibit + one stop.
    guard.inspect_and_publish(candidate['pos'], now, feedback, feedback_received)
'''
    return ''.join(difflib.unified_diff(old.splitlines(True),new.splitlines(True),
        fromfile='original_reference.py',tofile=f'proposal_{mode}_offline.py'))


def main():
    inputs=read_inputs();out=ROOT/'real_trials/20261112_190404_reference_replay_03'
    out.mkdir(exist_ok=False)
    summaries=[];selected=[]
    for mode in ('original','command','actual'):
        for guarded in (False,True):
            summary,rows,segments=replay(mode,guarded,inputs)
            summaries.append(summary)
            tag=f'{mode}_{"guarded" if guarded else "unguarded"}'
            write_csv(out/f'{tag}_commands.csv',rows)
            write_csv(out/f'{tag}_segments.csv',segments)
            if mode=='original' and not guarded:
                selected=[s for s in segments if 215<=s['step']<=222]
    sweeps=[]
    for t,r in ((5.,1.),(10.,2.),(20.,4.),(50.,8.)):
        summary,_,_=replay('original',True,inputs,t,r,budget_check=False);sweeps.append(summary)
    predictions,commands,limits,transport,initial,frozen=inputs
    sources={p['camera_timing']['inference_start_monotonic']:p['step'] for p in predictions}
    ci=next(i for i,c in enumerate(commands) if sources[c['source_monotonic']]==215)
    local=(predictions,commands[ci:],limits,transport,commands[ci-1]['pos'],frozen)
    local_results=[]
    for mode in ('original','command','actual'):
        for guarded in (False,True):
            summary,rows,segments=replay(mode,guarded,local,seed=commands[ci-1])
            local_results.append(summary)
            write_csv(out/f'local215_{mode}_{guarded}_segments.csv',segments)
    result=dict(physical_calls=0,simulation_kind='RECORDED_INPUT_GENERATOR_REPLAY_NOT_VENDOR_OR_ROBOT_PHYSICS',
        diagnostic_thresholds_not_physically_approved=True,summary=summaries,following_limit_sensitivity=sweeps,
        selected_original_segments=selected,local_step215_seeded_comparison=local_results,
        limitation='Recorded real poses remain exogenous under alternatives; no counterfactual physical motion or task success can be inferred.')
    (out/'results.json').write_text(json.dumps(result,indent=2))
    write_csv(out/'steps_215_222_reference_table.csv',selected)
    for mode in ('command','actual'):(out/f'proposal_{mode}.diff').write_text(proposal_diff(mode))
    (out/'source_sha256.json').write_text(json.dumps({str(TRIAL/name):hashlib.sha256((TRIAL/name).read_bytes()).hexdigest()
        for name in ('predictions.jsonl','servo_commands.jsonl','config_snapshot.json','preflight.json')},indent=2))
    print(json.dumps(result,indent=2))


if __name__=='__main__':main()
