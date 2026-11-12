"""190404 wrap timing, signed lead, empirical errors and bounded replay; offline."""
import bisect
import csv
import hashlib
import json
import math
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
from replay_servo_references import read_inputs,replay
from servo_experimental.bounded_segment import CommandSegmentInterpolator
from servo_experimental.reference_replay import pose_distance,rotation

ROOT=Path(__file__).resolve().parent


def csv_write(path,rows):
    if not rows:return
    with path.open('w',newline='') as f:
        w=csv.DictWriter(f,fieldnames=list(rows[0]));w.writeheader();w.writerows(rows)


def distribution(values):
    return {key:float(value) for key,value in zip(('p50','p95','p99','max'),np.percentile(values,[50,95,99,100]))}


def unit(v):
    v=np.array(v,dtype=float);n=np.linalg.norm(v)
    return None if n<1e-12 else v/n


def metric_segments(rows,segments):
    budget_t=budget_r=overshoot_t=overshoot_r=0;max_t=max_r=0.
    for segment in segments:
        samples=[r for r in rows if r['step']==segment['step']]
        if not samples:continue
        start=np.array(segment['stream_start']);target=np.array(segment['target'])
        t_axis=unit(target[:3]-start[:3])
        target_r=(rotation(target[3:])*rotation(start[3:]).inv()).as_rotvec()
        r_axis=unit(target_r)
        arc_t=arc_r=0.;previous=start;net_t=net_r=0.;projection_t=projection_r=0.
        for row in samples:
            p=np.array(row['pose']);dt,dr=pose_distance(previous,p);arc_t+=dt;arc_r+=dr
            nt,nr=pose_distance(start,p);net_t=max(net_t,nt);net_r=max(net_r,nr)
            if t_axis is not None:projection_t=max(projection_t,float(np.dot(p[:3]-start[:3],t_axis)))
            if r_axis is not None:
                projection_r=max(projection_r,float(np.dot((rotation(p[3:])*rotation(start[3:]).inv()).as_rotvec(),r_axis)*180/math.pi))
            previous=p
        budget_t+=max(net_t,arc_t)>segment['action_translation_mm']+1e-6
        budget_r+=max(net_r,arc_r)>segment['action_rotation_deg']+1e-6
        ot=max(0.,projection_t-segment['start_target_translation_mm'])
        orot=max(0.,projection_r-segment['start_target_rotation_deg'])
        max_t=max(max_t,ot);max_r=max(max_r,orot)
        overshoot_t+=ot>1e-6;overshoot_r+=orot>1e-6
    return dict(translation_budget_violating_segments=int(budget_t),rotation_budget_violating_segments=int(budget_r),
        translation_target_overshoot_segments=int(overshoot_t),rotation_target_overshoot_segments=int(overshoot_r),
        maximum_translation_target_overshoot_mm=max_t,maximum_rotation_target_overshoot_deg=max_r)


def bounded_replay(inputs,initial_period=None):
    predictions,commands,limits,transport,initial,_=inputs
    lookup={p['camera_timing']['inference_start_monotonic']:p for p in predictions}
    tracker=CommandSegmentInterpolator(initial,limits)
    rows=[];segments=[];active=None;failure=None
    for command in commands:
        now=command['requested_monotonic'];source=command['source_monotonic'];p=lookup[source]
        action=p['action_pipeline']['limited_canonical_action']
        if source!=active:
            active=source;start=tracker.last.copy()
            try:tracker.set_action(action,source,now,period=initial_period if not segments else None)
            except (RuntimeError,ValueError) as exc:
                failure=dict(step=p['step'],time=now,reason=str(exc));break
            target=tracker.segment['target']
            segments.append(dict(step=p['step'],stream_start=start.tolist(),target=target.tolist(),
                action_translation_mm=float(np.linalg.norm(action[:3])*1000),action_rotation_deg=float(np.linalg.norm(action[3:6])*180/math.pi),
                start_target_translation_mm=pose_distance(start,target)[0],start_target_rotation_deg=pose_distance(start,target)[1],
                period_used_s=tracker.segment['duration']))
        row=tracker.tick(now)
        rows.append(dict(step=p['step'],now=now,pose=row['pos'],fraction=row['fraction']))
    stats=metric_segments(rows,segments)
    # Diagnose, never silently change configured dynamic bounds. Constant-time
    # linear interpolation can have velocity/acceleration discontinuities.
    violations=dict(translation_velocity=0,rotation_velocity=0,translation_acceleration=0,rotation_acceleration=0)
    previous_pose=np.array(initial);previous_time=commands[0]['requested_monotonic']-commands[0]['actual_tick_dt_s']
    previous_v=np.zeros(3);previous_w=np.zeros(3)
    for row in rows:
        dt=row['now']-previous_time;p=np.array(row['pose'])
        v=(p[:3]-previous_pose[:3])/dt
        w=(rotation(p[3:])*rotation(previous_pose[3:]).inv()).as_rotvec()*180/math.pi/dt
        violations['translation_velocity']+=np.linalg.norm(v)>limits['translation_velocity_mm_s']+1e-6
        violations['rotation_velocity']+=np.linalg.norm(w)>limits['rotation_velocity_deg_s']+1e-6
        violations['translation_acceleration']+=np.linalg.norm(v-previous_v)/dt>limits['translation_acceleration_mm_s2']+1e-6
        violations['rotation_acceleration']+=np.linalg.norm(w-previous_w)/dt>limits['rotation_acceleration_deg_s2']+1e-6
        previous_pose=p;previous_time=row['now'];previous_v=v;previous_w=w
    stats.update(mode='bounded_command_linear_slerp',rows=len(rows),segments=len(segments),failure=failure,
        workspace_rejections=int(failure is not None and failure['reason']=='workspace_rejection'),
        dynamic_diagnostic_violations={k:int(v) for k,v in violations.items()},
        xyz_limits_unchanged=True,all_ABC_within_180=all(abs(v)<=180+1e-9 for row in rows for v in row['pose'][3:]))
    return stats,rows,segments


def main():
    inputs=read_inputs();predictions,commands,limits,transport,initial,_=inputs
    out=ROOT/'real_trials/20261112_190404_wrap_bounded_analysis_02'
    out.mkdir(exist_ok=False)
    ct=[c['requested_monotonic'] for c in commands]
    timing=[]
    for p in predictions:
        now=p['controller_feedback']['received'];idx=bisect.bisect_right(ct,now)-1
        if idx<0:
            command_pose=initial;v=np.zeros(3);w=np.zeros(3);command_time=None
        else:
            c=commands[idx];command_pose=c['pos'];v=np.array(c['translation_velocity_mm_s']);w=np.array(c['rotation_velocity_rad_s'])
            command_time=c['requested_monotonic']
        actual=p['current_controller_pose'];e=np.array(actual[:3])-np.array(command_pose[:3])
        e_r=(rotation(actual[3:])*rotation(command_pose[3:]).inv()).as_rotvec()*180/math.pi
        vu=unit(v);wu=unit(w)
        pt,pr=pose_distance(actual,command_pose)
        prev=timing[-1] if timing else None;dt=None if prev is None else now-prev['feedback_receive_monotonic']
        timing.append(dict(step=p['step'],feedback_receive_monotonic=now,command_time=command_time,
            translation_error_mm=pt,rotation_error_deg=pr,
            translation_error_growth_mm_s=None if prev is None else (pt-prev['translation_error_mm'])/dt,
            rotation_error_growth_deg_s=None if prev is None else (pr-prev['rotation_error_deg'])/dt,
            actual_minus_command_xyz_mm=e.tolist(),actual_minus_command_world_rotvec_deg=e_r.tolist(),
            translation_axis_signed_lead_mm=[None if abs(v[i])<1e-9 else float(e[i]*np.sign(v[i])) for i in range(3)],
            signed_translation_direction_lead_mm=None if vu is None else float(np.dot(e,vu)),
            signed_rotation_direction_lead_deg=None if wu is None else float(np.dot(e_r,wu)),
            commanded_translation_velocity_mm_s=v.tolist(),commanded_rotation_velocity_rad_s=w.tolist()))
    normal=timing[:217]
    distributions={k:distribution([r[k] for r in normal]) for k in ('translation_error_mm','rotation_error_deg')}
    candidates=[]
    def ceil_to(x,step):return math.ceil(x/step)*step
    for name,t,r in (
        ('p99_rounded_up',ceil_to(distributions['translation_error_mm']['p99'],.5),ceil_to(distributions['rotation_error_deg']['p99'],.1)),
        ('observed_max_rounded_up',ceil_to(distributions['translation_error_mm']['max'],1.),ceil_to(distributions['rotation_error_deg']['max'],.5))):
        candidates.append(dict(name=name,translation_mm=t,rotation_deg=r,
            normal_rejections=sum(x['translation_error_mm']>t or x['rotation_error_deg']>r for x in normal),
            first_late_rejection=next((x['step'] for x in timing[217:] if x['translation_error_mm']>t or x['rotation_error_deg']>r),None),
            applied_to_live=False))
    crossing=next(c for c in commands if c['pos'][5]<-180 or c['pos'][5]>180)
    # Define onset reproducibly; do not claim an objectively unique change point.
    after=[r for r in timing if r['feedback_receive_monotonic']>=crossing['requested_monotonic']]
    sustained=None
    for i in range(1,len(timing)-2):
        if timing[i]['step']<200:continue
        if all(timing[j]['rotation_error_growth_deg_s']>0 for j in range(i,i+3)):
            sustained=timing[i];break
    first_p99=next((r for r in timing[217:] if r['rotation_error_deg']>distributions['rotation_error_deg']['p99']),None)
    terminal_index=len(timing)-1
    while terminal_index>1 and timing[terminal_index-1]['rotation_error_growth_deg_s']>0:
        terminal_index-=1
    terminal_growth=dict(first_increasing_sample=timing[terminal_index]['step'],
        onset_interval_monotonic=[timing[terminal_index-1]['feedback_receive_monotonic'],timing[terminal_index]['feedback_receive_monotonic']],
        C_crossing_inside_interval=timing[terminal_index-1]['feedback_receive_monotonic']<=crossing['requested_monotonic']<=timing[terminal_index]['feedback_receive_monotonic'])
    results=[]
    for mode in ('original','command'):
        summary,rows,segments=replay(mode,False,inputs)
        m=metric_segments(rows,segments)
        m.update(mode=mode,rows=len(rows),segments=len(segments),failure=summary['failure'],
            workspace_rejections=int(summary['failure'] is not None and summary['failure']['reason']=='workspace_rejection'))
        results.append(m)
    bounded,rows,segments=bounded_replay(inputs);results.append(bounded)
    csv_write(out/'bounded_commands.csv',rows);csv_write(out/'bounded_segments.csv',segments)
    csv_write(out/'all_steps_error_projection.csv',timing)
    csv_write(out/'steps_200_222_error_growth.csv',[r for r in timing if 200<=r['step']<=222])
    by_source={p['camera_timing']['inference_start_monotonic']:p['step'] for p in predictions}
    ci=next(i for i,c in enumerate(commands) if by_source[c['source_monotonic']]==200)
    prior_activation=next(c['requested_monotonic'] for c in commands if by_source[c['source_monotonic']]==199)
    first_period=commands[ci]['requested_monotonic']-prior_activation
    local_inputs=(predictions,commands[ci:],limits,transport,commands[ci-1]['pos'],inputs[-1])
    local=[]
    for mode in ('original','command'):
        summary,rs,ss=replay(mode,False,local_inputs,seed=commands[ci-1]);m=metric_segments(rs,ss)
        m.update(mode=mode,rows=len(rs),segments=len(ss),failure=summary['failure']);local.append(m)
    summary,rs,ss=bounded_replay(local_inputs,first_period);local.append(summary)
    csv_write(out/'terminal_200_bounded_commands.csv',rs)
    report=dict(physical_calls=0,C_first_outside=dict(time=crossing['requested_monotonic'],C_deg=crossing['pos'][5],source=crossing['source_monotonic']),
        first_feedback_after_crossing=after[0],sustained_three_positive_growth_candidate=sustained,
        first_late_above_normal_rotation_p99=first_p99,
        terminal_growth_interval=terminal_growth,terminal_200_seeded_replay=local,
        normal_steps_0_216_distribution=distributions,threshold_candidates=candidates,
        replay_comparison=results,
        interpretation='Signed lead projects actual-minus-requested pose onto requested world velocity. Positive indicates ahead along that local direction, not causality or a vendor-confirmed tracking error.',
        onset_definition='First of three positive rotation-error growth samples in steps>=200; also report late normal-p99 crossing. No unique onset assumed.',
        periods='Bounded interpolation uses previous target activation interval; first uses frozen 1/control_rate_hz. No future interval lookahead.',
        limitations=['Normal range is user-designated, not independently certified healthy.',
            'Controller pose receive times, not acquisition/consumption times; delay is unknown.',
            'Alternative replay is open-loop recorded input. Recorded real feedback is not a physical counterfactual.',
            'Linear/Slerp with linear time is budget-bounded but not acceleration-continuous; dynamic violations are diagnostic and block physical readiness.'])
    (out/'results.json').write_text(json.dumps(report,indent=2))
    trial=ROOT/'real_trials/20261112_190404_arm_only_servo_episode'
    (out/'input_sha256.json').write_text(json.dumps({str(trial/name):hashlib.sha256((trial/name).read_bytes()).hexdigest() for name in ('servo_commands.jsonl','predictions.jsonl','config_snapshot.json')},indent=2))
    print(json.dumps(report,indent=2))


if __name__=='__main__':main()
