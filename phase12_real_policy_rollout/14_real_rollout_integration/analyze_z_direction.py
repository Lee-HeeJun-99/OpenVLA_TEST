"""Offline-only raw/action/feedback timestamp audit. No ROS or model calls."""
import argparse
import bisect
import csv
import hashlib
import json
import re
from pathlib import Path
import sys
from collections import Counter
import numpy as np

ROOT=Path(__file__).resolve().parent
sys.path.insert(0,str(ROOT.parent/'03_shadow_mode'))
from jointstate_flange_fk import A0509FlangeFK, CANONICAL_JOINTS, _origin


def load(path):return json.loads(path.read_text())
def lines(path):
    if not path.exists():return []
    return [json.loads(s) for s in path.read_text().splitlines() if s.strip()]
def direction(value):
    # Numerical sign only, not a physical accuracy/PASS tolerance.
    if value is None:return 'UNKNOWN'
    return 'UP' if value>1e-9 else 'DOWN' if value< -1e-9 else 'ZERO'
def agrees(a,b):
    return None if a is None or b is None else direction(a)==direction(b)


def driver_alarms():
    records=[]
    for path in Path('/home/ubuntu/.ros/log').glob('ros2_control_node_*_1793*.log'):
        text=path.read_text(errors='replace').splitlines()
        for i,line in enumerate(text):
            match=re.search(r'\[(179\d+\.\d+)\].*\bindex\s*:\s*(\d+)',line)
            if not match:continue
            params=[]
            for following in text[i+1:i+6]:
                if 'param :' in following:params.append(following.split('param :',1)[1].strip())
                elif params:break
            records.append(dict(wall_time=float(match[1]),index=int(match[2]),
                parameters=params,log_path=str(path),line=i+1))
    return sorted(records,key=lambda r:r['wall_time'])
def write_csv(path,rows):
    if not rows:return
    keys=list(dict.fromkeys(k for r in rows for k in r))
    with path.open('w',newline='') as f:
        w=csv.DictWriter(f,fieldnames=keys);w.writeheader();w.writerows(rows)


def trajectory(path,fk):
    source=load(path/'jointstate.json')['samples']
    samples=[r for r in source if r.get('valid',True) and len(r.get('names',[]))==6 and
        len(set(r['names']))==6 and set(r['names'])==set(CANONICAL_JOINTS) and
        len(r.get('position',[]))==6 and np.isfinite(r['position']).all()]
    times=np.array([r['receive'] for r in samples])
    if np.any(np.diff(times)<0):raise ValueError('receive_timestamp_regression')
    q=np.array([[dict(zip(r['names'],r['position']))[n] for n in CANONICAL_JOINTS] for r in samples])
    t=np.broadcast_to(np.eye(4),(len(q),4,4)).copy()
    for j,joint in enumerate(fk.chain):
        t=t@np.array(_origin(joint.xyz,joint.rpy))
        axis=np.array(joint.axis);axis/=np.linalg.norm(axis)
        x,y,z=axis;cross=np.array([[0,-z,y],[z,0,-x],[-y,x,0]])
        c=np.cos(q[:,j])[:,None,None];s=np.sin(q[:,j])[:,None,None]
        rot=c*np.eye(3)+(1-c)*np.outer(axis,axis)+s*cross
        m=np.broadcast_to(np.eye(4),(len(q),4,4)).copy();m[:,:3,:3]=rot;t=t@m
    xyz=t[:,:3,3]*1000
    for idx in (0,len(q)//2,len(q)-1):
        reference=fk.compute(dict(zip(CANONICAL_JOINTS,q[idx])))['position_m']
        if not np.allclose(xyz[idx],np.array(reference)*1000,atol=1e-9,rtol=0):raise ValueError('batch_FK_mismatch')
    return samples,times,xyz


def before(times,time):return max(0,min(len(times)-1,bisect.bisect_right(times,time)-1))


def moveline(path):
    result=[]
    for r in load(path/'episode_records.json'):
        rec=r['record'];receipt=rec.get('command_receipt') or {}
        if receipt.get('sent_at') is None:continue
        current=rec['observation']['tcp_m_abc'];request=receipt['request']
        if request.get('ref')!=0 or request.get('mode')!=0:raise ValueError('unexpected_MoveLine_frame_or_mode')
        b=r['jointstate_before'];a=r.get('jointstate_after')
        observed=None if 'fk_after' not in r else 1000*(r['fk_after']['position_m'][2]-r['fk_before']['position_m'][2])
        raw=r['raw_action'][2]*1000;limited=rec['filtered_action'][2]*1000
        delivered=request['pos'][2]-1000*current[2]
        result.append(dict(trial=path.name,controller='MoveLine',step=r['step_id'],
            inference_start=r['camera_timing']['inference_start_monotonic'],
            inference_end=r['camera_timing']['inference_end_monotonic'],dispatch=receipt['sent_at'],
            ack=receipt.get('ack_at'),window_end=None if a is None else a['receive'],
            before_joint_receive=b['receive'],after_joint_receive=None if a is None else a['receive'],
            raw_z_mm=raw,limited_z_mm=limited,submitted_net_z_mm=delivered,
            observed_fk_z_mm=observed,raw_direction=direction(raw),submitted_direction=direction(delivered),
            observed_direction=direction(observed),raw_vs_observed=agrees(raw,observed),
            submitted_vs_observed=agrees(delivered,observed),raw_vs_limited=agrees(raw,limited),
            limited_vs_request_error_mm=delivered-limited,ack_success=receipt['state']=='completed',
            comparison='SETTLED_BEFORE_AFTER' if a else 'MISSING_AFTER_FEEDBACK',
            max_observed_joint_delta_deg=None if not a else float(np.degrees(max(
                abs(dict(zip(a['names'],a['position']))[n]-dict(zip(b['names'],b['position']))[n]) for n in CANONICAL_JOINTS))),
            current_controller_z_mm=1000*current[2],target_controller_z_mm=request['pos'][2],
            feedback_age_before_dispatch_ms=(receipt['sent_at']-b['receive'])*1000))
    return result


def servo(path,times,xyz):
    predictions=lines(path/'predictions.jsonl');logs=lines(path/'servo_commands.jsonl')
    previous=load(path/'preflight.json')['initial_pose'][2]
    streams=[];pending=None
    for r in logs:
        if 'pos' in r:pending=r
        elif r.get('event')=='PUBLISH_RETURNED_NOT_VENDOR_ACK' and pending is not None:
            if r['source_monotonic']!=pending['source_monotonic']:raise ValueError('stream_source_pair_mismatch')
            streams.append(dict(source=r['source_monotonic'],t=r['published_monotonic'],
                z=pending['pos'][2],previous_z=previous,delta=pending['pos'][2]-previous))
            previous=pending['pos'][2];pending=None
    groups=[]
    for s in streams:
        if not groups or groups[-1][0]['source']!=s['source']:groups.append([])
        groups[-1].append(s)
    by_source={p['camera_timing']['inference_start_monotonic']:p for p in predictions}
    result=[];ticks=[]
    for i,group in enumerate(groups):
        first,last=group[0],group[-1];p=by_source[first['source']]
        end=groups[i+1][0]['t'] if i+1<len(groups) else min(times[-1],last['t']+.2)
        lo=before(times,first['t']);hi=before(times,end)
        observed=float(xyz[hi,2]-xyz[lo,2]);raw=p['raw_action'][2]*1000
        limited=p['action_pipeline']['limited_canonical_action'][2]*1000
        net=last['z']-first['previous_z']
        result.append(dict(trial=path.name,controller='ServoL',step=p['step'],
            inference_start=first['source'],inference_end=p['camera_timing']['inference_end_monotonic'],
            dispatch=first['t'],window_end=end,before_joint_receive=float(times[lo]),after_joint_receive=float(times[hi]),
            raw_z_mm=raw,limited_z_mm=limited,submitted_net_z_mm=net,
            observed_fk_z_mm=observed,raw_direction=direction(raw),submitted_direction=direction(net),
            observed_direction=direction(observed),raw_vs_observed=agrees(raw,observed),
            submitted_vs_observed=agrees(net,observed),raw_vs_limited=agrees(raw,limited),
            stream_messages=len(group),ack_success=None,comparison='CONTINUOUS_DISPATCH_WINDOW_NOT_SETTLED',
            target_z_mm=p['action_pipeline']['target_pose_candidate_mm_zyz_deg'][2],
            final_stream_z_mm=last['z'],feedback_age_before_dispatch_ms=(first['t']-times[lo])*1000,
            feedback_age_at_window_end_ms=(end-times[hi])*1000))
        for j,s in enumerate(group):
            tick_end=group[j+1]['t'] if j+1<len(group) else end
            tick=dict(trial=path.name,step=p['step'],dispatch=s['t'],window_end=tick_end,
                submitted_z_mm=s['z'],submitted_increment_z_mm=s['delta'])
            for lag in (0,.1,.2,.3):
                start=s['t']+lag;finish=tick_end+lag
                delta=None if finish>times[-1] else float(xyz[before(times,finish),2]-xyz[before(times,start),2])
                tick[f'fk_delta_lag_{lag:.1f}s_mm']=delta
            ticks.append(tick)
    return result,ticks,streams


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args();args.output.mkdir(parents=True,exist_ok=False)
    urdf=Path('/home/ubuntu/robot_ws/src/doosan-robot2/dsr_description2/urdf/a0509.urdf')
    fk=A0509FlangeFK(urdf);all_rows=[];all_ticks=[];summaries=[];inventory=[];hashes={}
    alarms=driver_alarms();matched_alarms=[]
    for summary_file in sorted((ROOT/'real_trials').glob('202611*/trial_summary.json')):
        path=summary_file.parent;summary=load(summary_file)
        if not summary.get('arm_commands',summary.get('servo_publish_attempts',0)):continue
        mode='ServoL' if (path/'servo_commands.jsonl').exists() else 'MoveLine'
        inventory.append(dict(trial=path.name,controller=mode,status=summary['status'],
            prediction_file=str(path/'predictions.jsonl'),trajectory_file=str(path/'jointstate.json')))
        if not (path/'jointstate.json').exists():continue
        samples,times,xyz=trajectory(path,fk)
        if mode=='ServoL':rows,ticks,streams=servo(path,times,xyz);all_ticks.extend(ticks)
        else:rows=moveline(path);streams=[]
        # ROS driver logs use wall/ROS epoch; convert HOST_MONOTONIC through
        # simultaneously recorded callback clock values, never across boots.
        offset=float(np.median([s['source']+s.get('header_age',0)-s['receive'] for s in samples]))
        trial_alarms=[a for a in alarms if times[0]+offset<=a['wall_time']<=times[-1]+offset]
        matched_alarms.extend(dict(a,trial=path.name,estimated_monotonic=a['wall_time']-offset) for a in trial_alarms)
        for r in rows:
            end=r.get('ack') if mode=='MoveLine' else r['window_end']
            found=[] if end is None else [a for a in trial_alarms if r['dispatch']+offset-.01<=a['wall_time']<=end+offset+.01]
            r['driver_alarm_indices']=','.join(str(a['index']) for a in found)
            r['dispatch_wall_time_estimate']=r['dispatch']+offset
        all_rows.extend(rows)
        valid=[r for r in rows if r['observed_fk_z_mm'] is not None]
        if not valid:continue
        start=valid[0]['dispatch'];end=valid[-1]['window_end']
        lo=before(times,start);hi=before(times,end)
        trajectory_rows=[dict(receive_monotonic=float(times[i]),source_timestamp=samples[i]['source'],
            x_mm=float(xyz[i,0]),y_mm=float(xyz[i,1]),z_mm=float(xyz[i,2])) for i in range(lo,hi+1)]
        write_csv(args.output/(path.name+'_trajectory_fk.csv'),trajectory_rows)
        totals=dict(trial=path.name,controller=mode,status=summary['status'],
            predictions_logged=len(lines(path/'predictions.jsonl')),submitted_prediction_windows=len(rows),
            matched_feedback_windows=len(valid),raw_directions=dict(Counter(r['raw_direction'] for r in rows)),
            limited_directions=dict(Counter(direction(r['limited_z_mm']) for r in rows)),
            submitted_directions=dict(Counter(r['submitted_direction'] for r in rows)),
            observed_directions=dict(Counter(r['observed_direction'] for r in valid)),
            raw_vs_limited_mismatches=sum(not r['raw_vs_limited'] for r in rows),
            raw_vs_observed_mismatches=sum(not r['raw_vs_observed'] for r in valid),
            submitted_vs_observed_mismatches=sum(not r['submitted_vs_observed'] for r in valid),
            start_fk_z_mm=float(xyz[lo,2]),end_fk_z_mm=float(xyz[hi,2]),net_observed_z_mm=float(xyz[hi,2]-xyz[lo,2]),
            min_fk_z_mm=float(xyz[lo:hi+1,2].min()),max_fk_z_mm=float(xyz[lo:hi+1,2].max()),
            net_submitted_z_mm=sum(r['submitted_net_z_mm'] for r in rows),
            max_feedback_boundary_age_ms=max(r['feedback_age_before_dispatch_ms'] for r in rows))
        prediction_rows=lines(path/'predictions.jsonl')
        totals['all_prediction_raw_directions']=dict(Counter(direction(p['raw_action'][2]*1000) for p in prediction_rows))
        totals['predictions_without_paired_dispatch_window']=len(prediction_rows)-len(rows)
        totals['summary_reported_arm_submissions']=summary.get('arm_commands',summary.get('servo_publish_attempts'))
        totals['driver_alarm_counts']=dict(Counter(a['index'] for a in trial_alarms))
        totals['stationary_identical_joint_windows']=sum(r.get('max_observed_joint_delta_deg')==0 for r in valid)
        totals['unreachable_1206_windows']=sum('1206' in r['driver_alarm_indices'].split(',') for r in rows)
        totals['net_submitted_z_mm_matched_feedback_only']=sum(r['submitted_net_z_mm'] for r in valid)
        if streams:
            totals['stream_publish_returns']=len(streams)
            totals['stream_increment_directions']=dict(Counter(direction(s['delta']) for s in streams))
        summaries.append(totals)
        for name in ('predictions.jsonl','episode_records.json','servo_commands.jsonl','jointstate.json','arm_commands.json','preflight.json'):
            f=path/name
            if f.exists():hashes[str(f)]=hashlib.sha256(f.read_bytes()).hexdigest()
    write_csv(args.output/'per_prediction_comparison.csv',all_rows)
    write_csv(args.output/'servo_stream_lag_comparison.csv',all_ticks)
    mismatch=[r for r in all_rows if r['raw_vs_observed'] is False or r['submitted_vs_observed'] is False]
    write_csv(args.output/'mismatch_windows.csv',mismatch)
    (args.output/'driver_alarm_evidence.json').write_text(json.dumps(matched_alarms,indent=2))
    report=dict(physical_commands=0,scope='EXISTING_NOVEMBER_PHYSICAL_EPISODE_LOGS_ONLY',
        coordinate='BASE/base_link +Z upward; raw m converted to mm; measured joints radians reordered by name',
        actual_trajectory='URDF FK flange from measured JointState, not independent measured TCP',
        sign_epsilon_mm=1e-9,sign_epsilon_is_not_hardware_tolerance=True,
        matching='MoveLine settled before/after with dispatch/ACK; ServoL source_monotonic exact join to publish-return paired requests, first publish to next-source first publish',
        limitations=['Servo publish return is not vendor ACK','Continuous windows include previous-target inertia and are not isolated settled step responses',
            'Servo final window includes at most 0.2s after last publish; stop/fault transient can contribute',
            'No formal active TCP verification; flange FK has nominal-model bias',
            'No inference, ROS import, service call or robot command performed'],
        inventory=inventory,trials=summaries,input_sha256=hashes,urdf_sha256=hashlib.sha256(urdf.read_bytes()).hexdigest())
    (args.output/'z_direction_summary.json').write_text(json.dumps(report,indent=2))
    print(json.dumps(summaries,indent=2))


if __name__=='__main__':main()
