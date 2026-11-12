"""Offline forensic analysis of saved artifacts; no ROS or robot calls."""
import bisect
import csv
import hashlib
import json
import math
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation

ROOT=Path(__file__).resolve().parent
TRIAL=ROOT/'real_trials/20261112_190404_arm_only_servo_episode'


def main():
    out=ROOT/'real_trials/20261112_190404_servo_incident_analysis'
    out.mkdir(exist_ok=False)
    predictions=[json.loads(x) for x in (TRIAL/'predictions.jsonl').read_text().splitlines()]
    commands=[json.loads(x) for x in (TRIAL/'servo_commands.jsonl').read_text().splitlines()]
    commands=[r for r in commands if 'pos' in r]
    joints=json.loads((TRIAL/'jointstate.json').read_text())['samples']
    summary=json.loads((TRIAL/'trial_summary.json').read_text())
    start=commands[0]['requested_monotonic'];stop=summary['stop_requested_monotonic']
    def r(abc):return Rotation.from_euler('ZYZ',abc,degrees=True)
    def qv(row,key):
        values=dict(zip(row['names'],row[key]))
        return [math.degrees(values[f'joint_{i}']) for i in range(1,7)]
    qt=np.array([x['receive'] for x in joints]);q=np.array([qv(x,'position') for x in joints]);v=np.array([qv(x,'velocity') for x in joints])
    ct=[x['requested_monotonic'] for x in commands]
    abc=np.array([x['pos'][3:] for x in commands]);rot=r(abc)
    angles=(rot[:-1].inv()*rot[1:]).magnitude()*180/math.pi
    diffs=np.diff(abc,axis=0);euler_norm=np.linalg.norm(diffs,axis=1)
    raw_angles=np.array([np.linalg.norm(x['raw_action'][3:6])*180/math.pi for x in predictions])
    timeline=[]
    for pr in predictions:
        receive=pr['controller_feedback']['received'];idx=bisect.bisect_right(ct,receive)-1
        if idx<0:continue
        command=commands[idx];pose=pr['current_controller_pose']
        ji=bisect.bisect_right(qt,receive)-1
        row=dict(step=pr['step'],feedback_receive_monotonic=receive,
            seconds_before_stop=receive-stop,robot_state_polled_before_inference=pr['robot_state'],
            raw_rotation_deg=float(np.linalg.norm(pr['raw_action'][3:6])*180/math.pi),
            filtered_rotation_deg=float(np.linalg.norm(pr['action_pipeline']['limited_canonical_action'][3:6])*180/math.pi),
            raw_translation_mm=float(np.linalg.norm(pr['raw_action'][:3])*1000),
            current_ABC_deg=pose[3:],command_ABC_deg=command['pos'][3:],
            command_requested_monotonic=command['requested_monotonic'],
            command_age_at_feedback_s=receive-command['requested_monotonic'],
            orientation_difference_deg=float((r(pose[3:]).inv()*r(command['pos'][3:])).magnitude()*180/math.pi),
            position_difference_mm=float(np.linalg.norm(np.array(pose[:3])-command['pos'][:3])),
            joint_deg=q[ji].tolist(),joint_velocity_deg_s=v[ji].tolist())
        timeline.append(row)
    crossings=np.where(q[:,4]<-135)[0]
    c_cross=[x for x in commands if x['pos'][5]<-180]
    peak=int(np.argmax(abs(v[:,4])))
    result=dict(physical_calls=0,classification='OFFLINE_INCIDENT_ANALYSIS',
        reason=summary['reason'],robot_state_code=10,robot_state_name='SAFE_OFF2',
        controller_specific_alarm_code='NOT_RECORDED',stop_ack=summary['stop_ack'],
        runtime_start_monotonic=start,stop_requested_monotonic=stop,
        final_joint_deg=q[-1].tolist(),minimum_joint_deg=q.min(0).tolist(),maximum_joint_deg=q.max(0).tolist(),
        peak_absolute_joint_velocity_deg_s=abs(v).max(0).tolist(),
        peak_J5_velocity=dict(receive=float(qt[peak]),deg_s=float(v[peak,4]),joint_deg=q[peak].tolist()),
        historical_J5_reference_lower_deg=-135.,current_controller_J5_limit_verified=False,
        first_below_historical_J5_reference=None if not len(crossings) else dict(receive=float(qt[crossings[0]]),angle_deg=float(q[crossings[0],4])),
        first_published_C_below_minus180=None if not c_cross else dict(receive=c_cross[0]['requested_monotonic'],ABC=c_cross[0]['pos'][3:]),
        maximum_raw_model_rotation_deg=float(raw_angles.max()),maximum_raw_model_rotation_step=int(np.argmax(raw_angles)),
        maximum_stream_pair_geodesic_deg=float(angles.max()),
        maximum_stream_pair_Euler_L2_deg=float(euler_norm.max()),
        maximum_commanded_rotation_speed_deg_s=max(float(np.linalg.norm(x['rotation_velocity_rad_s'])*180/math.pi) for x in commands),
        minimum_distance_stream_B_to_0_or_180_deg=float(min(min(abs(x),abs(abs(x)-180)) for x in abc[:,1])),
        last_feedback_comparison=timeline[-1],
        caveats=['Command comparison uses most recent requested command <= feedback receive time; vendor consumption time unknown.',
                 'No vendor receipt ACK per ServoL message; publisher success is not controller execution proof.',
                 'JointState joint names reordered; raw array order is joint_1, joint_2, joint_4, joint_5, joint_3, joint_6.',
                 'Crossing C=-180 is correlated with later divergence, not proof of vendor Euler interpolation behavior.',
                 'J5 crosses historical -135 lower reference; actual configured limit and alarm are not recorded.'])
    (out/'incident_summary.json').write_text(json.dumps(result,indent=2))
    with (out/'feedback_command_timeline.csv').open('w',newline='') as f:
        writer=csv.DictWriter(f,fieldnames=list(timeline[0]));writer.writeheader();writer.writerows(timeline)
    with (out/'jointstate_last_5_seconds.csv').open('w',newline='') as f:
        writer=csv.writer(f);writer.writerow(['receive_monotonic','seconds_before_stop',*[f'J{i}_deg' for i in range(1,7)],*[f'J{i}_velocity_deg_s' for i in range(1,7)]])
        for i in np.where(qt>=stop-5)[0]:writer.writerow([qt[i],qt[i]-stop,*q[i],*v[i]])
    provenance={name:hashlib.sha256((TRIAL/name).read_bytes()).hexdigest() for name in ('predictions.jsonl','servo_commands.jsonl','jointstate.json','trial_summary.json','config_snapshot.json')}
    (out/'source_sha256.json').write_text(json.dumps(provenance,indent=2))
    print(json.dumps(result,indent=2))
    print('FINAL TIMELINE')
    for row in timeline[-8:]:print(json.dumps(row))


if __name__=='__main__':main()
