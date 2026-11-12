"""Offline prediction / target / stream / measured trajectory comparison."""
import csv
import hashlib
import json
import math
from pathlib import Path
import sys
import numpy as np
from scipy.spatial.transform import Rotation

ROOT=Path(__file__).resolve().parent
sys.path.insert(0,str(ROOT.parent/'03_shadow_mode'))
from jointstate_flange_fk import A0509FlangeFK,_origin


def save_csv(path,rows):
    with path.open('w',newline='') as f:
        w=csv.DictWriter(f,fieldnames=list(rows[0]));w.writeheader();w.writerows(rows)


def stats(values):
    a=np.asarray(values,dtype=float)
    return dict(count=len(a),median=float(np.median(a)),p95=float(np.percentile(a,95)),max=float(a.max()))


def cosine(a,b):
    denom=np.linalg.norm(a)*np.linalg.norm(b)
    return None if denom<=1e-12 else float(np.dot(a,b)/denom)


def main():
    trial=ROOT/'real_trials/20261112_190404_arm_only_servo_episode'
    output=ROOT/'real_trials/20261112_190404_prediction_trajectory_comparison'
    output.mkdir(exist_ok=False)
    predictions=[json.loads(s) for s in (trial/'predictions.jsonl').read_text().splitlines()]
    stream=[json.loads(s) for s in (trial/'servo_commands.jsonl').read_text().splitlines()]
    stream=[s for s in stream if 'pos' in s]
    joints=json.loads((trial/'jointstate.json').read_text())['samples']
    rot=lambda a:Rotation.from_euler('ZYZ',a,degrees=True)
    linked={}
    for s in stream:linked.setdefault(s['source_monotonic'],[]).append(s)
    translation_changed=rotation_changed=0;target_translation_errors=[];target_rotation_errors=[];pair_rows=[]
    for i,p in enumerate(predictions):
        raw=np.array(p['raw_action']);limited=np.array(p['action_pipeline']['limited_canonical_action'])
        pose=np.array(p['current_controller_pose']);target=np.array(p['action_pipeline']['target_pose_candidate_mm_zyz_deg'])
        translation_changed+=int(not np.allclose(raw[:3],limited[:3],atol=1e-12,rtol=0))
        rotation_changed+=int(not np.allclose(raw[3:6],limited[3:6],atol=1e-12,rtol=0))
        target_translation_errors.append(np.linalg.norm(target[:3]-pose[:3]-raw[:3]*1000))
        target_rotation_errors.append((rot(target[3:]).inv()*Rotation.from_rotvec(limited[3:6])*rot(pose[3:])).magnitude()*180/math.pi)
        if i+1==len(predictions):continue
        nxt=predictions[i+1];end=np.array(nxt['current_controller_pose'])
        measured_translation=end[:3]-pose[:3]
        measured_rotation=(rot(end[3:])*rot(pose[3:]).inv()).as_rotvec()
        source=p['camera_timing']['inference_start_monotonic'];commands=linked.get(source,[])
        row=dict(step=i,pose_receive_start=p['controller_feedback']['received'],pose_receive_end=nxt['controller_feedback']['received'],
            feedback_interval_s=nxt['controller_feedback']['received']-p['controller_feedback']['received'],
            raw_translation_xyz_mm=(raw[:3]*1000).tolist(),raw_rotation_vector_rad=raw[3:6].tolist(),
            filtered_rotation_vector_rad=limited[3:6].tolist(),target_pose=target.tolist(),
            raw_translation_mm=float(np.linalg.norm(raw[:3])*1000),observed_translation_mm=float(np.linalg.norm(measured_translation)),
            raw_rotation_deg=float(np.linalg.norm(raw[3:6])*180/math.pi),filtered_rotation_deg=float(np.linalg.norm(limited[3:6])*180/math.pi),
            observed_rotation_deg=float(np.linalg.norm(measured_rotation)*180/math.pi),
            observed_translation_xyz_mm=measured_translation.tolist(),observed_world_rotation_vector_rad=measured_rotation.tolist(),
            translation_direction_cosine=cosine(raw[:3],measured_translation),
            rotation_direction_cosine=cosine(limited[3:6],measured_rotation),
            translation_magnitude_ratio=float(np.linalg.norm(measured_translation)/(np.linalg.norm(raw[:3])*1000)),
            rotation_magnitude_ratio=float(np.linalg.norm(measured_rotation)/np.linalg.norm(limited[3:6])),
            translation_target_error_at_next_feedback_mm=float(np.linalg.norm(end[:3]-target[:3])),
            rotation_target_error_at_next_feedback_deg=float((rot(target[3:]).inv()*rot(end[3:])).magnitude()*180/math.pi),
            stream_messages_for_source=len(commands),
            stream_rotation_first_to_last_deg=None if len(commands)<2 else float((rot(commands[0]['pos'][3:]).inv()*rot(commands[-1]['pos'][3:])).magnitude()*180/math.pi),
            note='Overlapping continuous-control response window, not an isolated action gain or completion test')
        pair_rows.append(row)
    # Batch FK from measured joint angles. No numerical IK, ROS or simulation.
    urdf=Path('/home/ubuntu/robot_ws/src/doosan-robot2/dsr_description2/urdf/a0509.urdf')
    fk=A0509FlangeFK(urdf)
    qt=np.array([j['receive'] for j in joints])
    q=np.array([[dict(zip(j['names'],j['position']))[f'joint_{i}'] for i in range(1,7)] for j in joints])
    t=np.broadcast_to(np.eye(4),(len(q),4,4)).copy()
    for i,joint in enumerate(fk.chain):
        axis=np.asarray(joint.axis,dtype=float);axis/=np.linalg.norm(axis)
        turn=np.broadcast_to(np.eye(4),t.shape).copy()
        turn[:,:3,:3]=Rotation.from_rotvec(q[:,i,None]*axis).as_matrix()
        t=t@np.array(_origin(joint.xyz,joint.rpy))@turn
    rotations=Rotation.from_matrix(t[:,:3,:3]);quaternions=rotations.as_quat()
    trajectory=[dict(receive_monotonic=float(qt[i]),source_ros_stamp=joints[i]['source'],
        joint_deg=np.rad2deg(q[i]).tolist(),flange_xyz_mm=(t[i,:3,3]*1000).tolist(),
        flange_quaternion_xyzw=quaternions[i].tolist(),pose_kind='URDF_FK_FLANGE_NOT_CONTROLLER_TCP') for i in range(len(q))]
    command_rows=[]
    for c in stream:
        idx=int(np.searchsorted(qt,c['requested_monotonic'],side='right')-1)
        if idx<0:continue
        command_rows.append(dict(request_monotonic=c['requested_monotonic'],source_monotonic=c['source_monotonic'],
            command_pose=c['pos'],joint_receive_monotonic=float(qt[idx]),joint_age_s=float(c['requested_monotonic']-qt[idx]),
            computed_flange_xyz_mm=(t[idx,:3,3]*1000).tolist(),
            requested_vs_FK_flange_position_mm=float(np.linalg.norm(np.array(c['pos'][:3])-t[idx,:3,3]*1000)),
            requested_vs_FK_flange_rotation_deg=float((rot(c['pos'][3:]).inv()*rotations[idx]).magnitude()*180/math.pi),
            caveat='Timestamp-aligned requested pose vs computed flange; not vendor-confirmed following error'))
    groups={}
    for name,rs in [('all',pair_rows),('steps_0_to_216',pair_rows[:217]),('steps_217_to_222',pair_rows[217:])]:
        keys=['translation_direction_cosine','rotation_direction_cosine','translation_magnitude_ratio','rotation_magnitude_ratio',
            'translation_target_error_at_next_feedback_mm','rotation_target_error_at_next_feedback_deg']
        groups[name]={k:stats([r[k] for r in rs]) for k in keys}
        groups[name]['translation_opposite_direction_count']=sum(r['translation_direction_cosine']<0 for r in rs)
        groups[name]['rotation_opposite_direction_count']=sum(r['rotation_direction_cosine']<0 for r in rs)
    summary=dict(physical_calls=0,predictions=len(predictions),pose_feedback_samples=len(predictions),
        joint_trajectory_samples=len(joints),stream_pose_messages=len(stream),prediction_sources_in_stream=len(linked),
        last_prediction_not_published=predictions[-1]['camera_timing']['inference_start_monotonic'] not in linked,
        translation_modified_count=translation_changed,rotation_modified_count=rotation_changed,
        max_target_translation_conversion_error_mm=max(target_translation_errors),
        max_target_rotation_conversion_error_deg=max(target_rotation_errors),groups=groups,
        max_joint_age_at_publish_s=max(r['joint_age_s'] for r in command_rows),
        command_to_FK_flange_translation_mm=stats([r['requested_vs_FK_flange_position_mm'] for r in command_rows]),
        command_to_FK_flange_rotation_deg=stats([r['requested_vs_FK_flange_rotation_deg'] for r in command_rows]),
        urdf_path=str(urdf),urdf_sha256=hashlib.sha256(urdf.read_bytes()).hexdigest(),
        limitations=['No per-target completion wait; intervals include previous control state and command transport delay.',
                    'No per-message vendor ACK; source mapping is host publication bookkeeping only.',
                    'FK flange is computed from measured joints, not direct TCP measurement; formal TCP remains unverified.',
                    'No new physical pass tolerance invented; diagnostic errors and direction cosines reported as values.'])
    (output/'summary.json').write_text(json.dumps(summary,indent=2))
    (output/'input_sha256.json').write_text(json.dumps({name:hashlib.sha256((trial/name).read_bytes()).hexdigest() for name in ('predictions.jsonl','servo_commands.jsonl','jointstate.json')},indent=2))
    save_csv(output/'prediction_vs_controller_feedback.csv',pair_rows)
    save_csv(output/'measured_joint_FK_trajectory.csv',trajectory)
    save_csv(output/'stream_vs_joint_FK.csv',command_rows)
    print(json.dumps(summary,indent=2))


if __name__=='__main__':main()
