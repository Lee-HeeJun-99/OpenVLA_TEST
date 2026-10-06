#!/usr/bin/env python3
"""End-to-end command-free audit of FK, actions, gripper, K=5, safety and logging."""
from __future__ import annotations

import argparse, json, math, sys
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation

HERE=Path(__file__).resolve().parent; ROOT=HERE.parent
sys.path[:0]=[str(HERE),str(ROOT/'02_safety')]
from action_contract import matrix_to_doosan_zyz_deg, translation_m_to_mm
from gripper_contract import ClosednessHysteresis
from integrated_logger import FsyncJsonlLogger
from jointstate_flange_fk import A0509FlangeFK, CANONICAL_JOINTS
from oft_chunk_queue import OFTChunkQueue
from runtime_safety_supervisor import RuntimeSafetySupervisor
from action_rate_limiter import CanonicalActionRateLimiter
from workspace_contract import PHASE12_DATA_DERIVED_WORKSPACE


def _supervisor():
    # Operator-selected vel/acc=20 equivalent, still command-free and not motion approval.
    return RuntimeSafetySupervisor(max_action_age_sec=1.2,min_command_period_sec=.19,
        max_translation_m=.004,max_rotation_rad=math.radians(4.0),
        max_translation_velocity_m_s=.02,max_rotation_velocity_rad_s=math.radians(20.0),
        max_translation_acceleration_m_s2=.02,max_rotation_acceleration_rad_s2=math.radians(20.0))


def _read_first_valid(path, model):
    key='openvla_canonical_action' if model=='openvla' else 'oft_canonical_action_chunk'
    with open(path,encoding='utf-8') as f:
        for line in f:
            row=json.loads(line)
            if row.get('valid') and row.get(key): return row
    raise RuntimeError(f'no_valid_{model}_record')


def _candidate(fk, vector):
    values=[float(x) for x in vector]
    if len(values)!=7 or not all(math.isfinite(x) for x in values): raise ValueError('invalid_canonical_action')
    delta_mm=translation_m_to_mm(values[:3]).delta_mm
    current=np.asarray(fk['rotation_matrix'],dtype=np.float64)
    target=Rotation.from_rotvec(values[3:6]).as_matrix() @ current
    return {
        'target_position_mm':[(fk['position_m'][i]*1000.0)+delta_mm[i] for i in range(3)],
        'target_rotation_doosan_zyz_deg':matrix_to_doosan_zyz_deg(target).tolist(),
        'delta_translation_m':values[:3],'delta_translation_mm':list(delta_mm),
        'delta_rotation_world_rotvec_rad':values[3:6],
        'translation_empirical_gain':1.0,'legacy_2800_gain_used':False,
        'candidate_only':True,'motion_authorized':False,
    }


def _base_record(model,row,fk,raw_vector,limited,chunk_id,chunk_index,now,safety,gripper):
    vector=limited.limited_action
    candidate=_candidate(fk,vector); gd=gripper.resolve(raw_vector[6])
    workspace=PHASE12_DATA_DERIVED_WORKSPACE.inspect_delta(fk['position_m'],vector[:3])
    candidate['gripper']={'raw_closedness':gd.raw_closedness,'commanded_closed':gd.commanded_closed,
                          'command_suppressed':gd.command_suppressed,'reason':gd.reason}
    return {'classification':'OFFLINE_RECORDED_ACTION_CONTRACT_AUDIT','model':model,
        'episode_id':row['episode_id'],'frame_id':row['frame_id'],'phase':row.get('phase'),
        'instruction':row['instruction'],'model_checkpoint_contract':('step8130_k1' if model=='openvla' else 'vision_step28560_k5'),
        'pose_proxy':fk,'raw_canonical_action':list(raw_vector),'limited_canonical_action':list(vector),
        'canonical_action':list(vector),'chunk_id':chunk_id,'chunk_index':chunk_index,
        'limiter':{'translation_speed_clipped':limited.translation_speed_clipped,
          'rotation_speed_clipped':limited.rotation_speed_clipped,
          'translation_acceleration_clipped':limited.translation_acceleration_clipped,
          'rotation_acceleration_clipped':limited.rotation_acceleration_clipped,
          'translation_change_m':limited.translation_change_m,'rotation_change_rad':limited.rotation_change_rad,
          'translation_velocity_m_s':list(limited.translation_velocity_m_s),
          'rotation_velocity_rad_s':list(limited.rotation_velocity_rad_s),'command_issued':False},
        'safety_result':{'accepted':safety.accepted,'hold_required':safety.hold_required,
                         'hold_reason':safety.hold_reason,'command_issued':False},
        'workspace_result':{'accepted':workspace.accepted,'reason':workspace.reason,
          'command_issued':False,'status':PHASE12_DATA_DERIVED_WORKSPACE.provenance,
          'minimum_m':list(PHASE12_DATA_DERIVED_WORKSPACE.minimum_m),
          'maximum_m':list(PHASE12_DATA_DERIVED_WORKSPACE.maximum_m)},
        'command_candidate':candidate,'safety_limits_status':'OPERATOR_SELECTED_20_PROFILE_OFFLINE_ONLY_NOT_MOTION_APPROVED',
        'safety_limits':{'control_rate_hz':5.0,'max_translation_step_m':.004,
          'max_rotation_step_rad':math.radians(4.0),'max_translation_velocity_m_s':.02,
          'max_rotation_velocity_rad_s':math.radians(20.0),'max_translation_acceleration_m_s2':.02,
          'max_rotation_acceleration_rad_s2':math.radians(20.0)},
        'measured_tcp_pose':None,'measured_gripper_state':None,
        'executed_action':None,'ai_executed_action':None,'robot_delivered_command':None,'command_issued':False}


def run(openvla_path,oft_path,urdf,output):
    ov=_read_first_valid(openvla_path,'openvla'); of=_read_first_valid(oft_path,'oft')
    fk_engine=A0509FlangeFK(urdf); rows=[]
    with FsyncJsonlLogger(output,minimum_free_bytes=1) as log:
        vec=ov['openvla_canonical_action']['vector']; fk=fk_engine.compute(dict(zip(CANONICAL_JOINTS,ov['raw_joint_position'][:6])))
        sup=_supervisor();limiter=CanonicalActionRateLimiter();limited=limiter.limit(vec)
        safety=sup.inspect(action_id='openvla-frame0-k0',action=limited.limited_action,source_monotonic=10.,now_monotonic=10.,tcp_ok=True)
        rec=_base_record('openvla',ov,fk,vec,limited,'openvla-frame0',0,10.,safety,ClosednessHysteresis());log.append(rec);rows.append(rec)
        chunk=[x['vector'] for x in of['oft_canonical_action_chunk']]
        q=OFTChunkQueue(mode='sequential_k5',max_age_sec=1.2);q.enqueue(chunk,'oft-frame0',20.)
        fk=fk_engine.compute(dict(zip(CANONICAL_JOINTS,of['raw_joint_position'][:6])));sup=_supervisor();grip=ClosednessHysteresis();limiter=CanonicalActionRateLimiter()
        for i in range(5):
            qa=q.pop(20.+.2*i);limited=limiter.limit(qa.action)
            safety=sup.inspect(action_id=f'{qa.chunk_id}-k{qa.chunk_index}',action=limited.limited_action,source_monotonic=qa.inference_monotonic,now_monotonic=20.+.2*i,tcp_ok=True)
            rec=_base_record('oft',of,fk,qa.action,limited,qa.chunk_id,qa.chunk_index,20.+.2*i,safety,grip);rec['action_age_sec']=qa.action_age_sec;log.append(rec);rows.append(rec)
    return {'status':'COMPLETED_OFFLINE_COMMAND_FREE_20_PROFILE','records':len(rows),'openvla_records':1,'oft_records':5,
            'oft_chunk_indices':[r['chunk_index'] for r in rows if r['model']=='oft'],
            'safety_accepted_records':sum(r['safety_result']['accepted'] for r in rows),
            'safety_rejected_records':sum(not r['safety_result']['accepted'] for r in rows),
            'safety_rejection_reasons':[r['safety_result']['hold_reason'] for r in rows if not r['safety_result']['accepted']],
            'workspace_accepted_records':sum(r['workspace_result']['accepted'] for r in rows),
            'workspace_rejected_records':sum(not r['workspace_result']['accepted'] for r in rows),
            'limiter_modified_records':sum(any((r['limiter']['translation_speed_clipped'],r['limiter']['rotation_speed_clipped'],r['limiter']['translation_acceleration_clipped'],r['limiter']['rotation_acceleration_clipped'])) for r in rows),
            'max_translation_change_m':max(r['limiter']['translation_change_m'] for r in rows),
            'max_rotation_change_rad':max(r['limiter']['rotation_change_rad'] for r in rows),
            'motion_profile':{'task_translation_velocity_mm_s':20.0,'task_rotation_velocity_deg_s':20.0,
              'task_translation_acceleration_mm_s2':20.0,'task_rotation_acceleration_deg_s2':20.0,
              'joint_velocity_deg_s':20.0,'joint_acceleration_deg_s2':20.0,'control_rate_hz':5.0,
              'translation_step_ceiling_m':.004,'rotation_step_ceiling_rad':math.radians(4.0)},
            'all_executed_action_null':all(r['executed_action'] is None for r in rows),
            'all_robot_delivered_command_null':all(r['robot_delivered_command'] is None for r in rows),
            'all_command_issued_false':all(not r['command_issued'] for r in rows),
            'motion_readiness':False,'output':str(output)}


def main():
    p=argparse.ArgumentParser();p.add_argument('--openvla',required=True);p.add_argument('--oft',required=True);p.add_argument('--urdf',required=True);p.add_argument('--output',required=True);p.add_argument('--summary',required=True);a=p.parse_args()
    result=run(a.openvla,a.oft,a.urdf,a.output)
    with open(a.summary,'x',encoding='utf-8') as f:json.dump(result,f,indent=2)
    print(json.dumps(result,indent=2))
if __name__=='__main__':main()
