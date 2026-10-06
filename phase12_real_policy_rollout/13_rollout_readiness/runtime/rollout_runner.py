"""Recorded prediction pipeline, strict abort, null delivery only."""
import argparse,json,math,sys,hashlib
from pathlib import Path
import yaml
ROOT=Path(__file__).resolve().parents[1]
PHASE=ROOT.parent
sys.path[:0]=[str(PHASE/'02_safety'),str(PHASE/'03_shadow_mode'),str(ROOT/'metrics')]
from prediction_shadow_runtime import actions_for,abc
from safety_pipeline import SafetyPipeline,RuntimeState
from canonical_action import CanonicalAction
from hard_command_gate import HardCommandGate,UNSAFE_FLAGS
from command_sinks import MockDoosanCommandSink
from integrated_logger import FsyncJsonlLogger
from oft_timing_contract import target_step,target_time_sec,inference_time_sec
from rollout_metrics import metrics
from rollout_state_machine import RolloutStateMachine

def finite_json(value):
    if isinstance(value,float) and not math.isfinite(value):return None
    if isinstance(value,dict):return {k:finite_json(v) for k,v in value.items()}
    if isinstance(value,(list,tuple)):return [finite_json(v) for v in value]
    return value

def run(model, input_path, output_path, *, initial_open_acknowledged=False, audit_all=False, faults=None, protocol='full_task'):
    config_path=ROOT/'configs'/f'rollout_{model}.yaml'
    config=yaml.safe_load(config_path.read_text())
    protocol_path=ROOT/'configs'/f'{protocol}.yaml'
    limits=yaml.safe_load(protocol_path.read_text())
    if config['command_mode']!='disabled' or config['motion_enabled'] is not False or config['explicit_motion_approval'] is not False:
        raise ValueError('unsafe_config')
    if limits['command_mode']!='disabled' or limits['motion_enabled'] is not False:raise ValueError('unsafe_protocol')
    if model=='oft' and config['chunk_execution_mode']!='sequential_k5':raise ValueError('invalid_oft_contract')
    config_hash=hashlib.sha256(config_path.read_bytes()).hexdigest()
    safety_hash=hashlib.sha256((PHASE/'01_configs/safety_limits.yaml').read_bytes()).hexdigest()
    rows=[json.loads(l) for l in Path(input_path).read_text().splitlines() if l.strip()]
    by_frame={int(r['frame_id']):r for r in rows}
    pipeline=SafetyPipeline(model,operator_confirmed_initial_open=initial_open_acknowledged)
    gate=HardCommandGate(dict(mode='command_disabled_pre_motion',include_action_adapter=False,include_doosan_bridge=False,**{k:False for k in UNSAFE_FLAGS}))
    sink=MockDoosanCommandSink();machine=RolloutStateMachine();records=[];faults=faults or {}
    with FsyncJsonlLogger(output_path) as log:
        for row in rows:
            if not row.get('valid') or row.get('fixture_smoke_test_only'):continue
            vectors=actions_for(row,model)
            if not vectors:continue
            frame=int(row['frame_id']);base=inference_time_sec(frame)
            if model=='oft' and len(vectors)!=5:
                machine.abort('oft_partial_chunk');break
            for k,vector in enumerate(vectors):
                step=target_step(frame,k);now=target_time_sec(frame,k)
                mapped=by_frame.get(step)
                if mapped is None:
                    machine.abort('missing_target_observation');break
                position=mapped.get('raw_ee_position');phase=mapped.get('phase','unknown')
                reason=[];decision=None;action=None
                try:
                    if row.get('instruction')!=config['instruction']:raise ValueError('instruction_mismatch')
                    if not position:raise ValueError('tcp_unavailable')
                    if faults.get('manual_abort'):raise ValueError('manual_abort')
                    if faults.get('oft_underrun') and model=='oft':raise ValueError('oft_underrun')
                    action=CanonicalAction.from_vector(vector,timestamp_monotonic=base,sequence_id=f'{row["episode_id"]}-{frame}-k{k}',source_model=model,chunk_index=k,chunk_size=len(vectors))
                    state=RuntimeState(now,tuple(position),abc(mapped.get('raw_ee_rotation')),phase,
                        camera_ok=not faults.get('camera_stale'),joint_state_ok=not faults.get('joint_state_stale'),
                        tcp_ok=not faults.get('tcp_stale'),model_ok=not faults.get('model_timeout'),logger_ok=not faults.get('logger_failure'))
                    decision=pipeline.inspect(action,state);reason=list(decision.reason)
                    if protocol=='minimum_motion' and (math.dist(vector[:3],[0,0,0])>limits['translation_step_m'] or any(vector[3:6]) or vector[6]!=0):
                        reason.append('minimum_motion_contract_violation')
                    if not limits.get('gripper_enabled',True) and vector[6]>=.7:reason.append('protocol_gripper_disabled')
                except (ValueError,RuntimeError) as e:reason=[str(e)]
                accepted=not reason and decision is not None
                candidate=dict(decision.candidate) if decision else {'technical_blockers':reason,'technical_valid':False}
                gated=gate.audit_candidate(candidate)
                receipt=sink.submit(action.as_dict() if action else {},{'accepted':accepted,'reason':reason})
                record=dict(timestamp=now,clock_domain='RECORDED_RELATIVE_SYNTHETIC_MONOTONIC',
                    frame_id=frame,target_step=step,target_time=now,image_timestamp=mapped.get('timestamp_camera'),
                    image_clock_domain=mapped.get('camera_clock_domain'),image_sha256=mapped.get('raw_image_sha256'),
                    joint_state=mapped.get('raw_joint_position'),tcp_pose=position,tcp_source=mapped.get('pose_source'),
                    instruction=row.get('instruction'),raw_model_output=row.get('oft_raw_action_chunk') if model=='oft' else row.get('openvla_raw_action'),
                    canonical_action=action.as_dict() if action else None,filtered_action=decision.filtered_action if decision else None,
                    translation_norm=math.sqrt(sum(float(x)**2 for x in vector[:3])),rotation_norm=math.sqrt(sum(float(x)**2 for x in vector[3:6])),
                    gripper_closedness=vector[6],chunk_id=f'{row["episode_id"]}-{frame}' if model=='oft' else None,chunk_index=k,
                    safety_accepted=accepted,safety_blockers=reason,hold_required=bool(reason),
                    command_requested=accepted,command_issued=False,command_acknowledged=False,
                    executed_action=None,robot_delivered_command=None,phase=phase,episode_id=row['episode_id'],condition=row.get('condition_id','recorded'),
                    latency=row.get('inference_latency_oft' if model=='oft' else 'inference_latency_openvla'),gate=gated['hard_gate'],mock_receipt=receipt,
                    model=model,checkpoint=config['checkpoint'],model_config_hash=config_hash,safety_config_hash=safety_hash,protocol=protocol,
                    **{key:mapped.get(key) for key in ('matched_pair_id','sim_observation_id','real_observation_id','observation_gap_score','action_gap_translation','action_gap_rotation','action_gap_gripper')})
                record=finite_json(record)
                log.append(record);records.append(record)
                if reason:
                    machine.abort('|'.join(reason))
                    if not audit_all:break
                if len(records)>=limits['maximum_actions']:break
            if len(records)>=limits['maximum_actions']:break
            if machine.aborted and not audit_all:break
    result=metrics(records)
    result.update(aborted=machine.aborted,abort_reason=machine.reason,mode='OFFLINE_AUDIT_ALL' if audit_all else 'STRICT_ABORT',
        real_command_integration='NOT_IMPLEMENTED',MOTION_READY=False,input_sha256=hashlib.sha256(Path(input_path).read_bytes()).hexdigest())
    Path(str(output_path)+'.metrics.json').write_text(json.dumps(result,indent=2)+'\n')
    return result

if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('--model',choices=['openvla','oft'],required=True)
    parser.add_argument('--input',type=Path,required=True);parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--initial-open-acknowledged',action='store_true');parser.add_argument('--audit-all',action='store_true')
    parser.add_argument('--protocol',choices=['minimum_motion','short_horizon','full_task'],default='full_task')
    args=parser.parse_args();print(json.dumps(run(args.model,args.input,args.output,initial_open_acknowledged=args.initial_open_acknowledged,audit_all=args.audit_all,protocol=args.protocol),indent=2))
