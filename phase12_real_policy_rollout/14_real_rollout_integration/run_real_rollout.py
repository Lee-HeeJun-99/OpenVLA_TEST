"""Same gated controller used with fake, recorded, and future live observations."""
import argparse
import hashlib
import json
import math
from pathlib import Path
import sys
import time
import threading

ROOT=Path(__file__).resolve().parent
PHASE=ROOT.parent
sys.path[:0]=[str(PHASE/'02_safety'),str(PHASE/'03_shadow_mode'),str(PHASE/'13_rollout_readiness/runtime'),str(PHASE/'15_valid_invalid_rollout_policy')]
from rollout_trial_classifier import RolloutTrial
from canonical_action import CanonicalAction
from safety_pipeline import SafetyPipeline,RuntimeState
from integrated_logger import FsyncJsonlLogger
from command_sinks import NullCommandSink
from real_sink import Authorization,RealDoosanCommandSink,AbortBoundary,RosServiceTransport
from pre_real_rollout_check import preflight
from stage_gate import require_stage,save_stage


class Controller:
    def __init__(self,model,protocol,sink,logger,*,dry_run=True,initial_open=False,trial=None):
        self.pipeline=SafetyPipeline(model,operator_confirmed_initial_open=initial_open)
        self.model=model;self.protocol=protocol;self.sink=sink;self.log=logger;self.dry_run=dry_run
        self.abort_boundary=AbortBoundary(sink) if not dry_run else None
        self.aborted=False;self.count=0;self.rows=[]
        self.abort_lock=threading.Lock();self.abort_reason=None
        self.scheduler=None
        self.trial=trial
        if trial is not None:sink.trial_guard=trial

    def abort(self,reason):
        with self.abort_lock:
            if self.aborted:return {'state':'ABORTED','reason':self.abort_reason,'command_issued':False}
            self.aborted=True;self.abort_reason=reason
            if self.trial:self.trial.invalidate(reason)
            self.sink.inhibit()
        if self.scheduler:self.scheduler.abort()
        result={'state':'ABORTED','reason':reason,'command_issued':False}
        if not self.dry_run:result=self.abort_boundary.abort(reason)
        # Stop path must still run when logger is broken; preserve error for caller.
        try:self.log.append({'event':'ABORT','abort':result,'dry_run':self.dry_run})
        except Exception:pass
        return result

    def step(self,vector,obs,*,inference_time,chunk_id,chunk_index):
        if self.aborted:raise RuntimeError('session_aborted')
        if self.trial:
            obs=dict(obs)
            if obs.get('episode_id') is None:obs['episode_id']=self.trial.trial_id
        if obs.get('live_execution_clock'):
            obs=dict(obs);obs['receive_monotonic']=time.monotonic()
        now=obs['receive_monotonic'];reason=[];candidate={};decision=None
        if self.trial:self.trial.prediction({'raw_action':list(vector),'raw_model_output':obs.get('raw_model_output'),
                                            'chunk_id':chunk_id,'chunk_index':chunk_index,'observation':obs})
        if self.trial and not self.trial.observe(obs):return self.abort(self.trial.summary()['invalid_reason'])
        try:
            if not obs.get('robot_state_ok'):raise ValueError('unexpected_robot_state')
            if obs.get('manual_abort'):raise ValueError('manual_abort')
            action=CanonicalAction.from_vector(vector,timestamp_monotonic=inference_time,
                sequence_id=f'{chunk_id}-k{chunk_index}',source_model=self.model,
                chunk_index=chunk_index,chunk_size=5 if self.model=='oft' else 1)
            state=RuntimeState(now,tuple(obs['tcp_m_abc'][:3]),tuple(obs['tcp_m_abc'][3:]),obs['phase'],
                camera_ok=obs['camera_ok'],joint_state_ok=obs['joint_state_ok'],tcp_ok=obs['tcp_ok'],
                model_ok=obs['model_ok'],communication_ok=obs.get('communication_ok',False),logger_ok=True)
            decision=self.pipeline.inspect(action,state);candidate=dict(decision.candidate);reason=list(decision.reason)
            if self.protocol=='minimum_motion' and (math.dist(vector[:3],[0,0,0])>.001 or any(vector[3:6]) or vector[6]!=0):reason.append('minimum_motion_protocol')
            if self.protocol!='full_task' and vector[6]>=.7:reason.append('protocol_gripper_disabled')
        except Exception as exc:reason.append(str(exc))
        record={'event':'ACTION','command_requested':not reason,'command_issued':False,'command_acknowledged':False,
            'executed_action':None,'robot_delivered_command':None,'observation':obs,'raw_model_output':obs.get('raw_model_output'),
            'canonical_action':list(vector),'filtered_action':candidate.get('limited_canonical_action'),
            'safety_blockers':reason,'chunk_id':chunk_id,'chunk_index':chunk_index,
            'target_step':obs.get('inference_frame',0)+chunk_index,
            'target_time':(obs.get('inference_frame',0)+chunk_index)*.2,
            'phase':obs['phase'],'tcp_source':obs.get('tcp_source'),'dry_run':self.dry_run,
            **{key:obs.get(key) for key in ('episode_id','condition','matched_pair_id','sim_observation_id','real_observation_id','observation_gap_score','action_gap_translation','action_gap_rotation','action_gap_gripper')}}
        from rollout_runner import finite_json
        record=finite_json(record)
        if self.trial and reason:self.trial.invalidate('|'.join(reason))
        try:self.log.append(record)
        except Exception:
            self.abort('logger_failure');raise
        self.rows.append(record);self.count+=1
        if reason:return self.abort('|'.join(reason))
        if self.trial and not self.trial.command_allowed():return self.abort(self.trial.summary()['invalid_reason'] or 'trial_command_inhibited')
        if self.dry_run:
            record['sink_receipt']=NullCommandSink().submit(action.as_dict(),decision.as_dict())
            return record
        receipt=self.sink.send_pose(candidate['target_pose_candidate_mm_zyz_deg'])
        # Send/ACK/completion each persisted by their actual timestamp, not fabricated.
        record['command_receipt']=receipt
        record['command_issued']=receipt.get('sent_at') is not None
        record['command_acknowledged']=receipt.get('ack_at') is not None
        try:self.log.append({**record,'event':'COMMAND_RESULT'})
        except Exception:self.abort('logger_failure');raise
        if receipt['state']!='completed':return self.abort(receipt.get('failure_reason','command_failed'))
        grip=candidate.get('gripper',{})
        cmd=grip.get('candidate_command')
        if self.protocol=='full_task' and cmd in ('OPEN','CLOSE','CLOSED','COMMAND_OPEN','COMMAND_CLOSED'):
            g=self.sink.send_gripper(cmd in ('CLOSE','CLOSED','COMMAND_CLOSED'))
            if g['state'] not in ('completed','suppressed'):return self.abort('gripper_command_failed')
            from open_loop_gripper_supervisor import GripperRuntimeContext
            supervisor=self.pipeline.runtime.gripper
            transition=supervisor.resolve_with_context(vector[6],phase=obs['phase'],
                context=GripperRuntimeContext(True,True,True,True,True,candidate_executed=True))
            try:self.log.append({'event':'GRIPPER_RESULT','receipt':g,
                'command_knowledge_after':transition.command_knowledge_after,'measured_gripper_state':None})
            except Exception:self.abort('logger_failure');raise
        return record


def main():
    import yaml
    parser=argparse.ArgumentParser()
    parser.add_argument('--model',choices=['openvla','oft'],required=True)
    parser.add_argument('--protocol',choices=['minimum_motion','short_horizon','full_task'],required=True)
    parser.add_argument('--dry-run',action='store_true')
    parser.add_argument('--live',action='store_true',help='subscriber inputs, no recorded replay')
    parser.add_argument('--prediction-only-flange',action='store_true',help='vision-only diagnostic Shadow with unverified flange context; dry/live OpenVLA only, never motion')
    parser.add_argument('--recorded-input',type=Path)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--preflight-evidence',type=Path)
    parser.add_argument('--approval',type=Path)
    parser.add_argument('--session-results',type=Path)
    parser.add_argument('--model-config',type=Path,help='operator-approved config copy; defaults remain disabled')
    parser.add_argument('--task-outcome-evidence',type=Path,help='post-trial explicit task_success boolean; protocol ACK/COMPLETE alone is not task evidence')
    args=parser.parse_args()
    if args.output.exists():raise FileExistsError('preserve_existing_log')
    if args.prediction_only_flange:
        from flange_prediction_shadow import run as run_flange_shadow
        return run_flange_shadow(args)
    if args.recorded_input:
        if args.live:raise ValueError('live_and_recorded_mutually_exclusive')
        if not args.dry_run:raise PermissionError('recorded_input_never_commands_hardware')
        from rollout_runner import run
        print(json.dumps(run(args.model,args.recorded_input,args.output,initial_open_acknowledged=False,protocol=args.protocol),indent=2))
        return
    from live_observation import LiveObservation
    model=yaml.safe_load((args.model_config or ROOT/'configs'/f'real_{args.model}.yaml').read_text())
    if model.get('model')!=args.model:raise ValueError('model_config_identity_mismatch')
    protocol_config=yaml.safe_load((ROOT/'configs'/f'{args.protocol}.yaml').read_text())
    evidence=json.loads(args.preflight_evidence.read_text()) if args.preflight_evidence else {}
    readiness=preflight(evidence,dry_run=args.dry_run)
    approval=json.loads(args.approval.read_text()) if args.approval else {}
    identity={'model':args.model,'checkpoint':model['checkpoint'],'protocol_config_sha256':hashlib.sha256((ROOT/'configs/hardware_interface.yaml').read_bytes()).hexdigest()}
    if not args.dry_run:
        if evidence.get('source')=='FAKE_TEST_GRAPH':raise PermissionError('fake_evidence_never_authorizes_hardware')
        if model.get('motion_enabled') is not True or model.get('explicit_motion_approval') is not True:
            raise PermissionError('real_config_default_disabled')
        if args.model=='oft' and evidence.get('model_behavior_review_passed') is not True:
            raise PermissionError('oft_model_behavior_review_pending')
        if not readiness['MOTION_READY']:raise PermissionError('hardware_preflight_incomplete')
        if not (approval.get('explicit_motion_approval') is True and approval.get('protocol')==args.protocol and approval.get('model')==args.model and approval.get('operator') and 0<=time.time()-approval.get('approved_wall_time',0)<=60):raise PermissionError('explicit_stage_approval_required')
        if not args.session_results:raise PermissionError('session_results_required')
        require_stage(args.protocol,args.session_results,identity)
    obs=LiveObservation(model['server_url'],model['checkpoint'],model['variant'],evidence)
    # Do not authorize from a checklist alone: require actual observations first.
    if not args.dry_run:
        try:
            _,live=obs.snapshot()
            if not all(live[k] for k in ('camera_ok','joint_state_ok','tcp_ok','robot_state_ok','servo_mode_ok','protective_stop_ok')):
                raise PermissionError('live_observation_preflight_failed')
            if live.get('hardware_state',{}).get('authority') is not True:
                raise PermissionError('hardware_authority_unconfirmed')
            if live.get('hardware_state',{}).get('motion_state')!=1:
                raise PermissionError('hardware_not_stationary_at_preflight')
            graph=dict(obs.node.get_service_names_and_types())
            from real_sink import SERVICES
            if any('dsr_msgs2/srv/'+typ not in graph.get(name,[]) for name,typ in SERVICES.values()):
                raise PermissionError('live_command_service_contract_missing')
        except Exception:
            obs.close();raise
    auth=Authorization(not args.dry_run,not args.dry_run,readiness['MOTION_READY'],'MOTION_ENABLED' if not args.dry_run else 'COMMAND_DISABLED')
    gripper=yaml.safe_load((ROOT/'configs/gripper_hardware.yaml').read_text())
    gripper['polarity_confirmed']=evidence.get('gripper_polarity_confirmed') is True
    if evidence.get('gripper_abort_value_confirmed') is True:
        gripper['abort_value']=evidence.get('gripper_abort_value')
    if not args.dry_run and args.protocol=='full_task' and gripper.get('abort_value') not in (0,1):
        obs.close();raise PermissionError('hardware_gripper_abort_value_unconfirmed')
    sink=RealDoosanCommandSink(lambda:RosServiceTransport(obs.node),auth,gripper_config=gripper)
    limit=protocol_config['maximum_actions']
    passed=False
    watchdog=None
    scheduler=None
    try:
        with FsyncJsonlLogger(args.output) as logger:
            from concurrent_logger import ConcurrentLogger
            fault=3 if args.dry_run and evidence.get('source')=='FAKE_TEST_GRAPH' and evidence.get('fake_fault')=='logger' else None
            logger=ConcurrentLogger(logger,fail_after=fault)
            trial=RolloutTrial(args.output.parent.name,evaluation_scope='prediction_only' if args.dry_run else 'task')
            js=obs.joint_readiness.status(time.monotonic())
            logger.append({'event':'TRIAL_STARTUP_READINESS','scope':'PRE_TRIAL_NOT_PERFORMANCE',
                           'readiness':js,'startup_warmup_events':list(obs.joint_readiness.events)})
            trial.start({**js,'first_fresh_after_rearm':obs.joint_readiness.first_fresh is not None and obs.joint_readiness.first_fresh>=obs.joint_readiness.started,
                         'warmup_seconds':js['clean_window_s']})
            controller=Controller(args.model,args.protocol,sink,logger,dry_run=args.dry_run,initial_open=evidence.get('gripper_initial_confirmed') is True,trial=trial)
            from hardware_watchdog import HardwareWatchdog
            from oft_action_scheduler import ActionScheduler
            from task_phase import TaskPhase
            phase_machine=None
            if args.protocol=='full_task':
                phase_config=yaml.safe_load((ROOT/'configs/task_phase.yaml').read_text())
                if phase_config['grasp_pose_m'] is None or phase_config['pregrasp_z_m'] is None:
                    raise PermissionError('approved_task_geometry_required')
                phase_machine=TaskPhase(phase_config)
            def watch_snapshot():
                _,current=obs.snapshot()
                return {**current,
                    'model_ok':obs.check_health(),'logger_ok':logger.healthy,'command_ack_ok':not sink.aborted,
                    'manual_abort_clear':not controller.aborted}
            watchdog=HardwareWatchdog(watch_snapshot,controller.abort).start()
            if watchdog.check():raise PermissionError('initial_watchdog_fault')
            scheduler=ActionScheduler(lambda payload:controller.step(**payload))
            controller.scheduler=scheduler
            started=time.monotonic();seq=0
            while controller.count<limit and not controller.aborted:
                try:image,observation=obs.snapshot()
                except Exception as exc:
                    controller.abort('observation_failure:'+str(exc));break
                base=time.monotonic()
                if args.protocol=='minimum_motion':
                    delta=protocol_config['deterministic_delta_m']
                    if len(delta)!=3 or math.dist(delta,[0,0,0])>protocol_config['translation_step_m']:
                        controller.abort('minimum_motion_config_invalid');break
                    actions=[list(delta)+[0,0,0,0]]
                else:
                    try:actions=obs.predict(image,args.model,selected_frame_receive=observation.get('camera_receive_monotonic'),selected_frame_source=observation.get('image_source_timestamp'))
                    except Exception as exc:
                        controller.abort('model_inference_failure:'+str(exc));break
                if len(actions)!=(5 if args.model=='oft' and args.protocol!='minimum_motion' else 1):
                    controller.abort('partial_chunk');break
                for k,vector in enumerate(actions):
                    scheduled=base+k*.2
                    if time.monotonic()<scheduled:time.sleep(scheduled-time.monotonic())
                    try:_,observation=obs.snapshot()
                    except Exception as exc:
                        controller.abort('observation_failure:'+str(exc));break
                    observation.update(getattr(obs,'last_prediction',{}))
                    if observation.get('selected_prediction_frame_age') is not None:
                        observation['camera']['selected_age']=observation['selected_prediction_frame_age']
                    observation['inference_frame']=seq
                    observation['live_execution_clock']=True
                    if phase_machine:
                        closed=controller.pipeline.runtime.gripper.state.value=='COMMAND_CLOSED'
                        phase=phase_machine.update(observation['tcp_m_abc'],command_closed=closed,
                            safety_ok=not controller.aborted,elapsed=time.monotonic()-started)
                        if phase=='COMPLETE':break
                        observation['phase']={'APPROACH':'alignment','DESCEND':'descent_to_grasp',
                            'GRASP_CLOSE':'grasp_close','LIFT':'lift','COMPLETE':'final_hold','ABORT':'abort'}[phase]
                    payload=dict(vector=vector,obs=observation,inference_time=base,chunk_id=f'live-{seq}',chunk_index=k)
                    row=scheduler.dispatch(seq,k,base-seq*.2,payload)
                    if row['status']=='NO_OVERLAPPING_MOTION':controller.abort('cadence_unresolved_previous_command')
                    if phase_machine and phase_machine.phase=='GRASP_CLOSE' and vector[6]>=.7:
                        while scheduler.pending and not scheduler.pending.done() and not controller.aborted:
                            if time.monotonic()-started>limit*.2:
                                controller.abort('protocol_duration_timeout');break
                            time.sleep(.02)
                        logger.append({'event':'CHUNK_INTERRUPTED','reason':'GRIPPER_ACK_BARRIER',
                            'chunk_id':f'live-{seq}','remaining_indices':list(range(k+1,len(actions))),
                            'command_issued':False})
                        break
                    if controller.aborted:break
                seq+=len(actions)
                if phase_machine and phase_machine.phase=='COMPLETE':break
                cadence=base+(1. if args.model=='oft' else .2)
                if time.monotonic()<cadence:time.sleep(cadence-time.monotonic())
                if time.monotonic()-started>limit*.2:
                    if controller.count<limit:controller.abort('protocol_duration_timeout')
                    break
            if scheduler.pending:scheduler.pending.result(timeout=sink.ack_timeout+1)
            passed=not controller.aborted and (phase_machine.phase=='COMPLETE' if phase_machine else controller.count==limit)
    except KeyboardInterrupt:
        if 'controller' in locals():controller.abort('manual_abort')
    except Exception as exc:
        if 'controller' in locals() and not controller.aborted:controller.abort(str(exc))
        raise
    finally:
        if watchdog:watchdog.close()
        if scheduler:scheduler.close()
        obs.close()
        if 'trial' in locals():
            if trial.status is None:
                try:outcome=json.loads(args.task_outcome_evidence.read_text()) if args.task_outcome_evidence else {}
                except Exception as exc:
                    trial.invalidate('TASK_OUTCOME_EVIDENCE_ERROR:'+str(exc));outcome={}
                physical_evidence=args.dry_run or (outcome.get('operator') and outcome.get('physical_task_evidence') is True and
                    trial.started_wall_time<=outcome.get('observed_wall_time',0)<=time.time())
                if outcome.get('trial_id')==trial.trial_id and isinstance(outcome.get('task_success'),bool) and outcome.get('evaluation_scope')==trial.evaluation_scope and physical_evidence:
                    trial.finish(outcome['task_success'],persist=lambda summary:trial.save(args.output.parent/'trial_result',summary=summary))
                else:trial.invalidate('TASK_OUTCOME_UNVERIFIED',category='INVALID_TASK_OUTCOME_UNVERIFIED')
            if trial.status=='INVALID':trial.save(args.output.parent/'trial_result')
            if not args.dry_run and trial.status=='INVALID':passed=False
        if args.session_results:
            from integration_metrics import collect
            save_stage(args.protocol,args.session_results,identity,passed=passed,dry_run=args.dry_run,
                details={'command_events':sink.events,'protocol_completed':passed,'task_success':None,
                    'metrics':collect(controller.rows,sink.events) if 'controller' in locals() else {},
                    'watchdog_fault':watchdog.fault if watchdog else None,
                    'abort_reason':controller.abort_reason if 'controller' in locals() else None,
                    'dispatch_timeline':scheduler.rows if scheduler else [],
                    'physical_grasp_success':'UNVERIFIED'})

if __name__=='__main__':main()
