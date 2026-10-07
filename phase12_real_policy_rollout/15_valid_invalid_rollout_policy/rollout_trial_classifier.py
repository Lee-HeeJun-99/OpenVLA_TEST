"""Trial validity and task outcome are independent; no robot/ROS dependency."""
import copy
import json
import math
import os
import threading
import time
from pathlib import Path

STATUSES = frozenset(('SUCCESS', 'FAILURE', 'INVALID'))

def json_safe(value):
    if isinstance(value,float) and not math.isfinite(value):return {'nonfinite':repr(value)}
    if isinstance(value,dict):return {k:json_safe(v) for k,v in value.items()}
    if isinstance(value,(list,tuple)):return [json_safe(v) for v in value]
    return value


def invalid_category(reason):
    text=str(reason).lower()
    if 'task_outcome' in text:return 'INVALID_TASK_OUTCOME_UNVERIFIED'
    if any(x in text for x in ('joint_state','jointstate','missing_joint','timestamp_regression','timestamp_duplicate')):return 'INVALID_JOINTSTATE_GAP'
    if any(x in text for x in ('camera','frame','encoding','resolution')):return 'INVALID_CAMERA_STALE'
    if any(x in text for x in ('logger','disk','write_failure')):return 'INVALID_LOGGER_ERROR'
    if any(x in text for x in ('model_input','input_construction','canonical_action','nonfinite','nan','inf action')):return 'INVALID_MODEL_INPUT'
    if any(x in text for x in ('model','inference','network','http','connection','partial_chunk')):return 'INVALID_MODEL_REQUEST'
    if any(x in text for x in ('manual_abort','operator_abort')):return 'INVALID_OPERATOR_ABORT'
    if any(x in text for x in ('limit','workspace','gripper','safety_rejection','minimum_motion_protocol')):return 'INVALID_SAFETY_REJECTION'
    return 'INVALID_RUNTIME_ERROR'


class RolloutTrial:
    def __init__(self,trial_id,*,clock=time.monotonic,evaluation_scope='task'):
        self.trial_id=trial_id;self.clock=clock;self.evaluation_scope=evaluation_scope
        self.lock=threading.RLock();self.started=None;self.started_wall_time=None;self.ended=None;self.status=None
        self.runtime_valid=None;self.task_success=None;self.invalid_event=None
        self.commands_inhibited=True;self.command_count=0;self.prediction_count=0
        self.counts={'jointstate_gap_count':0,'camera_failure_count':0,'model_failure_count':0,'safety_rejection_count':0}
        self.last_valid_observation=None;self.last_prediction=None;self.task_phase_end=None
        self.runtime_events=[];self.observations=[];self.predictions=[];self._last_joint=None

    def start(self,readiness):
        with self.lock:
            if self.started is not None or self.status is not None:raise RuntimeError('trial_cannot_restart')
            if not (readiness.get('phase')=='RUNTIME_READY' and not readiness.get('fault') and
                    readiness.get('first_fresh_after_rearm') is True and readiness.get('warmup_seconds',0)>=10):
                raise PermissionError('new_trial_requires_fresh_sample_and_warmup')
            self.started=self.clock();self.started_wall_time=time.time();self.runtime_valid=True;self.commands_inhibited=False

    def invalidate(self,reason,*,timestamp=None,category=None):
        with self.lock:
            if self.status is not None:return self.summary()
            self.commands_inhibited=True;self.status='INVALID';self.runtime_valid=False;self.task_success=None
            self.ended=self.clock();category=category or invalid_category(reason)
            self.invalid_event=dict(invalid_timestamp=self.ended if timestamp is None else timestamp,
                invalid_wall_time=time.time(),clock_domain='HOST_MONOTONIC',
                invalid_reason=str(reason),invalid_category=category,
                last_valid_observation=copy.deepcopy(self.last_valid_observation),last_prediction=copy.deepcopy(self.last_prediction),
                command_count_before_invalid=self.command_count)
            self.runtime_events.append(copy.deepcopy(self.invalid_event))
            key={'INVALID_JOINTSTATE_GAP':'jointstate_gap_count','INVALID_CAMERA_STALE':'camera_failure_count',
                 'INVALID_MODEL_INPUT':'model_failure_count','INVALID_MODEL_REQUEST':'model_failure_count',
                 'INVALID_SAFETY_REJECTION':'safety_rejection_count'}.get(category)
            if key and (key!='jointstate_gap_count' or any(token in str(reason).lower() for token in ('gap','receive_timeout','latest_age'))):self.counts[key]+=1
            return self.summary()

    def command_allowed(self):
        with self.lock:return self.started is not None and self.status is None and not self.commands_inhibited

    def command_dispatched(self):
        with self.lock:
            if not self.command_allowed():raise PermissionError('trial_command_inhibited')
            self.command_count+=1

    def observe(self,observation):
        with self.lock:
            if self.status is not None:return False
            if self.started is None:raise RuntimeError('observation_before_trial_start')
            obs=copy.deepcopy(observation);self.observations.append(obs)
            self.task_phase_end=obs.get('phase',self.task_phase_end)
            joint=obs.get('jointstate')
            if joint:
                reason=None
                values=list(joint.get('position',[]))+list(joint.get('velocity',[]))
                if set(joint.get('names',[]))!={f'joint_{i}' for i in range(1,7)} or len(joint.get('names',[]))!=6:reason='JOINTSTATE_MISSING_JOINTS'
                elif len(joint.get('position',[]))!=6 or len(joint.get('velocity',[]))!=6 or not all(isinstance(v,(int,float)) and math.isfinite(v) for v in values):reason='JOINTSTATE_NONFINITE_OR_INVALID_VALUES'
                elif not all(isinstance(joint.get(k),(int,float)) and math.isfinite(joint[k]) for k in ('latest_age','source')):reason='JOINTSTATE_NONFINITE_TIMESTAMP'
                elif joint['latest_age']<0:reason='JOINTSTATE_INVALID_RECEIVE_CLOCK'
                elif any(joint.get(k) is not None and (not isinstance(joint[k],(int,float)) or not math.isfinite(joint[k])) for k in ('source_gap','receive_gap')):reason='JOINTSTATE_NONFINITE_GAP'
                elif joint.get('latest_age',math.inf)>=.5:reason='JOINTSTATE_LATEST_AGE'
                elif joint.get('source_gap') is not None and joint['source_gap']>=.1:reason='JOINTSTATE_SOURCE_GAP'
                elif joint.get('receive_gap') is not None and joint['receive_gap']>=.1:reason='JOINTSTATE_RECEIVE_GAP'
                elif joint.get('source_gap') is not None and joint['source_gap']<=0:reason='JOINTSTATE_NONINCREASING_TIMESTAMP'
                elif joint.get('receive_gap') is not None and joint['receive_gap']<=0:reason='JOINTSTATE_NONINCREASING_RECEIVE'
                # Only new sensor samples update the timestamp comparison; polling the
                # same sample is not itself a duplicate publisher timestamp.
                sample_id=joint.get('sample_id')
                if not reason and self._last_joint and sample_id is not None and sample_id!=self._last_joint.get('sample_id'):
                    if joint.get('source',math.inf)<=self._last_joint.get('source',-math.inf):reason='JOINTSTATE_NONINCREASING_TIMESTAMP'
                if reason:self.invalidate(reason);return False
                self._last_joint=joint
            camera=obs.get('camera')
            if camera:
                reason=None
                if not isinstance(camera.get('selected_age'),(int,float)) or not math.isfinite(camera['selected_age']) or camera['selected_age']<0:reason='CAMERA_INVALID_TIMESTAMP'
                elif camera.get('selected_age',math.inf)>=.5:reason='CAMERA_SELECTED_FRAME_STALE'
                elif camera.get('fresh_frame_timeout'):reason='CAMERA_FRESH_FRAME_TIMEOUT'
                elif camera.get('encoding') not in ('bgra8','bgr8','rgb8','rgba8'):reason='CAMERA_INVALID_ENCODING'
                elif camera.get('resolution_valid') is not True:reason='CAMERA_INVALID_RESOLUTION'
                elif camera.get('stream_alive') is not True:reason='CAMERA_STREAM_LOSS'
                if reason:self.invalidate(reason);return False
            for field,reason in [('joint_state_ok','JOINTSTATE_GATE'),('camera_ok','CAMERA_GATE'),('model_ok','MODEL_REQUEST'),
                                 ('tcp_ok','TCP_RUNTIME_ERROR'),('robot_state_ok','HARDWARE_RUNTIME_ERROR'),('logger_ok','LOGGER_ERROR')]:
                if obs.get(field) is False:self.invalidate(reason);return False
            self.last_valid_observation=obs;return True

    def prediction(self,raw,*,accepted=True,blockers=None):
        with self.lock:
            if self.status is not None:return False
            self.prediction_count+=1;self.last_prediction=copy.deepcopy(raw);self.predictions.append(copy.deepcopy(raw))
            if not accepted:self.invalidate('|'.join(blockers or ['SAFETY_REJECTION']));return False
            return True

    def finish(self,task_success,*,persist=None):
        with self.lock:
            if self.status is not None:return self.summary()
            if self.started is None:raise RuntimeError('trial_not_started')
            if not isinstance(task_success,bool):raise ValueError('task_outcome_requires_explicit_boolean_evidence')
            self.commands_inhibited=True;self.ended=self.clock()
            prospective={**self.summary(),'status':'SUCCESS' if task_success else 'FAILURE',
                         'runtime_valid':True,'task_success':task_success,'reason':None if task_success else 'FAILURE_TASK'}
            if persist:
                try:persist(prospective)
                except Exception as exc:return self.invalidate('LOGGER_ERROR:'+str(exc),category='INVALID_LOGGER_ERROR')
            self.status=prospective['status'];self.task_success=task_success;return self.summary()

    def summary(self):
        with self.lock:
            return dict(trial_id=self.trial_id,status=self.status,runtime_valid=self.runtime_valid,task_success=self.task_success,
                invalid_reason=self.invalid_event['invalid_reason'] if self.invalid_event else None,
                invalid_category=self.invalid_event['invalid_category'] if self.invalid_event else None,
                reason='FAILURE_TASK' if self.status=='FAILURE' else None,
                duration_s=self.ended-self.started if self.started is not None and self.ended is not None else None,
                prediction_count=self.prediction_count,command_count=self.command_count,**self.counts,task_phase_end=self.task_phase_end,
                evaluation_scope=self.evaluation_scope,commands_inhibited=self.commands_inhibited,
                **{key:(self.last_valid_observation or {}).get(key) for key in ('episode_id','condition','matched_pair_id','sim_observation_id','real_observation_id',
                   'observation_gap_score','representation_gap_score','action_gap_translation','action_gap_rotation','action_gap_gripper')})

    def save(self,directory,*,summary=None):
        directory=Path(directory);directory.mkdir(parents=True,exist_ok=True)
        def atomic(name,value):
            path=directory/name;temporary=path.with_suffix(path.suffix+'.tmp')
            with temporary.open('w') as f:json.dump(json_safe(value),f,allow_nan=False);f.flush();os.fsync(f.fileno())
            os.replace(temporary,path)
        with self.lock:
            result=summary or self.summary()
            if result['status'] not in STATUSES:raise RuntimeError('only_terminal_trial_can_be_saved')
            atomic('trial_summary.json',result);atomic('runtime_events.json',self.runtime_events);atomic('invalid_event.json',self.invalid_event)
            for filename,rows in [('observations.jsonl',self.observations),('predictions.jsonl',self.predictions)]:
                with (directory/filename).open('w') as f:
                    for row in rows:f.write(json.dumps(json_safe(row),allow_nan=False)+'\n')
                    f.flush();os.fsync(f.fileno())
