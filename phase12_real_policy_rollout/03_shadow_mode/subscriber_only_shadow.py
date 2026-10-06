"""Command-incapable Shadow core. ROS transport, if used, may only feed observations."""
from __future__ import annotations
import hashlib, math, time, uuid

class ShadowSampleError(RuntimeError): pass

def _finite_action(values, expected=7):
    out=[float(v) for v in values]
    if len(out)!=expected or not all(math.isfinite(v) for v in out):
        raise ShadowSampleError('invalid_action')
    return out

def validate_prediction(model, response):
    if model=='openvla':
        action=response.get('action')
        if action is None and response.get('actions'): action=response['actions'][0]
        return {'raw_response':response,'canonical_actions':[_finite_action(action)],'chunk_size':1}
    if model=='oft':
        actions=response.get('actions')
        if not isinstance(actions,list) or len(actions)!=5: raise ShadowSampleError('oft_requires_k5')
        return {'raw_response':response,'canonical_actions':[_finite_action(a) for a in actions],'chunk_size':5}
    raise ShadowSampleError('unknown_model')

class SubscriberOnlyShadowCore:
    """Builds immutable observation records; it has no command methods."""
    def __init__(self, config, logger):
        self.config=config; self.logger=logger; self.last_chunk_ids=set()
    def record(self, *, image_bytes, camera_source_timestamp, camera_clock_domain,
               receive_monotonic_timestamp, receive_ros_timestamp, joint_state,
               tcp_pose, predictions, errors=None):
        hold=[]; clean={}
        for model in ('openvla','oft'):
            try:
                result=validate_prediction(model,predictions[model])
                cid=str(predictions[model].get('chunk_id') or uuid.uuid4())
                if cid in self.last_chunk_ids: raise ShadowSampleError('duplicate_chunk')
                self.last_chunk_ids.add(cid); result['chunk_id']=cid; clean[model]=result
            except Exception as exc:
                hold.append(f'{model}:{exc}')
        record={
          'mode':'subscriber_only_shadow','instruction':self.config['instruction'],
          'image_sha256':hashlib.sha256(image_bytes).hexdigest(),
          'camera_source_timestamp':camera_source_timestamp,
          'camera_clock_domain':camera_clock_domain,
          'receive_monotonic_timestamp':float(receive_monotonic_timestamp),
          'receive_ros_timestamp':receive_ros_timestamp,
          'receive_monotonic_clock_domain':'host_monotonic',
          'receive_ros_clock_domain':'ros_clock',
          'joint_state':joint_state,'measured_tcp_pose':tcp_pose,
          'predictions':clean,'prediction_errors':errors or {},
          'executed_action':None,'robot_delivered_command':None,
          'command_issued':False,'hold_required':bool(hold or errors),
          'hold_reason':';'.join(hold) if hold else ('prediction_error' if errors else None),
          'valid':not bool(hold or errors),
          'exclusion_reason':';'.join(hold) if hold else ('prediction_error' if errors else None),
        }
        self.logger.append(record); return record

def monotonic_now(): return time.monotonic()
