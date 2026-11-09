"""Reuse the production gripper lifecycle without blocking the ServoL clock.

No ROS imports or IO on construction. The one worker executes full pulse
sequences serially. Its mailbox is latest-intent, not a queue of old grasps.
All normal IO submissions share the arm stream's inhibit lock. Fault cleanup
reuses BOTH_OUTPUTS_OFF/max3/readback and does not claim a physical jaw state.
"""
import math
import threading
import time
from full_policy_gripper import FullPolicyGripper
from real_sink import RealDoosanCommandSink,Authorization


class GripperOnlySink(RealDoosanCommandSink):
    def _send(self,kind,request,**kwargs):
        if kind!='gripper':raise PermissionError('servo_gripper_sink_cannot_send_arm_commands')
        return super()._send(kind,request,**kwargs)


class GripperDispatchGuard:
    def __init__(self,guard):self.guard=guard;self.lock=guard.lock;self.count=0
    def command_allowed(self):return not self.guard.closed and self.guard.reason is None
    def command_dispatched(self):self.count+=1


class ServoFullPolicyGripper:
    def __init__(self,factory,config,logger,guard,check,fail,*,age_limit=1.2):
        self.sink=GripperOnlySink(factory,Authorization(True,True,True,'MOTION_ENABLED'),gripper_config=config)
        self.sink.trial_guard=GripperDispatchGuard(guard)
        self.full_policy_gripper=FullPolicyGripper(self.sink,logger,
            dict(abort_behavior='BOTH_OUTPUTS_OFF',operator_approved=True))
        self.guard=guard;self.check=check;self.fail=fail;self.logger=logger;self.age_limit=age_limit
        self.cv=threading.Condition();self.pending=None;self.closing=False;self.busy=False
        self.worker=None;self.cleanup=None;self.cleanup_lock=threading.Lock();self.cleanup_result=None
        self.completed_transitions=0;self.replaced_intents=0

    def start(self):
        if not self.full_policy_gripper.initialized:raise PermissionError('observed_OPEN_required')
        if self.worker is not None:raise PermissionError('gripper_worker_already_started')
        self.worker=threading.Thread(target=self._work,name='servo-gripper-serial-worker');self.worker.start()

    def offer(self,value,source,sequence):
        if not math.isfinite(value) or not 0<=value<=1:raise ValueError('invalid_gripper_closedness')
        with self.guard.lock:
            self.guard.check()
            with self.cv:
                if self.closing:raise PermissionError('gripper_worker_closed')
                if self.pending is not None:self.replaced_intents+=1
                self.pending=(value,source,sequence);self.cv.notify_all()

    def _work(self):
        try:
            while True:
                with self.cv:
                    while self.pending is None and not self.closing:self.cv.wait(.02)
                    if self.closing:return
                    value,source,sequence=self.pending;self.pending=None;self.busy=True
                try:
                    self.check()
                    if not math.isfinite(source) or not 0<=time.monotonic()-source<=self.age_limit:
                        raise RuntimeError('stale_gripper_intent')
                    self.logger.append(dict(event='SERVO_GRIPPER_INTENT',sequence=sequence,
                        raw_closedness=value,source_monotonic=source,age_at_transition_start_s=time.monotonic()-source))
                    receipt=self.full_policy_gripper.transition(value)
                    if receipt['state']=='completed':self.completed_transitions+=1
                finally:
                    with self.cv:self.busy=False;self.cv.notify_all()
        except Exception as exc:
            if not self.guard.closed:self.fail('gripper:'+str(exc))

    def prepare_finish(self):
        """Called under guard.lock before stream close; no late ON is allowed."""
        with self.cv:
            unfinished=self.busy
            if self.pending is not None:
                value=self.pending[0];current=self.full_policy_gripper.state
                desired='OPEN' if value<.3 else 'CLOSED' if value>.7 else current
                unfinished=unfinished or desired!=current
            self.closing=True;self.pending=None;self.cv.notify_all()
            return unfinished

    def abort_async(self,reason):
        self.full_policy_gripper.inhibit();self.sink.inhibit()
        with self.cv:self.closing=True;self.pending=None;self.cv.notify_all()
        with self.cleanup_lock:
            if self.cleanup is not None:return
            def cleanup():
                try:self.cleanup_result=self.sink.abort_gripper_outputs_off()
                except Exception as exc:self.cleanup_result=dict(state='failed',error=str(exc),physical_jaw_state='UNKNOWN')
                try:self.logger.append(dict(event='SERVO_GRIPPER_ABORT_OFF',reason=reason,result=self.cleanup_result))
                except Exception:pass  # Broken logging must never suppress OFF.
            self.cleanup=threading.Thread(target=cleanup,name='servo-gripper-OFF-cleanup');self.cleanup.start()

    def join(self):
        # Existing sink bounds every discovery/ACK and OFF retries to three.
        for thread in (self.worker,self.cleanup):
            if thread is not None and thread is not threading.current_thread():
                while thread.is_alive():thread.join(.1)

    def summary(self):
        events=self.sink.events
        return dict(physical_gripper_commands=sum(e['sent_at'] is not None for e in events),
            logical_state=self.full_policy_gripper.state,measured_gripper_state=None,
            completed_transitions=self.completed_transitions,replaced_pending_intents=self.replaced_intents,
            abort_behavior='BOTH_OUTPUTS_OFF_MAX_3_ATTEMPTS',abort_result=self.cleanup_result)
