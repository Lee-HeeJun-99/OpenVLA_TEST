"""Doosan service boundary. Construction never creates a ROS command client."""
from dataclasses import dataclass, asdict
import math
import time
import uuid
import threading

SERVICES = {
    'pose': ('/dsr01/motion/move_line','MoveLine'),
    'gripper': ('/dsr01/io/set_tool_digital_output','SetToolDigitalOutput'),
    'hold': ('/dsr01/motion/move_stop','MoveStop'),
    'stop': ('/dsr01/motion/move_stop','MoveStop'),
}

@dataclass(frozen=True)
class Authorization:
    motion_enabled: bool = False
    explicit_motion_approval: bool = False
    hardware_preflight_passed: bool = False
    command_gate: str = 'COMMAND_DISABLED'

    def require(self):
        if not (self.motion_enabled is True and self.explicit_motion_approval is True and
                self.hardware_preflight_passed is True and self.command_gate=='MOTION_ENABLED'):
            raise PermissionError('motion_not_authorized')

class RealDoosanCommandSink:
    def __init__(self, transport_factory, authorization=None, *, ack_timeout=3., gripper_config=None):
        self.authorization=authorization or Authorization()
        self.factory=transport_factory
        self.transport=None
        self.ack_timeout=ack_timeout
        self.gripper_config=gripper_config or {}
        self.events=[]
        self.aborted=False
        self.last_gripper=None
        self.gripper_cancel=threading.Event()
        self.gripper_state='GRIPPER_IDLE'
        self._transport_lock=threading.Lock()
        self._abort_event=threading.Event()
        self.trial_guard=None
        self._dispatch_lock=threading.RLock()

    def cancel_gripper(self):
        self.gripper_cancel.set();self.gripper_state='GRIPPER_ABORTED'

    def inhibit(self):
        self.aborted=True;self._abort_event.set();self.cancel_gripper()

    def health_check(self):
        return {'command_capability_enabled':self.transport is not None,
                'motion_authorized':all((self.authorization.motion_enabled,self.authorization.explicit_motion_approval,
                    self.authorization.hardware_preflight_passed,self.authorization.command_gate=='MOTION_ENABLED')),
                'services':SERVICES,'aborted':self.aborted}

    def _send(self, kind, request, *, emergency=False):
        self.authorization.require()
        if self.aborted and not emergency:raise RuntimeError('pipeline_aborted')
        with self._transport_lock:
            if self.transport is None:self.transport=self.factory()
        event=dict(command_id=uuid.uuid4().hex,command_type=kind,requested_at=time.monotonic(),
            sent_at=None,ack_at=None,completed_at=None,ack_latency=None,state='requested',
            request=request,result=None,failure_reason=None,clock_domain='HOST_MONOTONIC')
        self.events.append(event)
        try:
            # Discovery/request construction may block, but never holds the trial
            # invalidation lock. Only the actual async submission is serialized.
            prepared=self.transport.prepare(*SERVICES[kind],request) if hasattr(self.transport,'prepare') else None
            gate=self.trial_guard.lock if self.trial_guard is not None else self._dispatch_lock
            with gate:
                if not emergency and (self.aborted or (self.trial_guard is not None and not self.trial_guard.command_allowed())):
                    raise PermissionError('trial_command_inhibited')
                future=self.transport.dispatch_prepared(prepared) if prepared is not None else self.transport.send(*SERVICES[kind],request)
                event.update(sent_at=time.monotonic(),state='sent')
                if not emergency and self.trial_guard is not None:self.trial_guard.command_dispatched()
            result=self.wait_for_ack(future,self.ack_timeout)
            event.update(ack_at=time.monotonic(),state='acknowledged',result=result)
            event['ack_latency']=event['ack_at']-event['sent_at']
            if self._abort_event.is_set() and not emergency:
                event.update(state='ABORTED_IN_FLIGHT',failure_reason='IN_FLIGHT_STATUS_UNKNOWN',
                    physical_completion='UNVERIFIED')
                return dict(event)
            if result.get('success') is not True:raise RuntimeError('service_response_failure')
            event.update(state='completed',completed_at=time.monotonic(),
                completion_evidence='SYNCHRONOUS_SERVICE_RETURN_NOT_INDEPENDENT_PHYSICAL_FEEDBACK')
        except TimeoutError:
            event.update(state='timeout',failure_reason='command_ack_timeout')
            self.aborted=True
        except Exception as exc:
            event.update(state='failed',failure_reason=str(exc));self.aborted=True
        return dict(event)

    def wait_for_ack(self, future, timeout):
        return self.transport.wait(future,timeout)

    def send_pose(self, pose_mm_zyz_deg, *, velocity=(20.,20.), acceleration=(20.,20.)):
        pose=[float(x) for x in pose_mm_zyz_deg]
        if len(pose)!=6 or not all(math.isfinite(x) for x in pose):raise ValueError('invalid_pose')
        if not all(0<float(x)<=20 for x in (*velocity,*acceleration)):raise ValueError('velocity_acceleration_contract')
        return self._send('pose',dict(pos=pose,vel=list(velocity),acc=list(acceleration),time=0.,radius=0.,ref=0,mode=0,blend_type=0,sync_type=0))

    def send_gripper(self, closed):
        self.authorization.require()
        cfg=self.gripper_config
        if cfg.get('polarity_confirmed') is not True:raise PermissionError('gripper_polarity_unconfirmed')
        if cfg.get('abort_value') not in (0,1):raise PermissionError('gripper_abort_value_unconfirmed')
        if self.gripper_cancel.is_set():raise PermissionError('gripper_aborted')
        if closed==self.last_gripper:return {'state':'suppressed','command_type':'gripper','reason':'duplicate'}
        prefix='closed' if closed else 'open'
        index=cfg[f'gripper_{prefix}_output_index']
        active=cfg[f'gripper_{prefix}_hardware_value'];inactive=cfg['inactive_hardware_value']
        if index not in range(1,7) or active not in (0,1) or inactive not in (0,1):raise ValueError('invalid_gripper_io')
        events=[]
        self.gripper_state='GRIPPER_CLOSING' if closed else 'GRIPPER_OPENING'
        for _ in range(cfg[f'{prefix}_pulse_count']):
            on=self._send('gripper',dict(index=index,value=active));events.append(on)
            if on['state']!='completed':return {'state':on['state'],'events':events}
            if self.gripper_cancel.wait(cfg[f'{prefix}_pulse_time_s']):
                safe=self._send('gripper',dict(index=index,value=cfg['abort_value']),emergency=True)
                events.append(safe)
                return {'state':'aborted','events':events,'measured_gripper_state':None}
            off=self._send('gripper',dict(index=index,value=inactive));events.append(off)
            if off['state']!='completed':return {'state':off['state'],'events':events}
        self.last_gripper=closed
        self.gripper_state='GRIPPER_HOLDING'
        return {'state':'completed','events':events,'measured_gripper_state':None}

    def hold(self):return self._send('hold',{'stop_mode':3},emergency=True)
    def stop(self):return self._send('stop',{'stop_mode':0},emergency=True)

class AbortBoundary:
    def __init__(self,sink):
        self.sink=sink;self.state='RUNNING';self.reason=None;self.receipt=None;self.transitions=['RUNNING']
        self.lock=threading.Lock()
    def abort(self,reason):
        with self.lock:
            if self.state!='RUNNING':return {'state':self.state,'reason':self.reason,'receipt':self.receipt}
            self.state='ABORT_REQUESTED';self.reason=reason
            self.sink.inhibit()
        return self._abort_once(reason)
    def _abort_once(self,reason):
        self.reason=reason;self.state='HOLD_REQUESTED'
        self.transitions.append(self.state)
        self.receipt=self.sink.hold()
        if self.receipt['state']=='completed':
            self.state='HOLD_ACKNOWLEDGED';self.transitions.append(self.state)
        else:self.receipt={'hold':self.receipt,'stop':self.sink.stop()}
        self.sink.aborted=True
        self.state='ABORTED'
        self.transitions.append(self.state)
        return {'state':self.state,'reason':reason,'receipt':self.receipt,'transitions':self.transitions}

class RosServiceTransport:
    """Lazy import; only authorized sink constructs this transport."""
    def __init__(self,node):self.node=node;self.clients={}
    def send(self,name,type_name,values):
        return self.dispatch_prepared(self.prepare(name,type_name,values))
    def prepare(self,name,type_name,values):
        if (name,type_name) not in SERVICES.values():raise ValueError('service_not_allowlisted')
        import dsr_msgs2.srv as interfaces
        cls=getattr(interfaces,type_name)
        if name not in self.clients:self.clients[name]=self.node.create_client(cls,name)
        client=self.clients[name]
        if not client.wait_for_service(timeout_sec=1.):raise TimeoutError('service_unavailable')
        request=cls.Request()
        for key,value in values.items():setattr(request,key,value)
        return client,request
    @staticmethod
    def dispatch_prepared(prepared):
        client,request=prepared
        return client.call_async(request)
    def wait(self,future,timeout):
        # Node must run in an independent MultiThreadedExecutor.
        deadline=time.monotonic()+timeout
        while not future.done() and time.monotonic()<deadline:time.sleep(.005)
        if not future.done():raise TimeoutError('ack_timeout')
        response=future.result()
        if response is None:raise RuntimeError('empty_response')
        return {'success':bool(response.success)}

class FakeRosTransport:
    """Exact service schemas and names, in memory; no DDS/ROS graph."""
    def __init__(self,outcomes=None):self.calls=[];self.outcomes=list(outcomes or [])
    def send(self,name,type_name,values):
        if (name,type_name) not in SERVICES.values():raise ValueError('unknown_service')
        self.calls.append((name,type_name,values));return len(self.calls)
    def wait(self,future,timeout):
        value=self.outcomes.pop(0) if self.outcomes else {'success':True}
        if isinstance(value,Exception):raise value
        return value
