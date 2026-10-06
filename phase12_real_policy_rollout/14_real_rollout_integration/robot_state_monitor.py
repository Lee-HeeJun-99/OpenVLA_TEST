"""Normalize dsr_msgs2 RobotState without inventing absent servo/mode signals.

RobotState.actual_mode is position/torque control, NOT manual/auto robot mode.
No topic name is assumed: caller must provide a source verified in its graph.
"""
from dataclasses import dataclass,asdict
import threading,time

@dataclass(frozen=True)
class RobotHardwareState:
    connected: bool
    robot_mode: str|None
    servo_enabled: bool|None
    protective_stop: bool
    emergency_stop: bool
    motion_state: int
    timestamp: float
    source: str

class RobotStateMonitor:
    def __init__(self,max_age=.5):self.lock=threading.Lock();self.value=None;self.max_age=max_age
    def update(self,msg,source,*,robot_mode=None,servo_enabled=None,now=None):
        value=RobotHardwareState(not msg.disconnected,robot_mode,servo_enabled,
            msg.robot_state in (5,10),msg.robot_state in (6,7),int(msg.robot_state),
            time.monotonic() if now is None else now,source)
        with self.lock:self.value=value
    def snapshot(self,now=None):
        with self.lock:value=self.value
        now=time.monotonic() if now is None else now
        if value is None:return {'robot_state_ok':False,'protective_stop_ok':False,'servo_mode_ok':False,'hardware_state':None}
        fresh=0<=now-value.timestamp<=self.max_age
        return {'robot_state_ok':fresh and value.connected and value.motion_state in (1,2),
            'protective_stop_ok':fresh and not value.protective_stop and not value.emergency_stop,
            'servo_mode_ok':fresh and value.servo_enabled is True and value.robot_mode in ('AUTO','MANUAL'),
            'hardware_state':asdict(value),'hardware_state_fresh':fresh}
