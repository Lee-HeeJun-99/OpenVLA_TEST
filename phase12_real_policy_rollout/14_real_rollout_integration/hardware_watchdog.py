"""Independent fail-closed monitor; no ROS capability."""
import threading

REQUIRED=('camera_ok','joint_state_ok','tcp_ok','robot_state_ok','protective_stop_ok',
          'servo_mode_ok','authority_ok','logger_ok','model_ok','command_ack_ok','manual_abort_clear')

class HardwareWatchdog:
    def __init__(self,snapshot,on_fault,period=.02):
        self.snapshot=snapshot;self.on_fault=on_fault;self.period=period
        self.closed=threading.Event();self.fault=None;self.thread=None
    def check(self):
        try:
            state=self.snapshot()
            failed=[key for key in REQUIRED if state.get(key) is not True]
        except Exception as exc:failed=['observation_error:'+str(exc)]
        if failed and self.fault is None:
            self.fault='WATCHDOG_FAULT:'+','.join(failed)
            self.on_fault(self.fault)
        return self.fault
    def start(self):
        def loop():
            while not self.closed.wait(self.period):
                if self.check():break
        self.thread=threading.Thread(target=loop,daemon=True,name='hardware-watchdog');self.thread.start()
        return self
    def close(self):
        self.closed.set()
        if self.thread:self.thread.join(timeout=2)
