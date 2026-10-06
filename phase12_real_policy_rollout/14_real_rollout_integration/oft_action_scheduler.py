"""Target cadence independent of blocking command worker. No overlapping motion."""
from concurrent.futures import ThreadPoolExecutor
import time

class ActionScheduler:
    def __init__(self,worker,clock=time.monotonic):
        self.worker=worker;self.clock=clock;self.pool=ThreadPoolExecutor(max_workers=1)
        self.pending=None;self.rows=[];self.aborted=False
        self.last_dispatch=None
    def dispatch(self,inference_frame,k,origin,payload):
        if self.aborted:raise RuntimeError('scheduler_aborted')
        target_step=inference_frame+k;target=origin+target_step*.2
        now=self.clock()
        row=dict(target_step=target_step,target_dispatch_time=target,actual_dispatch_time=None,
                 lateness_ms=max(0.,(now-target)*1000),ack_time=None,completion_time=None,
                 cadence_status='CADENCE_NOT_HARDWARE_VALIDATED')
        if now+1e-9<target:row['status']='NOT_DUE'
        elif self.pending and not self.pending.done():row['status']='NO_OVERLAPPING_MOTION';self.aborted=True
        else:
            if self.last_dispatch is not None and now-self.last_dispatch<.2:
                time.sleep(.2-(now-self.last_dispatch)+.0001)
                now=self.clock()
            row.update(status='DISPATCHED',actual_dispatch_time=now)
            row['lateness_ms']=max(0.,(now-target)*1000)
            self.last_dispatch=now
            def execute():
                result=self.worker(payload)
                row['ack_time']=result.get('ack_at');row['completion_time']=result.get('completed_at')
                row['result']=result
                return result
            self.pending=self.pool.submit(execute)
        self.rows.append(row);return row
    def close(self):self.pool.shutdown(wait=True,cancel_futures=True)
    def abort(self):
        self.aborted=True
        if self.pending:self.pending.cancel()
