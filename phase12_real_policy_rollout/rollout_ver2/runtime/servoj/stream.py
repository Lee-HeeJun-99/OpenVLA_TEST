from __future__ import annotations
from dataclasses import dataclass
import threading,time
import numpy as np


@dataclass(frozen=True)
class StreamResult:
    sent_count:int;jitter_ms:dict;started:float;ended:float;fault:str|None


class ServoStreamRunner:
    """Absolute-deadline sender intended for process A's dedicated thread."""
    def __init__(self,publisher,*,clock=time.monotonic,sleep=time.sleep):
        self.publisher=publisher;self.clock=clock;self.sleep=sleep;self.abort=threading.Event()
    def inhibit(self):self.abort.set();self.publisher.inhibit()
    def run(self,segment,feedback_check=None,stop_callback=None):
        start=self.clock();jitters=[];sent=0;fault=None
        for index,(offset,q) in enumerate(zip(segment.times_s,segment.joints_rad)):
            deadline=start+float(offset)
            while not self.abort.is_set():
                remaining=deadline-self.clock()
                if remaining<=0:break
                self.sleep(min(remaining,.001))
            if self.abort.is_set():fault="ABORTED";break
            now=self.clock();jitters.append((now-deadline)*1000)
            try:
                if feedback_check is not None:
                    reason=feedback_check(q,index)
                    if reason:fault=str(reason);raise RuntimeError(fault)
                self.publisher.publish(q,now);sent+=1
            except Exception as error:
                fault=fault or f"STREAM_ERROR:{type(error).__name__}:{error}";self.publisher.inhibit()
                if stop_callback is not None:stop_callback(fault)
                break
        values=np.asarray(jitters) if jitters else np.asarray([0.])
        return StreamResult(sent,{"p50":float(np.percentile(values,50)),
          "p99":float(np.percentile(values,99)),"max":float(np.max(values))},start,self.clock(),fault)
    def start(self,segment,feedback_check=None,stop_callback=None):
        holder={}
        thread=threading.Thread(target=lambda:holder.setdefault("result",
          self.run(segment,feedback_check,stop_callback)),name="servoj-100hz-stream",daemon=True)
        thread.start();return thread,holder
