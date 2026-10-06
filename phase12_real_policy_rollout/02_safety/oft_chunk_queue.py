"""Timestamped OFT queue contract with no publisher or robot dependency."""
from dataclasses import dataclass
import math

@dataclass(frozen=True)
class QueuedAction:
    chunk_id: str
    chunk_index: int
    action: tuple
    inference_monotonic: float
    action_age_sec: float

class ChunkSafetyError(RuntimeError):pass
class OFTChunkQueue:
    def __init__(self,mode='sequential_k5',max_age_sec=1.2):
        if mode not in ('first_only','sequential_k5'):raise ValueError('invalid chunk mode')
        self.mode=mode;self.max_age_sec=float(max_age_sec);self._queue=[];self._last_chunk_id=None
    def clear(self):self._queue.clear()
    def enqueue(self,chunk,chunk_id,inference_monotonic):
        if chunk_id==self._last_chunk_id:raise ChunkSafetyError('duplicate_chunk')
        if len(chunk)!=5:raise ChunkSafetyError('oft_chunk_must_have_k5')
        checked=[]
        for action in chunk:
            if len(action)!=7 or not all(math.isfinite(float(x)) for x in action):raise ChunkSafetyError('invalid_action')
            checked.append(tuple(float(x) for x in action))
        use=checked[:1] if self.mode=='first_only' else checked
        self._queue=[(chunk_id,i,a,float(inference_monotonic)) for i,a in enumerate(use)]
        self._last_chunk_id=chunk_id
    def pop(self,now_monotonic):
        if not self._queue:raise ChunkSafetyError('queue_underrun_hold_required')
        cid,index,action,ts=self._queue.pop(0);age=float(now_monotonic)-ts
        if age<0:raise ChunkSafetyError('clock_domain_or_time_regression')
        if age>self.max_age_sec:self.clear();raise ChunkSafetyError('stale_chunk_hold_required')
        return QueuedAction(cid,index,action,ts,age)
