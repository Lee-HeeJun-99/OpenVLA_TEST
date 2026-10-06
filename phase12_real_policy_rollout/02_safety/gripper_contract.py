"""Pure gripper semantics for offline tests; contains no ROS or hardware calls."""
from dataclasses import dataclass
import math

@dataclass(frozen=True)
class GripperDecision:
    raw_closedness: float
    commanded_closed: bool
    command_suppressed: bool
    reason: str

class ClosednessHysteresis:
    """Dataset contract: 0=open, 1=closed."""
    def __init__(self, open_threshold=0.3, close_threshold=0.7, initial_closed=False):
        if not 0 <= open_threshold < close_threshold <= 1: raise ValueError('invalid thresholds')
        self.open_threshold=float(open_threshold);self.close_threshold=float(close_threshold);self.closed=bool(initial_closed);self.last_commanded=None
    def resolve(self,value):
        value=float(value)
        if not math.isfinite(value):raise ValueError('non-finite gripper closedness')
        if value <= self.open_threshold:self.closed=False;reason='open_threshold'
        elif value >= self.close_threshold:self.closed=True;reason='close_threshold'
        else:reason='hysteresis_hold'
        suppressed=self.last_commanded is not None and self.last_commanded==self.closed
        if not suppressed:self.last_commanded=self.closed
        return GripperDecision(value,self.closed,suppressed,reason)
