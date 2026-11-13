"""Transport-independent ServoJ Mode-A orchestration (no ROS imports)."""
from __future__ import annotations
from dataclasses import dataclass
import time

@dataclass(frozen=True)
class ServoJStepResult:
    segment:object
    stream:object
    settle_s:float
    step_total_s:float

class ServoJModeACore:
    def __init__(self,planner,stream_runner,settle,*,clock=time.monotonic):
        self.planner=planner;self.stream_runner=stream_runner;self.settle=settle;self.clock=clock
    def execute_action(self,start_tool,delta_position_m,rotation_vector_rad,seed_joint_rad,duration_s):
        started=self.clock()
        # No publisher is touched until the complete segment passes precheck.
        segment=self.planner.plan_action(start_tool,delta_position_m,rotation_vector_rad,
                                         seed_joint_rad,duration_s)
        streamed=self.stream_runner.run(segment)
        settle_started=self.clock();self.settle(segment.joints_rad[-1])
        return ServoJStepResult(segment,streamed,self.clock()-settle_started,self.clock()-started)
