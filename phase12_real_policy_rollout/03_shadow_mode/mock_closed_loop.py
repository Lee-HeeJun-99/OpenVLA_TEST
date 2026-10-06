"""Software-only closed-loop path validation; not a physics simulator."""
from __future__ import annotations
from dataclasses import dataclass
from typing import Iterable
from canonical_action import CanonicalAction
from command_sinks import MockDoosanCommandSink
from safety_pipeline import RuntimeState,SafetyPipeline

@dataclass
class MockRobotState:
    position_m:list[float]
    abc_deg:list[float]
    monotonic:float=0.0
    phase:str='alignment'

class MockClosedLoop:
    def __init__(self,model='mock'):
        runtime_model='openvla' if model=='mock' else model
        self.pipeline=SafetyPipeline(runtime_model,operator_confirmed_initial_open=True)
        self.sink=MockDoosanCommandSink()
    def run(self,state:MockRobotState,actions:Iterable[CanonicalAction]):
        rows=[]
        for action in actions:
            state.monotonic=max(state.monotonic+.2,action.timestamp_monotonic)
            decision=self.pipeline.inspect(action,RuntimeState(state.monotonic,tuple(state.position_m),tuple(state.abc_deg),state.phase))
            receipt=self.sink.submit(action.as_dict(),decision.as_dict())
            if receipt['mock_accepted'] and decision.filtered_action:
                for i in range(3):state.position_m[i]+=decision.filtered_action[i]
                target=decision.candidate['target_pose_candidate_mm_zyz_deg'];state.abc_deg=list(target[3:6])
            rows.append({'action':action.as_dict(),'decision':decision.as_dict(),'receipt':receipt,
                         'mock_state':{'position_m':list(state.position_m),'abc_deg':list(state.abc_deg)},
                         'command_issued':False,'executed_action':None,'robot_delivered_command':None})
        return rows

