"""TCP provenance abstraction for measured, FK-estimated and unavailable data."""
from __future__ import annotations
from dataclasses import dataclass
from typing import Any, Callable, Mapping, Protocol


class TcpSource(Protocol):
    def sample(self, observation: Mapping[str, Any]) -> Mapping[str, Any]: ...


@dataclass
class MeasuredTcpSource:
    key: str = "measured_tcp_pose"
    def sample(self, observation):
        value = observation.get(self.key)
        if value is None:
            raise RuntimeError("measured_tcp_unavailable")
        return {"source": "MEASURED_CONTROLLER_TCP", "pose": value,
                "measured": True, "estimated": False}


@dataclass
class JointStateFkTcpSource:
    fk_function: Callable[[Mapping[str, float]], Mapping[str, Any]]
    def sample(self, observation):
        joints = observation.get("joint_positions_by_name")
        if not isinstance(joints, Mapping):
            raise RuntimeError("joint_state_unavailable_for_fk")
        result = dict(self.fk_function(joints))
        return {"source": "FK_ESTIMATED_TCP", "pose": result,
                "measured": False, "estimated": True}


class RecordedTcpSource:
    def sample(self, observation):
        value = observation.get("recorded_tcp_pose")
        if value is None:
            raise RuntimeError("recorded_tcp_unavailable")
        return {"source": "RECORDED_TCP", "pose": value,
                "measured": False, "estimated": False}


class UnavailableTcpSource:
    def sample(self, observation):
        raise RuntimeError("tcp_source_unavailable")

