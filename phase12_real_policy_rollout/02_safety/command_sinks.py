"""Command sink interfaces for offline validation; contains no ROS API."""
from __future__ import annotations
from dataclasses import dataclass, field
from typing import Any, Mapping, Protocol


class CommandSink(Protocol):
    def submit(self, requested_action: Mapping[str, Any],
               safety_decision: Mapping[str, Any]) -> Mapping[str, Any]: ...


@dataclass
class NullCommandSink:
    records: list[dict[str, Any]] = field(default_factory=list)

    def submit(self, requested_action, safety_decision):
        record = {
            "sink": "NULL_COMMAND_SINK",
            "requested_action": dict(requested_action),
            "safety_decision": dict(safety_decision),
            "would_send_command": bool(safety_decision.get("accepted", False)),
            "command_issued": False,
            "robot_delivered_command": None,
            "executed_action": None,
        }
        self.records.append(record)
        return record


@dataclass
class MockDoosanCommandSink:
    """In-memory software-path validator; never talks to a robot."""
    accepted_records: list[dict[str, Any]] = field(default_factory=list)
    rejected_records: list[dict[str, Any]] = field(default_factory=list)

    def submit(self, requested_action, safety_decision):
        accepted = bool(safety_decision.get("accepted", False))
        record = {
            "sink": "MOCK_DOOSAN_COMMAND_SINK",
            "requested_action": dict(requested_action),
            "safety_decision": dict(safety_decision),
            "mock_accepted": accepted,
            "command_issued": False,
            "robot_delivered_command": None,
            "executed_action": None,
        }
        (self.accepted_records if accepted else self.rejected_records).append(record)
        return record


class DoosanCommandSinkSkeleton:
    """Future interface marker. Construction and submission always fail closed."""
    def __init__(self, *, motion_enabled=False, explicit_motion_approval=False):
        if motion_enabled or explicit_motion_approval:
            raise RuntimeError("REAL_DOOSAN_SINK_NOT_IMPLEMENTED_OR_VALIDATED")
        raise RuntimeError("REAL_DOOSAN_SINK_DISABLED")

