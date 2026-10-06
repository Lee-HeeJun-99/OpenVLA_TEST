"""Fail-closed Phase 12 command boundary.

This module intentionally has no ROS imports and no delivery API.  It is the
last boundary for pre-motion validation: candidates may be audited, but they
cannot be published, sent to a service, or executed.
"""
from __future__ import annotations

from dataclasses import dataclass, asdict
from typing import Any, Mapping


UNSAFE_FLAGS = (
    "allow_robot_command",
    "allow_gripper_command",
    "allow_home_command",
    "allow_trajectory_execution",
    "allow_motion_service",
    "allow_stop_service",
    "allow_estop_service",
    "publish_ai_action",
)


@dataclass(frozen=True)
class NullCommandSink:
    """A capability-free sink: it only returns an audit receipt."""

    sink_name: str = "NULL_COMMAND_SINK"

    def audit(self, candidate: Mapping[str, Any]) -> dict[str, Any]:
        return {
            "sink": self.sink_name,
            "candidate_action_id": candidate.get("action_id"),
            "delivery_blocked": True,
            "delivery_reason": "command_capability_not_installed",
            "executed_action": None,
            "robot_delivered_command": None,
            "command_issued": False,
        }


class HardCommandGate:
    """Validates configuration and guarantees null delivery in this stage."""

    def __init__(self, config: Mapping[str, Any]) -> None:
        unsafe = [name for name in UNSAFE_FLAGS if config.get(name) is not False]
        if unsafe:
            raise RuntimeError("unsafe_or_missing_false_flags:" + ",".join(unsafe))
        if config.get("mode") != "command_disabled_pre_motion":
            raise RuntimeError("mode_must_be_command_disabled_pre_motion")
        if config.get("include_action_adapter") is not False:
            raise RuntimeError("action_adapter_must_be_excluded")
        if config.get("include_doosan_bridge") is not False:
            raise RuntimeError("doosan_bridge_must_be_excluded")
        self._sink = NullCommandSink()

    @property
    def capability_report(self) -> dict[str, Any]:
        return {
            "robot_publisher": False,
            "gripper_publisher_or_client": False,
            "motion_service_client": False,
            "motion_action_client": False,
            "home_capability": False,
            "trajectory_capability": False,
            "hold_or_estop_client": False,
            "realtime_write_capability": False,
            "action_adapter_loaded": False,
            "doosan_bridge_loaded": False,
            "sink": asdict(self._sink),
        }

    def audit_candidate(self, candidate: Mapping[str, Any]) -> dict[str, Any]:
        receipt = self._sink.audit(candidate)
        blockers = list(candidate.get("technical_blockers") or [])
        return {
            **dict(candidate),
            "hard_gate": {
                "technical_candidate_valid": bool(candidate.get("technical_valid")),
                "candidate_blockers": blockers,
                "motion_enable_input_present": False,
                **receipt,
            },
            "executed_action": None,
            "robot_delivered_command": None,
            "command_issued": False,
            "pre_motion_ready": False,
        }

