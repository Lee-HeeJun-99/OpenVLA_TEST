#!/usr/bin/env python3
"""Pure safety decisions for Phase 10; contains no ROS publisher."""

from __future__ import annotations

from dataclasses import dataclass, field
import math
from typing import Sequence


ROBOT_CRITICAL_FLAGS = {
    "planner_timeout",
    "state_timeout",
    "communication_disconnect",
    "workspace_limit",
    "joint_limit",
    "velocity_limit",
    "unexpected_gripper_state",
    "emergency_stop",
    "critical_logger_failure",
}


@dataclass
class SafetyDecision:
    hold_required: bool
    planner_may_continue: bool
    invalid_prediction: bool
    reasons: list[str] = field(default_factory=list)


def validate_finite_action(values: Sequence[float]) -> bool:
    return len(values) == 7 and all(math.isfinite(float(value)) for value in values)


def decide(flags: dict[str, bool], *, stop_on_sync_failure: bool = True) -> SafetyDecision:
    active = sorted(key for key, value in flags.items() if value)
    robot_reasons = [key for key in active if key in ROBOT_CRITICAL_FLAGS]
    sync_failure = any(key in active for key in ("camera_timeout", "camera_state_sync_failure"))
    ai_failure = any(key in active for key in ("openvla_timeout", "oft_timeout", "ai_invalid_action"))
    hold = bool(robot_reasons) or (sync_failure and stop_on_sync_failure)
    return SafetyDecision(
        hold_required=hold,
        planner_may_continue=not hold,
        invalid_prediction=ai_failure or sync_failure,
        reasons=active,
    )


class ShadowCommandGate:
    """Hard structural block: AI commands can never be published in shadow mode."""

    def __init__(self, *, shadow_mode: bool = True) -> None:
        if not shadow_mode:
            raise ValueError("Phase 10 runner only supports shadow_mode=true")
        self.shadow_mode = True

    def publish_ai_action(self, _action: Sequence[float]) -> None:
        raise PermissionError("AI_ACTION_PUBLISH_BLOCKED_IN_SHADOW_MODE")

