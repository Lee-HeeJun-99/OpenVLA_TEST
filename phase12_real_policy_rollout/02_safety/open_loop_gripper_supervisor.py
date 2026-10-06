"""Fail-closed gripper policy for hardware without measured position feedback.

States describe command knowledge only. They never claim a measured jaw position.
This module has no ROS, IO, or hardware command capability.
"""
from __future__ import annotations
from dataclasses import dataclass
from enum import Enum
import math


class CommandKnowledge(str, Enum):
    UNKNOWN = "UNKNOWN"
    COMMAND_OPEN = "COMMAND_OPEN"
    COMMAND_CLOSED = "COMMAND_CLOSED"


@dataclass(frozen=True)
class OpenLoopDecision:
    raw_closedness: float | None
    command_knowledge_before: str
    command_knowledge_after: str
    candidate_command: str | None
    command_suppressed: bool
    accepted: bool
    lift_allowed: bool
    close_count: int
    reason: str
    measured_gripper_state: None = None


class OpenLoopGripperSupervisor:
    """One-close-per-rollout policy with phase gate and explicit UNKNOWN state."""

    def __init__(self, open_threshold=0.3, close_threshold=0.7):
        if not 0 <= open_threshold < close_threshold <= 1:
            raise ValueError("invalid_thresholds")
        self.open_threshold = float(open_threshold)
        self.close_threshold = float(close_threshold)
        self.state = CommandKnowledge.UNKNOWN
        self.close_count = 0

    def confirm_initial_open(self, operator_confirmed: bool) -> None:
        if operator_confirmed is not True:
            raise RuntimeError("initial_open_not_operator_confirmed")
        self.state = CommandKnowledge.COMMAND_OPEN
        self.close_count = 0

    def invalidate(self, reason: str) -> OpenLoopDecision:
        before = self.state
        self.state = CommandKnowledge.UNKNOWN
        return OpenLoopDecision(None, before.value, self.state.value, None, True,
                                False, False, self.close_count, reason)

    def resolve(self, closedness, *, phase, fresh=True, communication_ok=True,
                logger_ok=True, command_result_known=True) -> OpenLoopDecision:
        before = self.state
        try:
            value = float(closedness)
        except (TypeError, ValueError):
            return self.invalidate("invalid_closedness")
        if not math.isfinite(value):
            return self.invalidate("invalid_closedness")
        if not fresh:
            return self.invalidate("stale_or_timeout")
        if not communication_ok:
            return self.invalidate("communication_failure")
        if not logger_ok:
            return self.invalidate("logger_failure")
        if not command_result_known:
            return self.invalidate("command_result_unknown")
        if self.state is CommandKnowledge.UNKNOWN:
            return OpenLoopDecision(value, before.value, self.state.value, None, True,
                                    False, False, self.close_count, "unknown_state_blocks_command")

        requested = None
        if value <= self.open_threshold:
            requested = CommandKnowledge.COMMAND_OPEN
        elif value >= self.close_threshold:
            requested = CommandKnowledge.COMMAND_CLOSED

        if requested is None:
            return OpenLoopDecision(value, before.value, self.state.value, None, True,
                                    True, self.state is CommandKnowledge.COMMAND_CLOSED,
                                    self.close_count, "hysteresis_hold")
        if requested is self.state:
            return OpenLoopDecision(value, before.value, self.state.value, None, True,
                                    True, self.state is CommandKnowledge.COMMAND_CLOSED,
                                    self.close_count, "duplicate_suppressed")
        if requested is CommandKnowledge.COMMAND_CLOSED:
            if phase != "grasp_close":
                return OpenLoopDecision(value, before.value, self.state.value, None, True,
                                        False, False, self.close_count,
                                        "close_forbidden_outside_grasp_close")
            if self.close_count >= 1:
                return OpenLoopDecision(value, before.value, self.state.value, None, True,
                                        False, False, self.close_count,
                                        "close_limit_exceeded")
            self.state = requested
            self.close_count += 1
            return OpenLoopDecision(value, before.value, self.state.value, "CLOSE", False,
                                    True, True, self.close_count, "close_candidate_once")

        # Re-opening is permitted as a candidate, but never interpreted as measured state.
        self.state = requested
        return OpenLoopDecision(value, before.value, self.state.value, "OPEN", False,
                                True, False, self.close_count, "open_candidate")

