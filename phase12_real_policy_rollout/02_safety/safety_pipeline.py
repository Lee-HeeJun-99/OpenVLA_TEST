"""Unified canonical-action to fail-closed safety decision pipeline."""
from __future__ import annotations
from dataclasses import dataclass
from typing import Any, Mapping

from canonical_action import CanonicalAction
from command_disabled_runtime import CommandDisabledPolicyRuntime


@dataclass(frozen=True)
class RuntimeState:
    now_monotonic: float
    current_position_m: tuple[float, float, float]
    current_abc_deg: tuple[float, float, float]
    phase: str
    camera_ok: bool = True
    joint_state_ok: bool = True
    tcp_ok: bool = True
    model_ok: bool = True
    communication_ok: bool = True
    logger_ok: bool = True
    joint_limits_ok: bool = True


@dataclass(frozen=True)
class SafetyDecision:
    accepted: bool
    rejected: bool
    hold_required: bool
    reason: tuple[str, ...]
    filtered_action: tuple[float, ...] | None
    candidate: Mapping[str, Any]
    command_issued: bool = False

    def as_dict(self):
        return {"accepted": self.accepted, "rejected": self.rejected,
                "hold_required": self.hold_required, "reason": list(self.reason),
                "filtered_action": list(self.filtered_action) if self.filtered_action else None,
                "command_issued": False}


class SafetyPipeline:
    def __init__(self, model: str, *, operator_confirmed_initial_open: bool):
        self.runtime = CommandDisabledPolicyRuntime(
            model, operator_confirmed_initial_open=operator_confirmed_initial_open)

    def inspect(self, action: CanonicalAction, state: RuntimeState) -> SafetyDecision:
        candidate = self.runtime.evaluate(
            action.as_vector(), action_id=action.sequence_id,
            source_monotonic=action.timestamp_monotonic,
            now_monotonic=state.now_monotonic,
            current_position_m=state.current_position_m,
            current_abc_deg=state.current_abc_deg, phase=state.phase,
            camera_ok=state.camera_ok, state_ok=state.joint_state_ok and state.joint_limits_ok,
            tcp_ok=state.tcp_ok, inference_ok=state.model_ok,
            communication_ok=state.communication_ok, logger_ok=state.logger_ok,
            chunk_id=action.sequence_id.rsplit("-k", 1)[0] if action.source_model == "oft" else None,
            chunk_index=action.chunk_index,
            action_age_sec=state.now_monotonic-action.timestamp_monotonic)
        blockers = list(candidate.get("technical_blockers") or [])
        if not state.joint_limits_ok and "joint_limit_violation" not in blockers:
            blockers.append("joint_limit_violation")
        accepted = not blockers
        return SafetyDecision(accepted, not accepted, bool(blockers), tuple(blockers),
                              tuple(candidate["limited_canonical_action"]) if accepted else None,
                              candidate, False)

