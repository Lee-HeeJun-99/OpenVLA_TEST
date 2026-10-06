"""Integrated Phase 12 runtime boundary with delivery physically absent.

This facade has the same logical position as the future motion runtime boundary,
but its only sink is HardCommandGate/NullCommandSink.  It must be replaced in a
separately approved motion gate; it cannot be toggled into command mode.
"""
from __future__ import annotations
from typing import Any, Mapping, Sequence

from command_disabled_runtime import CommandDisabledPolicyRuntime
from hard_command_gate import HardCommandGate, UNSAFE_FLAGS


def sealed_config() -> dict[str, Any]:
    return {
        "mode": "command_disabled_pre_motion",
        "include_action_adapter": False,
        "include_doosan_bridge": False,
        **{name: False for name in UNSAFE_FLAGS},
    }


class PreMotionRuntimeFacade:
    """Model-to-command-boundary integration without a delivery capability."""

    def __init__(self, model: str, *, operator_confirmed_initial_open: bool,
                 logger: Any | None = None):
        if operator_confirmed_initial_open is not True:
            raise RuntimeError("operator_confirmed_initial_open_required")
        self.model = model
        self.runtime = CommandDisabledPolicyRuntime(
            model, operator_confirmed_initial_open=True)
        self.gate = HardCommandGate(sealed_config())
        self.logger = logger
        self.faulted = False

    @property
    def capability_report(self) -> Mapping[str, Any]:
        return self.gate.capability_report

    def _record(self, row: dict[str, Any]) -> dict[str, Any]:
        if self.faulted:
            raise RuntimeError("facade_faulted")
        if self.logger is not None:
            try:
                self.logger.append(row)
            except Exception as exc:
                self.faulted = True
                row["technical_valid"] = False
                row["hold_required"] = True
                row["technical_blockers"] = list(row.get("technical_blockers") or []) + ["logger_failure"]
                row["logger_error"] = type(exc).__name__
                row["command_issued"] = False
                row["executed_action"] = None
                row["robot_delivered_command"] = None
        return row

    def inspect_openvla(self, action: Sequence[float], **context: Any) -> dict[str, Any]:
        if self.model != "openvla":
            raise RuntimeError("model_boundary_mismatch")
        candidate = self.runtime.evaluate(action, **context)
        return self._record(self.gate.audit_candidate(candidate))

    def enqueue_oft(self, actions: Sequence[Sequence[float]], chunk_id: str,
                    inference_monotonic: float) -> None:
        if self.model != "oft":
            raise RuntimeError("model_boundary_mismatch")
        self.runtime.enqueue_oft(actions, chunk_id, inference_monotonic)

    def inspect_next_oft(self, now_monotonic: float, **context: Any) -> dict[str, Any]:
        if self.model != "oft":
            raise RuntimeError("model_boundary_mismatch")
        candidate = self.runtime.next_oft(now_monotonic, **context)
        return self._record(self.gate.audit_candidate(candidate))

