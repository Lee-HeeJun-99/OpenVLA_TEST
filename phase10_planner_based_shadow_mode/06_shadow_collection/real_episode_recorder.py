#!/usr/bin/env python3
"""Offline synchronizer/recorder for observed Real-runtime samples."""
from __future__ import annotations

from pathlib import Path
from typing import Any

from real_runtime_adapter import RealRuntimeObserverAdapter


class RealEpisodeRecorder:
    def __init__(self, config_path: Path, max_age_sec: float = 0.25) -> None:
        self.adapter = RealRuntimeObserverAdapter(config_path)
        self.max_age_sec = float(max_age_sec)

    def combine(self, camera: dict[str, Any], state: dict[str, Any],
                planner: dict[str, Any], executed: dict[str, Any] | None = None) -> dict[str, Any]:
        domains = {camera["clock_domain"], state["clock_domain"], planner["clock_domain"]}
        comparable = len(domains) == 1
        ages = None
        valid = True
        reason = None
        if comparable:
            ages = {"camera_state_sec": abs(camera["timestamp"] - state["timestamp"]),
                    "camera_planner_sec": abs(camera["timestamp"] - planner["timestamp"])}
            if max(ages.values()) > self.max_age_sec:
                valid, reason = False, "MESSAGE_SYNC_TIMEOUT"
        else:
            valid, reason = False, "CLOCK_DOMAIN_UNRESOLVED_NO_DIRECT_SUBTRACTION"
        return {"camera": camera, "state": state, "planner": planner,
                "executed_command_observation": executed, "clock_domain_comparable": comparable,
                "alignment_deltas": ages, "openvla_executed_action": None,
                "oft_executed_action": None, "hold_required": not valid,
                "valid": valid, "exclusion_reason": reason}
