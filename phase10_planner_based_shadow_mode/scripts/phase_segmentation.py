#!/usr/bin/env python3
"""Map planner labels and grasp events to the Phase 10 analysis vocabulary."""

from __future__ import annotations

from typing import Any, Sequence


PHASE_MAP = {
    "hold": "approach",
    "alignment": "approach",
    "approach": "approach",
    "descent_to_grasp": "pre_grasp",
    "pregrasp": "pre_grasp",
    "pre_grasp": "pre_grasp",
    "grasp_close": "grasp",
    "grasp": "grasp",
    "lift": "lift",
    "post_lift": "post_lift",
}


def canonical_phase(planner_phase: str) -> str:
    return PHASE_MAP.get(str(planner_phase).lower(), "unknown")


def grasp_event_window(steps: Sequence[dict[str, Any]], *, pre_frames: int = 3) -> dict[str, Any]:
    close_command = next(
        (index for index, step in enumerate(steps) if float(step.get("gripper_closedness", 0.0)) >= 0.5),
        None,
    )
    lift_start = next(
        (index for index, step in enumerate(steps) if canonical_phase(step.get("planner_phase", "")) == "lift"),
        None,
    )
    # Existing Phase 8 logs contain commanded gripper only. A measured change
    # and contact estimate must remain null rather than being inferred as fact.
    return {
        "pre_close_start_frame": None if close_command is None else max(0, close_command - pre_frames),
        "close_command_frame": close_command,
        "measured_gripper_change_frame": None,
        "contact_estimate_frame": None,
        "lift_start_frame": lift_start,
        "measured_fields_available": False,
    }

