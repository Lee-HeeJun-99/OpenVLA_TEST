#!/usr/bin/env python3
"""Read-only planner reference interface used by recorded-input Shadow Mode."""

from __future__ import annotations

import json
from pathlib import Path
from typing import Any, Iterator


class RecordedPlannerEpisode:
    def __init__(self, episode_dir: Path) -> None:
        self.episode_dir = Path(episode_dir).expanduser().resolve()
        self.metadata = json.loads((self.episode_dir / "metadata.json").read_text(encoding="utf-8"))
        self.steps_path = self.episode_dir / "steps_with_actions.jsonl"
        self.steps = [json.loads(line) for line in self.steps_path.read_text(encoding="utf-8").splitlines() if line.strip()]
        if not self.steps:
            raise ValueError("planner episode has no steps")

    @property
    def pose_source(self) -> str:
        return str(self.metadata.get("pose_source", "unknown"))

    def iter_steps(self) -> Iterator[dict[str, Any]]:
        yield from self.steps

    def assert_not_measured_pose(self) -> None:
        if "measured" in self.pose_source.lower() or self.metadata.get("feedback_pose_samples", 0):
            return
        raise RuntimeError(
            f"pose_source={self.pose_source!r}; this episode is planned/commanded, not measured feedback"
        )


class LivePlannerInterface:
    """Deliberately unimplemented until Real safety review and explicit approval."""

    status = "BLOCKED_SAFETY_REVIEW"

    def execute(self) -> None:
        raise PermissionError("REAL_PLANNER_EXECUTION_REQUIRES_EXPLICIT_USER_APPROVAL")

