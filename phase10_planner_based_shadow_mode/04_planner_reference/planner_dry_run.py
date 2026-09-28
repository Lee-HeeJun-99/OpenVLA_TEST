#!/usr/bin/env python3
"""Static Planner episode validation. Never imports ROS or sends commands."""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
from statistics import mean
from typing import Any


def finite(values: Any, size: int) -> bool:
    try:
        vector = [float(value) for value in values]
    except Exception:
        return False
    return len(vector) == size and all(math.isfinite(value) for value in vector)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--episode-dir", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--workspace-min-m", nargs=3, type=float, default=[0.25, -0.40, 0.02])
    parser.add_argument("--workspace-max-m", nargs=3, type=float, default=[0.90, 0.40, 0.76])
    parser.add_argument("--max-translation-step-m", type=float, default=0.035)
    args = parser.parse_args()

    episode = args.episode_dir.expanduser().resolve()
    metadata = json.loads((episode / "metadata.json").read_text(encoding="utf-8"))
    steps = [json.loads(line) for line in (episode / "steps_with_actions.jsonl").read_text(encoding="utf-8").splitlines() if line.strip()]
    reasons: list[str] = []
    if len(steps) < 2:
        reasons.append("too_few_steps")

    timestamps = [float(step["timestamp"]) for step in steps if step.get("timestamp") is not None]
    dts = [b - a for a, b in zip(timestamps[:-1], timestamps[1:])]
    if dts and not all(value > 0 for value in dts):
        reasons.append("non_monotonic_timestamp")
    measured_hz = None if not dts else 1.0 / mean(dts)

    invalid_actions = [index for index, step in enumerate(steps) if not finite(step.get("action"), 7)]
    if invalid_actions:
        reasons.append("invalid_action")
    poses = [step.get("tcp_pose") for step in steps]
    invalid_poses = [index for index, pose in enumerate(poses) if not finite(pose, 6)]
    if invalid_poses:
        reasons.append("invalid_tcp_pose")

    workspace_violations: list[int] = []
    segment_violations: list[int] = []
    valid_poses = [[float(value) for value in pose] for pose in poses if finite(pose, 6)]
    for index, pose in enumerate(valid_poses):
        if any(pose[axis] < args.workspace_min_m[axis] or pose[axis] > args.workspace_max_m[axis] for axis in range(3)):
            workspace_violations.append(index)
    for index, (left, right) in enumerate(zip(valid_poses[:-1], valid_poses[1:]), 1):
        distance = math.sqrt(sum((right[axis] - left[axis]) ** 2 for axis in range(3)))
        if distance > args.max_translation_step_m:
            segment_violations.append(index)
    if workspace_violations:
        reasons.append("workspace_violation")
    if segment_violations:
        reasons.append("segment_translation_violation")

    payload = {
        "status": "PASS_OFFLINE_ONLY" if not reasons else "FAIL",
        "robot_commands_published": 0,
        "episode_dir": str(episode),
        "step_count": len(steps),
        "instruction": metadata.get("instruction"),
        "pose_source": metadata.get("pose_source"),
        "feedback_pose_samples": metadata.get("feedback_pose_samples"),
        "configured_hz": metadata.get("record_frequency_hz"),
        "metadata_effective_hz": metadata.get("effective_record_frequency_hz"),
        "timestamp_measured_hz": measured_hz,
        "invalid_action_steps": invalid_actions,
        "invalid_pose_steps": invalid_poses,
        "workspace_violation_steps": workspace_violations,
        "segment_translation_violation_steps": segment_violations,
        "reasons": reasons,
        "real_execution_authorized": False,
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(payload, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(payload, indent=2))
    return 0 if not reasons else 1


if __name__ == "__main__":
    raise SystemExit(main())
