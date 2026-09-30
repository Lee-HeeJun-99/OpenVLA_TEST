#!/usr/bin/env python3
"""Extract dataset reference commands; never imports ROS or robot APIs."""
from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
from typing import Sequence


def normalize(q: Sequence[float]) -> list[float]:
    values = [float(v) for v in q]
    norm = math.sqrt(sum(v * v for v in values))
    if norm < 1e-12:
        raise ValueError("zero quaternion")
    return [v / norm for v in values]


def rpy_to_quaternion(roll: float, pitch: float, yaw: float) -> list[float]:
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    return normalize([cr*cp*cy + sr*sp*sy, sr*cp*cy - cr*sp*sy,
                      cr*sp*cy + sr*cp*sy, cr*cp*sy - sr*sp*cy])


def multiply(a: Sequence[float], b: Sequence[float]) -> list[float]:
    aw, ax, ay, az = a; bw, bx, by, bz = b
    return [aw*bw-ax*bx-ay*by-az*bz, aw*bx+ax*bw+ay*bz-az*by,
            aw*by-ax*bz+ay*bw+az*bx, aw*bz+ax*by-ay*bx+az*bw]


def relative_rotvec(previous_rpy: Sequence[float], current_rpy: Sequence[float]) -> list[float]:
    """Match the collector's saved-dataset convention: q_current * inv(q_previous)."""
    previous = rpy_to_quaternion(*[float(v) for v in previous_rpy])
    current = rpy_to_quaternion(*[float(v) for v in current_rpy])
    relative = normalize(multiply(current, [previous[0], -previous[1], -previous[2], -previous[3]]))
    if relative[0] < 0:
        relative = [-v for v in relative]
    vector_norm = math.sqrt(sum(v*v for v in relative[1:]))
    if vector_norm < 1e-10:
        return [0.0, 0.0, 0.0]
    angle = 2 * math.atan2(vector_norm, relative[0])
    return [v / vector_norm * angle for v in relative[1:]]


def load_rows(episode_dir: Path) -> tuple[dict, list[dict]]:
    metadata = json.loads((episode_dir / "metadata.json").read_text(encoding="utf-8"))
    source = episode_dir / "steps.jsonl"
    rows = [json.loads(line) for line in source.read_text(encoding="utf-8").splitlines() if line.strip()]
    if len(rows) < 2:
        raise ValueError("at least two recorded steps required")
    return metadata, rows


def extract(episode_dir: Path) -> list[dict]:
    metadata, rows = load_rows(episode_dir)
    pose_source = str(metadata.get("pose_source", "UNRESOLVED"))
    reference_frame = str(metadata.get("coordinate_frame", "UNRESOLVED"))
    output = []
    for index, previous in enumerate(rows):
        current = rows[index + 1] if index + 1 < len(rows) else previous
        prev_pose = [float(v) for v in previous["tcp_pose"]]
        curr_pose = [float(v) for v in current["tcp_pose"]]
        translation = [curr_pose[i] - prev_pose[i] for i in range(3)]
        rotation = relative_rotvec(prev_pose[3:6], curr_pose[3:6])
        gripper = float(current.get("gripper_closedness", 1.0 - float(current["gripper"])))
        valid = all(math.isfinite(v) for v in translation + rotation + [gripper]) and 0 <= gripper <= 1
        output.append({
            "episode_id": str(metadata.get("episode_id", episode_dir.name)),
            "step_index": int(previous.get("step_index", index)),
            "source_timestamp": previous.get("image_timestamp"),
            "receive_timestamp": previous.get("source_timestamp"),
            "phase": str(previous.get("planner_phase", "unknown")),
            "reference_pose_previous": prev_pose,
            "reference_pose_current": curr_pose,
            "reference_delta_translation_m": translation,
            "reference_delta_rotation": {"value": rotation, "representation": "relative_rotation_vector_rad",
                "source_interpretation": "collector_rpy_assumption"},
            "reference_gripper_command": gripper,
            "reference_frame": reference_frame,
            "pose_source": pose_source,
            "measured_feedback_available": False,
            "measured_action": None,
            "canonical_action": [*translation, *rotation, gripper],
            "valid": valid,
            "exclusion_reason": None if valid else "NONFINITE_OR_INVALID_GRIPPER",
            "reference_status": "REFERENCE_COMMAND_NOT_MEASURED_GROUND_TRUTH",
        })
    return output


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--episode-dir", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    rows = extract(args.episode_dir)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    with args.output.open("x", encoding="utf-8") as stream:
        for row in rows:
            stream.write(json.dumps(row, ensure_ascii=False, separators=(",", ":")) + "\n")
    print(json.dumps({"status": "COMPLETED_WITH_RECORDED_DATA", "records": len(rows),
                      "robot_commands": 0, "output": str(args.output)}))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
