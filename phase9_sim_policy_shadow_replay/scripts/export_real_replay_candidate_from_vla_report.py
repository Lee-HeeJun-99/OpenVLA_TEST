#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
from typing import Any, Mapping

import numpy as np


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Export a real-shadow replay candidate from an Isaac Sim closed-loop "
            "VLA validation report. This does not execute the real robot."
        )
    )
    parser.add_argument("--validation-report", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--episode-index", type=int, default=0)
    parser.add_argument("--max-steps", type=int, default=0)
    parser.add_argument(
        "--translation-scale",
        type=float,
        default=1.0,
        help="Scale applied to policy XYZ delta before real replay candidate export.",
    )
    parser.add_argument(
        "--rotation-scale",
        type=float,
        default=1.0,
        help="Scale applied to policy rotation delta before real replay candidate export.",
    )
    parser.add_argument(
        "--drop-rotation",
        action="store_true",
        help=(
            "Keep rotation deltas in the JSON but mark the candidate as XYZ-only. "
            "Use this for first safety review if real/sim orientation frames are not verified."
        ),
    )
    return parser.parse_args()


def _as_vector(value: Any, length: int, *, required: bool = True) -> list[float] | None:
    if value is None:
        if required:
            raise ValueError("Missing vector")
        return None
    arr = np.asarray(value, dtype=np.float64).reshape(-1)
    if arr.size < length or not np.all(np.isfinite(arr[:length])):
        raise ValueError(f"Invalid vector: {value}")
    return arr[:length].tolist()


def _row_from_action(
    action_record: Mapping[str, Any],
    step_index: int,
    translation_scale: float,
    rotation_scale: float,
    drop_rotation: bool,
) -> dict[str, Any]:
    raw_action = _as_vector(action_record.get("raw_action"), 7, required=False)
    clipped_action = _as_vector(action_record.get("clipped_action"), 7, required=False)
    action = clipped_action or raw_action
    if action is None:
        raise ValueError(f"Missing raw/clipped action at step {step_index}")
    current_position = _as_vector(action_record.get("current_position_m"), 3)
    target_position = _as_vector(action_record.get("target_position_m"), 3, required=False)
    current_quat = _as_vector(action_record.get("current_quaternion_wxyz"), 4)
    target_quat = _as_vector(action_record.get("target_quaternion_wxyz"), 4, required=False)

    xyz_delta_m = [float(v) * float(translation_scale) for v in action[:3]]
    rot_delta_rad = [float(v) * float(rotation_scale) for v in action[3:6]]
    replay_action = list(xyz_delta_m) + ([0.0, 0.0, 0.0] if drop_rotation else rot_delta_rad) + [float(action[6])]
    replay_target_position = [
        float(current_position[i]) + xyz_delta_m[i] for i in range(3)
    ]

    return {
        "step_index": step_index,
        "policy_step": int(action_record.get("policy_step", step_index)),
        "chunk_index": action_record.get("chunk_index"),
        "chunk_size": action_record.get("chunk_size"),
        "server_request_index": action_record.get("server_request_index"),
        "variant": action_record.get("variant"),
        "inference_seconds": action_record.get("inference_seconds"),
        "raw_action": raw_action,
        "clipped_action": clipped_action,
        "real_replay_action_delta": replay_action,
        "real_replay_mode": "xyz_delta_fixed_orientation" if drop_rotation else "xyz_rot_delta",
        "sim_current_position_m": current_position,
        "sim_current_quaternion_wxyz": current_quat,
        "sim_target_position_m_from_report": target_position,
        "sim_target_quaternion_wxyz_from_report": target_quat,
        "candidate_target_position_m_from_scaled_delta": replay_target_position,
        "gripper_target_closedness": action_record.get("gripper_target_closedness"),
        "gripper_measured_at_policy_step": action_record.get("gripper_measured_at_policy_step"),
        "ik_success_in_sim": bool(action_record.get("ik_success", False)),
        "action_was_clipped_in_sim": bool(action_record.get("action_was_clipped", False)),
    }


def main() -> None:
    args = parse_args()
    report_path = args.validation_report.expanduser().resolve()
    payload = json.loads(report_path.read_text(encoding="utf-8"))
    results = payload.get("results")
    if not isinstance(results, list):
        raise ValueError("Validation report must contain a results list")
    candidates = [
        result
        for result in results
        if isinstance(result, Mapping)
        and int(result.get("episode_index", len(results))) == args.episode_index
    ]
    if not candidates:
        raise ValueError(f"No result with episode_index={args.episode_index}")
    result = candidates[0]
    actions = result.get("actions")
    if not isinstance(actions, list):
        raise ValueError("Selected result has no actions list")
    if args.max_steps > 0:
        actions = actions[: args.max_steps]

    rows = [
        _row_from_action(
            action_record,
            index,
            args.translation_scale,
            args.rotation_scale,
            args.drop_rotation,
        )
        for index, action_record in enumerate(actions)
        if isinstance(action_record, Mapping)
    ]
    if not rows:
        raise ValueError("No replay candidate rows were exported")

    output_dir = args.output_dir.expanduser().resolve()
    output_dir.mkdir(parents=True, exist_ok=True)
    jsonl_path = output_dir / "real_replay_candidate_actions.jsonl"
    with jsonl_path.open("w", encoding="utf-8", newline="\n") as handle:
        for row in rows:
            handle.write(json.dumps(row, ensure_ascii=False) + "\n")

    summary = {
        "status": "CANDIDATE_EXPORTED_REVIEW_REQUIRED",
        "source_validation_report": str(report_path),
        "episode_index": args.episode_index,
        "step_count": len(rows),
        "translation_scale": float(args.translation_scale),
        "rotation_scale": float(args.rotation_scale),
        "drop_rotation": bool(args.drop_rotation),
        "important_warning": (
            "This file is a real replay candidate, not an approved robot command. "
            "Real/sim TCP frame equivalence and workspace safety must be reviewed before execution."
        ),
        "max_abs_xyz_delta_m": float(
            max(
                math.sqrt(sum(float(v) ** 2 for v in row["real_replay_action_delta"][:3]))
                for row in rows
            )
        ),
        "closed_gripper_steps": int(
            sum(float(row["real_replay_action_delta"][6]) >= 0.5 for row in rows)
        ),
        "sim_ik_failed_steps": int(
            sum(not bool(row["ik_success_in_sim"]) for row in rows)
        ),
        "output_jsonl": str(jsonl_path),
    }
    summary_path = output_dir / "real_replay_candidate_summary.json"
    summary_path.write_text(
        json.dumps(summary, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )
    print(json.dumps(summary, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
