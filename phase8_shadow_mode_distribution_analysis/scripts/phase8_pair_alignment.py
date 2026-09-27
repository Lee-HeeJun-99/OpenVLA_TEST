#!/usr/bin/env python3
"""Build Phase 8 aligned real/sim and real/real frame pairs.

The current Phase 8 episodes are planned to share trajectory timing and frame
count. This script therefore starts with source step/frame index alignment and
adds quality metrics so downstream analyses can filter pairs if needed.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
from pathlib import Path
from statistics import mean, median, pstdev
from typing import Any


DEFAULT_BUNDLE = Path("/home/ubuntu/a0509_vla_linux_field_bundle_20260903")


def parse_entity(value: str) -> tuple[str, str]:
    # Examples: real_episode_000008, sim_episode_000004
    if value.startswith("real_episode_"):
        return "real", "episode_" + value.rsplit("episode_", 1)[1]
    if value.startswith("sim_episode_"):
        return "sim", "episode_" + value.rsplit("episode_", 1)[1]
    raise ValueError(f"Unsupported comparison entity: {value}")


def read_csv(path: Path) -> list[dict[str, str]]:
    with path.open("r", encoding="utf-8", newline="") as handle:
        return list(csv.DictReader(handle))


def write_csv(path: Path, rows: list[dict[str, Any]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    if not rows:
        path.write_text("", encoding="utf-8")
        return
    keys = list(rows[0].keys())
    with path.open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=keys)
        writer.writeheader()
        writer.writerows(rows)


def parse_json_list(value: str) -> list[float] | None:
    if not value or value == "not_available":
        return None
    try:
        parsed = json.loads(value)
    except Exception:
        return None
    if not isinstance(parsed, list):
        return None
    out = []
    for item in parsed:
        try:
            out.append(float(item))
        except Exception:
            return None
    return out


def safe_float(value: Any) -> float | None:
    try:
        if value in (None, "", "not_available"):
            return None
        out = float(value)
        if math.isnan(out) or math.isinf(out):
            return None
        return out
    except Exception:
        return None


def l2(a: list[float] | None, b: list[float] | None, dims: int | None = None) -> float | None:
    if a is None or b is None:
        return None
    if dims is not None:
        a = a[:dims]
        b = b[:dims]
    if len(a) != len(b):
        return None
    return math.sqrt(sum((x - y) ** 2 for x, y in zip(a, b)))


def quat_angle_rad(q1: list[float] | None, q2: list[float] | None) -> float | None:
    if q1 is None or q2 is None or len(q1) != 4 or len(q2) != 4:
        return None
    n1 = math.sqrt(sum(x * x for x in q1))
    n2 = math.sqrt(sum(x * x for x in q2))
    if n1 <= 0 or n2 <= 0:
        return None
    q1n = [x / n1 for x in q1]
    q2n = [x / n2 for x in q2]
    dot = abs(sum(a * b for a, b in zip(q1n, q2n)))
    dot = min(1.0, max(-1.0, dot))
    return 2.0 * math.acos(dot)


def row_key(row: dict[str, str]) -> tuple[str, str, int]:
    return row["domain"], row["episode_id"], int(float(row["frame_id"]))


def truthy(value: str) -> bool:
    return str(value).lower() in {"true", "1", "yes"}


def build_pairs(comparison: dict[str, Any], by_key: dict[tuple[str, str, int], dict[str, str]], max_frame_by_episode: dict[tuple[str, str], int]) -> list[dict[str, Any]]:
    left_domain, left_episode = parse_entity(comparison["left"])
    right_domain, right_episode = parse_entity(comparison["right"])
    left_max = max_frame_by_episode[(left_domain, left_episode)]
    right_max = max_frame_by_episode[(right_domain, right_episode)]
    max_common = min(left_max, right_max)
    rows: list[dict[str, Any]] = []

    for frame_id in range(max_common + 1):
        left = by_key.get((left_domain, left_episode, frame_id))
        right = by_key.get((right_domain, right_episode, frame_id))
        pair_valid = True
        reasons: list[str] = []
        if left is None:
            pair_valid = False
            reasons.append("missing_left_frame")
        if right is None:
            pair_valid = False
            reasons.append("missing_right_frame")
        if left is None or right is None:
            rows.append({
                "comparison_id": comparison["comparison_id"],
                "left_domain": left_domain,
                "left_episode": left_episode,
                "right_domain": right_domain,
                "right_episode": right_episode,
                "left_frame": frame_id,
                "right_frame": frame_id,
                "pair_valid": False,
                "invalid_reason": ";".join(reasons),
            })
            continue

        left_t = safe_float(left["timestamp"])
        right_t = safe_float(right["timestamp"])
        time_diff = None if left_t is None or right_t is None else abs(right_t - left_t)
        left_progress = frame_id / left_max if left_max > 0 else 0.0
        right_progress = frame_id / right_max if right_max > 0 else 0.0
        progress_diff = abs(left_progress - right_progress)

        phase_match = left["phase"] == right["phase"]
        if not phase_match:
            pair_valid = False
            reasons.append("phase_mismatch")

        left_gripper = safe_float(left["gripper_state"])
        right_gripper = safe_float(right["gripper_state"])
        gripper_match = None if left_gripper is None or right_gripper is None else abs(left_gripper - right_gripper) < 1e-9

        left_ee = parse_json_list(left["ee_position"])
        right_ee = parse_json_list(right["ee_position"])
        left_rot = parse_json_list(left["ee_rotation"])
        right_rot = parse_json_list(right["ee_rotation"])
        left_joint = parse_json_list(left["joint_state"])
        right_joint = parse_json_list(right["joint_state"])

        # Real-vs-real comparisons share planned-commanded pose convention.
        # Sim-vs-real EEF frames are not assumed comparable; report as null.
        comparable_real_pose = left_domain == "real" and right_domain == "real"
        ee_translation_error = l2(left_ee, right_ee, dims=3) if comparable_real_pose else None
        ee_rotation_error = quat_angle_rad(left_rot, right_rot) if comparable_real_pose else None
        joint_error = l2(left_joint, right_joint, dims=6) if left_joint and right_joint else None

        if truthy(left["valid"]) is False or truthy(right["valid"]) is False:
            pair_valid = False
            reasons.append("source_manifest_invalid")
        if time_diff is not None and time_diff > 0.35:
            pair_valid = False
            reasons.append("large_time_difference")
        if progress_diff > 1e-9:
            pair_valid = False
            reasons.append("progress_difference")

        rows.append({
            "comparison_id": comparison["comparison_id"],
            "purpose": comparison.get("purpose", ""),
            "left_domain": left_domain,
            "left_episode": left_episode,
            "left_condition": left["condition_id"],
            "left_frame": frame_id,
            "left_timestamp": left_t,
            "left_image_path": left["image_path"],
            "left_phase": left["phase"],
            "right_domain": right_domain,
            "right_episode": right_episode,
            "right_condition": right["condition_id"],
            "right_frame": frame_id,
            "right_timestamp": right_t,
            "right_image_path": right["image_path"],
            "right_phase": right["phase"],
            "time_difference": time_diff,
            "left_progress": left_progress,
            "right_progress": right_progress,
            "trajectory_progress_difference": progress_diff,
            "ee_translation_error_m": ee_translation_error,
            "ee_rotation_error_rad": ee_rotation_error,
            "joint_error_rad_l2": joint_error,
            "gripper_match": gripper_match,
            "phase_match": phase_match,
            "alignment_score": (
                (time_diff or 0.0)
                + progress_diff
                + (0.0 if phase_match else 1.0)
                + (0.0 if gripper_match in (True, None) else 1.0)
            ),
            "pair_valid": pair_valid,
            "invalid_reason": ";".join(reasons),
            "pose_comparison_note": "real_planned_pose_comparable" if comparable_real_pose else "sim_real_pose_frame_not_assumed_comparable",
        })
    return rows


def summarize(rows: list[dict[str, Any]]) -> list[dict[str, Any]]:
    out = []
    comparisons = sorted({r["comparison_id"] for r in rows})
    for comparison_id in comparisons:
        subset = [r for r in rows if r["comparison_id"] == comparison_id]
        valid = [r for r in subset if r.get("pair_valid") is True]
        def stats(key: str) -> dict[str, float | None]:
            vals = [r[key] for r in subset if isinstance(r.get(key), (int, float))]
            if not vals:
                return {f"{key}_mean": None, f"{key}_median": None, f"{key}_max": None}
            return {f"{key}_mean": mean(vals), f"{key}_median": median(vals), f"{key}_max": max(vals)}
        row = {
            "comparison_id": comparison_id,
            "pair_count": len(subset),
            "valid_pair_count": len(valid),
            "invalid_pair_count": len(subset) - len(valid),
            "valid_ratio": len(valid) / len(subset) if subset else None,
            "phase_mismatch_count": sum(1 for r in subset if r.get("phase_match") is False),
            "gripper_mismatch_count": sum(1 for r in subset if r.get("gripper_match") is False),
        }
        row.update(stats("time_difference"))
        row.update(stats("trajectory_progress_difference"))
        row.update(stats("ee_translation_error_m"))
        row.update(stats("ee_rotation_error_rad"))
        row.update(stats("joint_error_rad_l2"))
        out.append(row)
    return out


def write_report(path: Path, summary_rows: list[dict[str, Any]]) -> None:
    lines = [
        "# Phase 8 Pair Alignment Quality Report",
        "",
        "## Method",
        "",
        "Pairs are aligned by identical frame/source step index because the checked episodes have identical 45-frame planned trajectories.",
        "The file still records timing, progress, phase, gripper, and available pose/joint differences for downstream filtering.",
        "",
        "Real-vs-real EEF and joint differences use the recorded planned-commanded convention. Sim-vs-real EEF frames are not treated as physically comparable here.",
        "",
        "## Summary",
        "",
        "| Comparison | Pairs | Valid | Invalid | Phase mismatches | Gripper mismatches | Mean time diff | Max real EE error m |",
        "|---|---:|---:|---:|---:|---:|---:|---:|",
    ]
    for row in summary_rows:
        lines.append(
            f"| {row['comparison_id']} | {row['pair_count']} | {row['valid_pair_count']} | "
            f"{row['invalid_pair_count']} | {row['phase_mismatch_count']} | {row['gripper_mismatch_count']} | "
            f"{row.get('time_difference_mean')} | {row.get('ee_translation_error_m_max')} |"
        )
    lines.extend([
        "",
        "## Interpretation",
        "",
        "- These pairs are suitable for the next observation-distribution step when `pair_valid=true`.",
        "- For sim-vs-real comparisons, image comparisons are valid as fixed-reference visual comparisons, but numeric EEF pose distance is intentionally not interpreted.",
        "- For real-only sequential comparisons, planned pose differences are expected to be near zero if the commanded trajectory was preserved.",
    ])
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--phase8-root", type=Path, default=DEFAULT_BUNDLE / "lhj" / "phase8_shadow_mode_distribution_analysis")
    args = parser.parse_args()

    phase8 = args.phase8_root.expanduser().resolve()
    manifest_path = phase8 / "01_data_validation" / "integrated_manifest.csv"
    condition_path = phase8 / "configs" / "condition_manifest.json"
    out_dir = phase8 / "03_pair_alignment"
    out_dir.mkdir(parents=True, exist_ok=True)

    rows = read_csv(manifest_path)
    condition_manifest = json.loads(condition_path.read_text(encoding="utf-8"))
    by_key = {row_key(row): row for row in rows}
    max_frame_by_episode: dict[tuple[str, str], int] = {}
    for row in rows:
        key = (row["domain"], row["episode_id"])
        frame = int(float(row["frame_id"]))
        max_frame_by_episode[key] = max(max_frame_by_episode.get(key, -1), frame)

    all_pairs: list[dict[str, Any]] = []
    for comparison in condition_manifest["primary_comparisons"]:
        all_pairs.extend(build_pairs(comparison, by_key, max_frame_by_episode))

    summary_rows = summarize(all_pairs)
    write_csv(out_dir / "aligned_pairs.csv", all_pairs)
    write_csv(out_dir / "alignment_statistics.csv", summary_rows)
    write_report(out_dir / "alignment_quality_report.md", summary_rows)
    (out_dir / "alignment_statistics.json").write_text(json.dumps({
        "status": "COMPLETED",
        "alignment_method": "same_frame_index_with_quality_metrics",
        "comparison_count": len(summary_rows),
        "pair_count": len(all_pairs),
        "valid_pair_count": sum(1 for r in all_pairs if r["pair_valid"] is True),
        "invalid_pair_count": sum(1 for r in all_pairs if r["pair_valid"] is not True),
        "comparisons": summary_rows,
    }, indent=2), encoding="utf-8")
    print(json.dumps({"pair_count": len(all_pairs), "summary": summary_rows}, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
