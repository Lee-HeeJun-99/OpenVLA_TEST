#!/usr/bin/env python3
"""Validate phase8 episode data before distribution analysis.

This script only inspects existing real/sim episode files. It does not modify
source data or run model inference.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
from dataclasses import dataclass
from pathlib import Path
from statistics import mean, median, pstdev
from typing import Any

try:
    from PIL import Image
except Exception:  # pragma: no cover - environment dependent
    Image = None


DEFAULT_BUNDLE = Path("/home/ubuntu/a0509_vla_linux_field_bundle_20260903")
REAL_EPISODES = ["episode_000004", "episode_000008", "episode_000009", "episode_000010"]
SIM_EPISODES = ["episode_000004", "episode_000008", "episode_000009", "episode_000010"]


@dataclass
class EpisodeCheck:
    domain: str
    episode_id: str
    condition_id: str
    episode_dir: Path
    steps_path: Path | None
    steps_with_actions_path: Path | None
    metadata_path: Path | None
    image_dir: Path | None
    frames_path: Path | None
    steps_count: int = 0
    steps_with_actions_count: int = 0
    image_count: int = 0
    frames_count: int = 0
    metadata_num_steps: int | None = None
    instruction: str | None = None
    target_color: str | None = None
    effective_record_frequency_hz: float | None = None
    duration_sec: float | None = None
    success: bool | None = None
    skipped_image: int | None = None
    duplicate_image_count: int | None = None
    pose_source: str | None = None
    action_shape: str | None = None
    chunk_or_step_action_dim_ok: bool | None = None
    image_decode_ok: int = 0
    image_decode_failed: int = 0
    timestamp_monotonic: bool | None = None
    timestamp_mean_dt: float | None = None
    timestamp_median_dt: float | None = None
    timestamp_min_dt: float | None = None
    timestamp_max_dt: float | None = None
    timestamp_std_dt: float | None = None
    inferred_hz: float | None = None
    invalid_reasons: list[str] | None = None


def read_json(path: Path) -> dict[str, Any]:
    return json.loads(path.read_text(encoding="utf-8"))


def read_jsonl(path: Path | None) -> list[dict[str, Any]]:
    if path is None or not path.exists():
        return []
    rows: list[dict[str, Any]] = []
    with path.open("r", encoding="utf-8") as handle:
        for line in handle:
            line = line.strip()
            if line:
                rows.append(json.loads(line))
    return rows


def safe_float(value: Any) -> float | None:
    try:
        if value is None:
            return None
        out = float(value)
        if math.isnan(out) or math.isinf(out):
            return None
        return out
    except Exception:
        return None


def dt_stats(timestamps: list[float]) -> dict[str, float | bool | None]:
    if len(timestamps) < 2:
        return {
            "monotonic": None,
            "mean_dt": None,
            "median_dt": None,
            "min_dt": None,
            "max_dt": None,
            "std_dt": None,
            "hz": None,
        }
    dts = [b - a for a, b in zip(timestamps[:-1], timestamps[1:])]
    mean_dt = mean(dts)
    return {
        "monotonic": all(dt > 0 for dt in dts),
        "mean_dt": mean_dt,
        "median_dt": median(dts),
        "min_dt": min(dts),
        "max_dt": max(dts),
        "std_dt": pstdev(dts) if len(dts) > 1 else 0.0,
        "hz": 1.0 / mean_dt if mean_dt > 0 else None,
    }


def image_size(path: Path) -> tuple[int, int] | None:
    if Image is None:
        return None
    try:
        with Image.open(path) as img:
            img.verify()
        with Image.open(path) as img:
            return img.size
    except Exception:
        return None


def condition_for_episode(condition_manifest: dict[str, Any], episode_id: str, domain: str) -> str:
    if episode_id == "episode_000004":
        return "baseline_real_episode4" if domain == "real" else "baseline_sim_episode4"
    for item in condition_manifest.get("new_real_conditions", []):
        if item.get("real_episode") == episode_id:
            return str(item.get("condition_id"))
    return "not_declared"


def real_episode_dir(bundle: Path, episode_id: str) -> Path:
    return bundle / "data" / "real_world" / "raw_dataset_oft" / "episodes" / episode_id


def sim_episode_dir(bundle: Path, episode_id: str) -> Path:
    return bundle / "outputs" / "sim2real_analysis" / f"raw_dataset_oft_{episode_id}_home_relative_joint_replay_images"


def sim_json_path(bundle: Path, episode_id: str) -> Path:
    return bundle / "outputs" / "sim2real_analysis" / f"raw_dataset_oft_{episode_id}_home_relative_joint_replay.json"


def check_episode(bundle: Path, condition_manifest: dict[str, Any], domain: str, episode_id: str) -> tuple[EpisodeCheck, list[dict[str, Any]], list[dict[str, Any]], list[dict[str, Any]]]:
    if domain == "real":
        ep_dir = real_episode_dir(bundle, episode_id)
        metadata_path = ep_dir / "metadata.json"
        steps_path = ep_dir / "steps.jsonl"
        swa_path = ep_dir / "steps_with_actions.jsonl"
        image_dir = ep_dir / "images" / "primary"
        frames_path = None
    else:
        ep_dir = sim_episode_dir(bundle, episode_id)
        metadata_path = sim_json_path(bundle, episode_id)
        steps_path = None
        swa_path = None
        image_dir = ep_dir / "images" / "primary"
        frames_path = ep_dir / "frames.jsonl"

    check = EpisodeCheck(
        domain=domain,
        episode_id=episode_id,
        condition_id=condition_for_episode(condition_manifest, episode_id, domain),
        episode_dir=ep_dir,
        steps_path=steps_path if steps_path and steps_path.exists() else None,
        steps_with_actions_path=swa_path if swa_path and swa_path.exists() else None,
        metadata_path=metadata_path if metadata_path.exists() else None,
        image_dir=image_dir if image_dir.exists() else None,
        frames_path=frames_path if frames_path and frames_path.exists() else None,
        invalid_reasons=[],
    )

    steps = read_jsonl(check.steps_path)
    steps_with_actions = read_jsonl(check.steps_with_actions_path)
    frames = read_jsonl(check.frames_path)
    check.steps_count = len(steps)
    check.steps_with_actions_count = len(steps_with_actions)
    check.frames_count = len(frames)

    metadata: dict[str, Any] = {}
    if check.metadata_path is not None:
        metadata = read_json(check.metadata_path)
    else:
        check.invalid_reasons.append("missing_metadata")

    if domain == "real":
        check.metadata_num_steps = metadata.get("num_steps")
        check.instruction = metadata.get("instruction")
        check.target_color = metadata.get("target_color")
        check.effective_record_frequency_hz = safe_float(metadata.get("effective_record_frequency_hz"))
        check.duration_sec = safe_float(metadata.get("duration_sec"))
        check.success = metadata.get("success")
        check.skipped_image = metadata.get("skipped_image")
        check.duplicate_image_count = metadata.get("duplicate_image_count")
        check.pose_source = metadata.get("pose_source")
        action_conv = metadata.get("action_convention") or {}
        check.action_shape = json.dumps(action_conv.get("shape"))
    else:
        image_capture = metadata.get("image_capture") or {}
        check.metadata_num_steps = metadata.get("source_step_count")
        check.effective_record_frequency_hz = safe_float(metadata.get("replay_rate_hz"))
        check.duration_sec = None
        check.success = metadata.get("status") == "complete"
        check.skipped_image = None
        check.duplicate_image_count = None
        check.pose_source = "sim_joint_replay_home_relative"
        check.action_shape = "not_available"
        if image_capture.get("frame_count") is not None:
            check.frames_count = int(image_capture.get("frame_count"))

    image_paths = sorted(check.image_dir.glob("*.jpg")) if check.image_dir else []
    check.image_count = len(image_paths)
    sizes: dict[tuple[int, int], int] = {}
    for path in image_paths:
        size = image_size(path)
        if size is None:
            if Image is None:
                continue
            check.image_decode_failed += 1
        else:
            check.image_decode_ok += 1
            sizes[size] = sizes.get(size, 0) + 1

    rows_for_time = steps if domain == "real" else frames
    time_key = "timestamp" if domain == "real" else "source_time_s"
    timestamps = [safe_float(row.get(time_key)) for row in rows_for_time]
    timestamps = [x for x in timestamps if x is not None]
    stats = dt_stats(timestamps)
    check.timestamp_monotonic = stats["monotonic"]  # type: ignore[assignment]
    check.timestamp_mean_dt = stats["mean_dt"]  # type: ignore[assignment]
    check.timestamp_median_dt = stats["median_dt"]  # type: ignore[assignment]
    check.timestamp_min_dt = stats["min_dt"]  # type: ignore[assignment]
    check.timestamp_max_dt = stats["max_dt"]  # type: ignore[assignment]
    check.timestamp_std_dt = stats["std_dt"]  # type: ignore[assignment]
    check.inferred_hz = stats["hz"]  # type: ignore[assignment]

    if domain == "real":
        action_rows = steps_with_actions
        dims = [len(row.get("action", [])) if isinstance(row.get("action"), list) else -1 for row in action_rows]
        check.chunk_or_step_action_dim_ok = bool(dims) and all(dim == 7 for dim in dims)
    else:
        check.chunk_or_step_action_dim_ok = None

    expected = check.metadata_num_steps
    if expected is not None:
        if domain == "real":
            if check.steps_count != expected:
                check.invalid_reasons.append("steps_count_mismatch_metadata")
            if check.steps_with_actions_count != expected:
                check.invalid_reasons.append("steps_with_actions_count_mismatch_metadata")
        if check.image_count != expected:
            check.invalid_reasons.append("image_count_mismatch_metadata")
    if domain == "real" and check.steps_count != check.steps_with_actions_count:
        check.invalid_reasons.append("steps_vs_steps_with_actions_mismatch")
    if check.image_decode_failed > 0:
        check.invalid_reasons.append("image_decode_failed")
    if check.timestamp_monotonic is False:
        check.invalid_reasons.append("timestamp_not_monotonic")
    if check.chunk_or_step_action_dim_ok is False:
        check.invalid_reasons.append("action_dim_not_7")

    manifest_rows: list[dict[str, Any]] = []
    if domain == "real":
        for row in steps_with_actions:
            image_rel = row.get("image")
            image_path = bundle / "data" / "real_world" / "raw_dataset_oft" / image_rel if image_rel else None
            manifest_rows.append({
                "domain": "real",
                "condition_id": check.condition_id,
                "episode_id": episode_id,
                "frame_id": row.get("step_index"),
                "timestamp": row.get("timestamp"),
                "image_path": str(image_path) if image_path else "not_available",
                "instruction": row.get("instruction") or check.instruction or "not_available",
                "joint_state": json.dumps(row.get("joint_position_rad")),
                "ee_position": json.dumps((row.get("end_effector_pose") or {}).get("position_m")),
                "ee_rotation": json.dumps((row.get("end_effector_pose") or {}).get("quaternion_wxyz")),
                "gripper_state": row.get("gripper_closedness"),
                "planner_action": json.dumps(row.get("action")),
                "phase": row.get("planner_phase"),
                "valid": True,
                "exclusion_reason": "",
            })
    else:
        for row in frames:
            images = row.get("images") if isinstance(row.get("images"), dict) else {}
            image_path_value = (
                row.get("image_path")
                or row.get("image")
                or row.get("path")
                or images.get("primary")
            )
            if image_path_value and not str(image_path_value).startswith("/"):
                image_path = ep_dir / str(image_path_value)
            else:
                image_path = Path(str(image_path_value)) if image_path_value else None
            manifest_rows.append({
                "domain": "sim",
                "condition_id": check.condition_id,
                "episode_id": episode_id,
                "frame_id": row.get("source_step_index", row.get("frame_index", row.get("step_index"))),
                "timestamp": row.get("source_time_s", row.get("timestamp")),
                "image_path": str(image_path) if image_path else "not_available",
                "instruction": row.get("instruction", "not_available"),
                "joint_state": json.dumps(row.get("joint_position_rad")),
                "ee_position": "not_available",
                "ee_rotation": "not_available",
                "gripper_state": row.get("gripper_closedness"),
                "planner_action": "not_available",
                "phase": row.get("planner_phase", "not_available"),
                "valid": True,
                "exclusion_reason": "",
            })

    timing_rows = [{
        "domain": domain,
        "episode_id": episode_id,
        "condition_id": check.condition_id,
        "count": len(timestamps),
        "monotonic": check.timestamp_monotonic,
        "mean_dt_sec": check.timestamp_mean_dt,
        "median_dt_sec": check.timestamp_median_dt,
        "min_dt_sec": check.timestamp_min_dt,
        "max_dt_sec": check.timestamp_max_dt,
        "std_dt_sec": check.timestamp_std_dt,
        "inferred_hz": check.inferred_hz,
        "metadata_effective_hz_or_replay_hz": check.effective_record_frequency_hz,
    }]

    invalid_rows = []
    if check.invalid_reasons:
        invalid_rows.append({
            "domain": domain,
            "episode_id": episode_id,
            "condition_id": check.condition_id,
            "invalid_reasons": ";".join(check.invalid_reasons),
            "episode_dir": str(ep_dir),
        })
    return check, manifest_rows, timing_rows, invalid_rows


def write_csv(path: Path, rows: list[dict[str, Any]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    if not rows:
        path.write_text("", encoding="utf-8")
        return
    keys = list(rows[0].keys())
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=keys)
        writer.writeheader()
        writer.writerows(rows)


def dataclass_row(check: EpisodeCheck) -> dict[str, Any]:
    return {
        "domain": check.domain,
        "episode_id": check.episode_id,
        "condition_id": check.condition_id,
        "episode_dir": str(check.episode_dir),
        "steps_count": check.steps_count,
        "steps_with_actions_count": check.steps_with_actions_count,
        "image_count": check.image_count,
        "frames_count": check.frames_count,
        "metadata_num_steps": check.metadata_num_steps,
        "instruction": check.instruction,
        "target_color": check.target_color,
        "effective_record_frequency_hz": check.effective_record_frequency_hz,
        "duration_sec": check.duration_sec,
        "success": check.success,
        "skipped_image": check.skipped_image,
        "duplicate_image_count": check.duplicate_image_count,
        "pose_source": check.pose_source,
        "action_shape": check.action_shape,
        "action_dim_7_ok": check.chunk_or_step_action_dim_ok,
        "image_decode_ok": check.image_decode_ok,
        "image_decode_failed": check.image_decode_failed,
        "timestamp_monotonic": check.timestamp_monotonic,
        "timestamp_mean_dt": check.timestamp_mean_dt,
        "timestamp_median_dt": check.timestamp_median_dt,
        "timestamp_min_dt": check.timestamp_min_dt,
        "timestamp_max_dt": check.timestamp_max_dt,
        "timestamp_std_dt": check.timestamp_std_dt,
        "inferred_hz": check.inferred_hz,
        "invalid_reasons": ";".join(check.invalid_reasons or []),
    }


def write_report(path: Path, checks: list[EpisodeCheck], invalid_rows: list[dict[str, Any]]) -> None:
    lines = [
        "# Phase 8 Data Integrity Report",
        "",
        "## Scope",
        "",
        "Validated baseline and changed-condition episodes before distribution analysis.",
        "",
        "Real episodes: `episode_000004`, `episode_000008`, `episode_000009`, `episode_000010`.",
        "",
        "Sim replay episodes checked for file availability and image/frame count.",
        "",
        "## Summary",
        "",
        "| Domain | Episode | Condition | Steps | Actions | Images | Metadata steps | Hz | Status |",
        "|---|---:|---|---:|---:|---:|---:|---:|---|",
    ]
    for c in checks:
        status = "VALID" if not c.invalid_reasons else "CHECK"
        lines.append(
            f"| {c.domain} | {c.episode_id} | {c.condition_id} | {c.steps_count} | "
            f"{c.steps_with_actions_count} | {c.image_count} | {c.metadata_num_steps} | "
            f"{c.inferred_hz if c.inferred_hz is not None else 'n/a'} | {status} |"
        )
    lines.extend(["", "## Findings", ""])
    real_checks = [c for c in checks if c.domain == "real"]
    if all(c.steps_count == 45 and c.steps_with_actions_count == 45 and c.image_count == 45 for c in real_checks):
        lines.append("- Real episode4/8/9/10 each contain 45 steps, 45 action rows, and 45 primary images.")
    if all(c.chunk_or_step_action_dim_ok for c in real_checks):
        lines.append("- Real `steps_with_actions.jsonl` action vectors are 7D for all checked rows.")
    if all(c.timestamp_monotonic for c in real_checks):
        lines.append("- Real timestamps are monotonic for all checked episodes.")
    if all((c.skipped_image == 0 and c.duplicate_image_count == 0) for c in real_checks):
        lines.append("- Metadata reports zero skipped images and zero duplicate images for all checked real episodes.")
    lines.append("- `pose_source` is `planned_commanded_pose`; measured robot tracking error is not available from these files.")
    if invalid_rows:
        lines.extend(["", "## Items Requiring Attention", ""])
        for row in invalid_rows:
            lines.append(f"- {row['domain']} {row['episode_id']}: {row['invalid_reasons']}")
    else:
        lines.extend(["", "## Items Requiring Attention", "", "- None from this file-level integrity pass."])
    lines.extend([
        "",
        "## Interpretation Limits",
        "",
        "- This report validates file-level integrity only.",
        "- It does not prove physical pose equivalence beyond planned/commanded trajectory metadata.",
        "- Shadow Mode and closed-loop rollout logs were not analyzed in this step.",
    ])
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--bundle-root", type=Path, default=DEFAULT_BUNDLE)
    parser.add_argument(
        "--phase8-root",
        type=Path,
        default=DEFAULT_BUNDLE / "lhj" / "phase8_shadow_mode_distribution_analysis",
    )
    args = parser.parse_args()

    bundle = args.bundle_root.expanduser().resolve()
    phase8 = args.phase8_root.expanduser().resolve()
    condition_manifest_path = phase8 / "configs" / "condition_manifest.json"
    condition_manifest = read_json(condition_manifest_path)

    checks: list[EpisodeCheck] = []
    manifest_rows: list[dict[str, Any]] = []
    timing_rows: list[dict[str, Any]] = []
    invalid_rows: list[dict[str, Any]] = []

    for episode_id in REAL_EPISODES:
        check, manifest, timing, invalid = check_episode(bundle, condition_manifest, "real", episode_id)
        checks.append(check)
        manifest_rows.extend(manifest)
        timing_rows.extend(timing)
        invalid_rows.extend(invalid)

    for episode_id in SIM_EPISODES:
        check, manifest, timing, invalid = check_episode(bundle, condition_manifest, "sim", episode_id)
        checks.append(check)
        manifest_rows.extend(manifest)
        timing_rows.extend(timing)
        invalid_rows.extend(invalid)

    out_dir = phase8 / "01_data_validation"
    out_dir.mkdir(parents=True, exist_ok=True)

    write_csv(out_dir / "episode_file_summary.csv", [dataclass_row(c) for c in checks])
    write_csv(out_dir / "integrated_manifest.csv", manifest_rows)
    write_csv(out_dir / "timing_statistics.csv", timing_rows)
    write_csv(out_dir / "invalid_samples.csv", invalid_rows)

    summary = {
        "status": "COMPLETED" if not invalid_rows else "PARTIALLY_COMPLETED",
        "real_episodes": REAL_EPISODES,
        "sim_episodes": SIM_EPISODES,
        "real_total_frames": sum(c.image_count for c in checks if c.domain == "real"),
        "sim_total_frames": sum(c.image_count for c in checks if c.domain == "sim"),
        "manifest_rows": len(manifest_rows),
        "invalid_episode_count": len(invalid_rows),
        "invalid_rows_path": str(out_dir / "invalid_samples.csv"),
        "condition_manifest": str(condition_manifest_path),
        "notes": [
            "File-level integrity only; feature extraction and observation metrics are not run here.",
            "Real pose source is planned_commanded_pose, not measured feedback.",
            "Sim replay is home-relative joint replay.",
        ],
    }
    (out_dir / "data_integrity_summary.json").write_text(json.dumps(summary, indent=2), encoding="utf-8")
    write_report(out_dir / "data_integrity_report.md", checks, invalid_rows)
    print(json.dumps(summary, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
