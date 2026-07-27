from __future__ import annotations

import argparse
import json
import math
import shutil
from bisect import bisect_left, bisect_right
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import numpy as np

from .make_actions import calculate_action, wrap_to_pi
from .rlds_utils import iter_episode_dirs, read_json, read_jsonl


@dataclass(frozen=True)
class ResampleStats:
    input_steps: int
    output_steps: int
    run_count: int
    gap_count: int
    dropped_single_sample_runs: int


def _as_pose(step: dict[str, Any]) -> np.ndarray:
    pose = np.asarray(step["tcp_pose"], dtype=np.float64)
    if pose.shape != (6,):
        raise ValueError("tcp_pose must have shape (6,)")
    if not np.isfinite(pose).all():
        raise ValueError("tcp_pose contains NaN/Inf")
    return pose


def _timestamp(step: dict[str, Any]) -> float:
    value = float(step["timestamp"])
    if not math.isfinite(value):
        raise ValueError("timestamp contains NaN/Inf")
    return value


def _split_runs(
    steps: list[dict[str, Any]],
    *,
    max_gap_sec: float,
) -> list[list[dict[str, Any]]]:
    if len(steps) < 2:
        return []

    runs: list[list[dict[str, Any]]] = [[steps[0]]]
    previous_timestamp = _timestamp(steps[0])
    for step in steps[1:]:
        timestamp = _timestamp(step)
        if timestamp <= previous_timestamp:
            raise ValueError("step timestamps must be strictly increasing")

        if timestamp - previous_timestamp > max_gap_sec:
            runs.append([step])
        else:
            runs[-1].append(step)
        previous_timestamp = timestamp

    return runs


def _interpolate_pose(
    before: dict[str, Any],
    after: dict[str, Any],
    ratio: float,
) -> np.ndarray:
    before_pose = _as_pose(before)
    after_pose = _as_pose(after)

    position = before_pose[:3] + ratio * (after_pose[:3] - before_pose[:3])
    rotation_delta = wrap_to_pi(after_pose[3:] - before_pose[3:])
    rotation = before_pose[3:] + ratio * rotation_delta
    rotation = wrap_to_pi(rotation)
    return np.concatenate([position, rotation])


def _pose_at_time(run: list[dict[str, Any]], timestamp: float) -> np.ndarray:
    timestamps = [_timestamp(step) for step in run]
    if timestamp <= timestamps[0]:
        return _as_pose(run[0])
    if timestamp >= timestamps[-1]:
        return _as_pose(run[-1])

    left_index = bisect_right(timestamps, timestamp) - 1
    right_index = left_index + 1
    left_time = timestamps[left_index]
    right_time = timestamps[right_index]
    ratio = (timestamp - left_time) / (right_time - left_time)
    return _interpolate_pose(run[left_index], run[right_index], ratio)


def _nearest_source_step(
    run: list[dict[str, Any]],
    timestamp: float,
) -> dict[str, Any]:
    timestamps = [_timestamp(step) for step in run]
    index = bisect_left(timestamps, timestamp)
    if index <= 0:
        return run[0]
    if index >= len(run):
        return run[-1]

    before = run[index - 1]
    after = run[index]
    if timestamp - _timestamp(before) <= _timestamp(after) - timestamp:
        return before
    return after


def _target_times_for_run(
    run: list[dict[str, Any]],
    *,
    period_sec: float,
) -> list[float]:
    start = _timestamp(run[0])
    end = _timestamp(run[-1])
    if end <= start:
        return []

    times: list[float] = []
    timestamp = start
    while timestamp < end - 1e-9:
        times.append(timestamp)
        timestamp += period_sec

    if not times or abs(times[-1] - end) > period_sec * 0.25:
        times.append(end)
    else:
        times[-1] = end
    return times


def _build_resampled_steps(
    steps: list[dict[str, Any]],
    *,
    target_hz: float,
    max_gap_sec: float,
    drop_single_sample_runs: bool,
) -> tuple[list[dict[str, Any]], ResampleStats]:
    if target_hz <= 0.0:
        raise ValueError("target_hz must be positive")
    if max_gap_sec <= 0.0:
        raise ValueError("max_gap_sec must be positive")

    period_sec = 1.0 / target_hz
    runs = _split_runs(steps, max_gap_sec=max_gap_sec)
    resampled: list[dict[str, Any]] = []
    dropped_single_sample_runs = 0

    for run_index, run in enumerate(runs):
        if len(run) < 2:
            if drop_single_sample_runs:
                dropped_single_sample_runs += 1
                continue
            target_times = [_timestamp(run[0])]
        else:
            target_times = _target_times_for_run(
                run,
                period_sec=period_sec,
            )

        for source_time in target_times:
            source_step = _nearest_source_step(run, source_time)
            step_index = len(resampled)
            resampled.append(
                {
                    "step_index": step_index,
                    "timestamp": step_index * period_sec,
                    "source_timestamp": source_time,
                    "source_step_index": int(source_step.get("step_index", 0)),
                    "resample_run_index": run_index,
                    "image_timestamp": source_step.get("image_timestamp"),
                    "image_age_sec": source_step.get("image_age_sec"),
                    "image": source_step["image"],
                    "tcp_pose": _pose_at_time(run, source_time)
                    .astype(float)
                    .tolist(),
                    "gripper": int(source_step["gripper"]),
                }
            )

    if len(resampled) < 2:
        raise ValueError("resampling produced fewer than two steps")

    stats = ResampleStats(
        input_steps=len(steps),
        output_steps=len(resampled),
        run_count=len(runs),
        gap_count=max(0, len(runs) - 1),
        dropped_single_sample_runs=dropped_single_sample_runs,
    )
    return resampled, stats


def _with_actions(steps: list[dict[str, Any]]) -> list[dict[str, Any]]:
    output = [dict(step) for step in steps]
    for index in range(len(output) - 1):
        current = output[index]
        following = output[index + 1]
        if current["resample_run_index"] != following["resample_run_index"]:
            current["action"] = [
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                float(current["gripper"]),
            ]
        else:
            current["action"] = calculate_action(current, following)

    output[-1]["action"] = [
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        float(output[-1]["gripper"]),
    ]
    return output


def _write_jsonl(path: Path, records: list[dict[str, Any]]) -> None:
    with path.open("w", encoding="utf-8") as file:
        for record in records:
            file.write(json.dumps(record, ensure_ascii=False) + "\n")


def _link_images(
    source_episode_dir: Path,
    output_episode_dir: Path,
    *,
    mode: str,
) -> None:
    source_images = source_episode_dir / "images"
    output_images = output_episode_dir / "images"
    if mode == "symlink":
        output_images.symlink_to(source_images, target_is_directory=True)
    elif mode == "copy":
        shutil.copytree(source_images, output_images)
    else:
        raise ValueError(f"unsupported image link mode: {mode}")


def resample_episode(
    source_episode_dir: Path,
    output_episode_dir: Path,
    *,
    target_hz: float = 10.0,
    max_gap_sec: float = 0.5,
    image_link_mode: str = "symlink",
    drop_single_sample_runs: bool = True,
) -> ResampleStats:
    metadata = read_json(source_episode_dir / "metadata.json")
    steps = read_jsonl(source_episode_dir / "steps.jsonl")
    resampled_steps, stats = _build_resampled_steps(
        steps,
        target_hz=target_hz,
        max_gap_sec=max_gap_sec,
        drop_single_sample_runs=drop_single_sample_runs,
    )
    resampled_steps_with_actions = _with_actions(resampled_steps)

    output_episode_dir.mkdir(parents=True, exist_ok=False)
    _link_images(
        source_episode_dir,
        output_episode_dir,
        mode=image_link_mode,
    )

    output_metadata = dict(metadata)
    output_metadata.update(
        {
            "num_steps": stats.output_steps,
            "record_frequency_hz": target_hz,
            "effective_record_frequency_hz": target_hz,
            "record_only_when_changed": False,
            "resampled": True,
            "resample_source_episode_dir": str(source_episode_dir),
            "resample_source_num_steps": stats.input_steps,
            "resample_target_hz": target_hz,
            "resample_period_sec": 1.0 / target_hz,
            "resample_max_gap_sec": max_gap_sec,
            "resample_run_count": stats.run_count,
            "resample_gap_count": stats.gap_count,
            "resample_dropped_single_sample_runs": (
                stats.dropped_single_sample_runs
            ),
        }
    )

    (output_episode_dir / "metadata.json").write_text(
        json.dumps(output_metadata, ensure_ascii=False, indent=2),
        encoding="utf-8",
    )
    _write_jsonl(output_episode_dir / "steps.jsonl", resampled_steps)
    _write_jsonl(
        output_episode_dir / "steps_with_actions.jsonl",
        resampled_steps_with_actions,
    )
    return stats


def resample_dataset(
    source_root: Path,
    output_root: Path,
    *,
    target_hz: float = 10.0,
    max_gap_sec: float = 0.5,
    image_link_mode: str = "symlink",
    drop_single_sample_runs: bool = True,
) -> list[tuple[str, ResampleStats]]:
    source_root = source_root.expanduser().resolve()
    output_root = output_root.expanduser().resolve()
    if output_root.exists():
        raise FileExistsError(f"Output directory already exists: {output_root}")
    output_root.mkdir(parents=True)

    results: list[tuple[str, ResampleStats]] = []
    for source_episode_dir in iter_episode_dirs(source_root):
        output_episode_dir = output_root / source_episode_dir.name
        stats = resample_episode(
            source_episode_dir,
            output_episode_dir,
            target_hz=target_hz,
            max_gap_sec=max_gap_sec,
            image_link_mode=image_link_mode,
            drop_single_sample_runs=drop_single_sample_runs,
        )
        results.append((source_episode_dir.name, stats))

    return results


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Resample recorded Doosan raw episodes to a fixed Hz."
    )
    parser.add_argument("source_root", type=Path)
    parser.add_argument("output_root", type=Path)
    parser.add_argument("--target-hz", type=float, default=10.0)
    parser.add_argument(
        "--max-gap-sec",
        type=float,
        default=0.5,
        help="Do not interpolate across larger timestamp gaps.",
    )
    parser.add_argument(
        "--image-link-mode",
        choices=("symlink", "copy"),
        default="symlink",
        help="How output episodes should reference source image folders.",
    )
    parser.add_argument(
        "--keep-single-sample-runs",
        action="store_true",
        help="Keep isolated one-sample runs instead of dropping them.",
    )
    args = parser.parse_args()

    results = resample_dataset(
        args.source_root,
        args.output_root,
        target_hz=args.target_hz,
        max_gap_sec=args.max_gap_sec,
        image_link_mode=args.image_link_mode,
        drop_single_sample_runs=not args.keep_single_sample_runs,
    )

    input_steps = sum(stats.input_steps for _, stats in results)
    output_steps = sum(stats.output_steps for _, stats in results)
    gaps = sum(stats.gap_count for _, stats in results)
    dropped = sum(stats.dropped_single_sample_runs for _, stats in results)
    print(f"Source episodes : {len(results)}")
    print(f"Source steps    : {input_steps}")
    print(f"Output steps    : {output_steps}")
    print(f"Skipped gaps    : {gaps}")
    print(f"Dropped runs    : {dropped}")
    print(f"Saved           : {args.output_root.expanduser().resolve()}")


if __name__ == "__main__":
    main()
