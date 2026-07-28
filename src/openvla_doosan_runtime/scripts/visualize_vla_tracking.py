#!/usr/bin/env python3
from __future__ import annotations

import argparse
import bisect
import json
import math
import re
import sqlite3
from pathlib import Path

import matplotlib

matplotlib.use("Agg")

import matplotlib.pyplot as plt
import numpy as np
from rclpy.serialization import deserialize_message
from tf2_msgs.msg import TFMessage


ROBOT_CHAIN = (
    ("base_link", "link_1"),
    ("link_1", "link_2"),
    ("link_2", "link_3"),
    ("link_3", "link_4"),
    ("link_4", "link_5"),
    ("link_5", "link_6"),
)


def _qmul(left, right):
    ax, ay, az, aw = left
    bx, by, bz, bw = right
    return (
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz,
    )


def _qrot(quaternion, vector):
    x, y, z, w = quaternion
    vx, vy, vz = vector
    tx = 2.0 * (y * vz - z * vy)
    ty = 2.0 * (z * vx - x * vz)
    tz = 2.0 * (x * vy - y * vx)
    return (
        vx + w * tx + (y * tz - z * ty),
        vy + w * ty + (z * tx - x * tz),
        vz + w * tz + (x * ty - y * tx),
    )


def _compose(parent, child):
    parent_translation, parent_rotation = parent
    child_translation, child_rotation = child
    rotated_child_translation = _qrot(parent_rotation, child_translation)
    return (
        (
            parent_translation[0] + rotated_child_translation[0],
            parent_translation[1] + rotated_child_translation[1],
            parent_translation[2] + rotated_child_translation[2],
        ),
        _qmul(parent_rotation, child_rotation),
    )


def _norm(vector: np.ndarray) -> float:
    return float(np.linalg.norm(vector))


def _finite_mean(values: np.ndarray) -> float | None:
    finite = values[np.isfinite(values)]

    if len(finite) == 0:
        return None

    return float(np.mean(finite))


def _finite_positive_ratio(values: np.ndarray) -> float | None:
    finite = values[np.isfinite(values)]

    if len(finite) == 0:
        return None

    return float(np.mean(finite > 0.0))


def _read_link6_path(bag_db: Path) -> tuple[np.ndarray, np.ndarray]:
    connection = sqlite3.connect(str(bag_db))
    cursor = connection.cursor()

    topic_ids = {
        row[1]: row[0]
        for row in cursor.execute("select id, name from topics")
    }

    tf_topic_id = topic_ids.get("/tf")
    if tf_topic_id is None:
        raise RuntimeError("Bag does not contain /tf")

    positions = []

    for (serialized,) in cursor.execute(
        "select data from messages where topic_id=? order by timestamp",
        (tf_topic_id,),
    ):
        message = deserialize_message(serialized, TFMessage)
        transforms = {}
        stamp = None

        for transform in message.transforms:
            key = (
                transform.header.frame_id,
                transform.child_frame_id,
            )

            translation = transform.transform.translation
            rotation = transform.transform.rotation

            transforms[key] = (
                (
                    translation.x,
                    translation.y,
                    translation.z,
                ),
                (
                    rotation.x,
                    rotation.y,
                    rotation.z,
                    rotation.w,
                ),
            )

            if key == ("base_link", "link_1"):
                stamp = (
                    transform.header.stamp.sec
                    + transform.header.stamp.nanosec * 1.0e-9
                )

        if stamp is None:
            continue

        if not all(key in transforms for key in ROBOT_CHAIN):
            continue

        pose = (
            (0.0, 0.0, 0.0),
            (0.0, 0.0, 0.0, 1.0),
        )

        for key in ROBOT_CHAIN:
            pose = _compose(pose, transforms[key])

        positions.append((stamp, pose[0]))

    connection.close()

    deduped = sorted(
        {
            round(stamp, 9): position
            for stamp, position in positions
        }.items()
    )

    if not deduped:
        raise RuntimeError("No complete base_link -> link_6 TF chain found")

    times = np.asarray(
        [stamp for stamp, _position in deduped],
        dtype=np.float64,
    )
    xyz_mm = (
        np.asarray(
            [position for _stamp, position in deduped],
            dtype=np.float64,
        )
        * 1000.0
    )

    return times, xyz_mm


def _read_actions(log_path: Path) -> list[tuple[float, list[float]]]:
    text = log_path.read_text(
        encoding="utf-8",
        errors="replace",
    )

    pattern = re.compile(
        r"stamp=([0-9]+\.[0-9]+).*?action=\[([^\]]+)\]",
        re.S,
    )

    actions = []

    for match in pattern.finditer(text):
        stamp = float(match.group(1))
        values = [
            float(value)
            for value in re.split(
                r"\s+",
                match.group(2).strip(),
            )
            if value
        ]

        if len(values) == 7:
            actions.append((stamp, values))

    latest_by_stamp = {}

    for stamp, values in actions:
        latest_by_stamp[round(stamp, 6)] = (stamp, values)

    return [
        latest_by_stamp[key]
        for key in sorted(latest_by_stamp)
    ]


def _longest_contiguous_segment(
    actions: list[tuple[float, list[float]]],
    max_gap_sec: float,
) -> list[tuple[float, list[float]]]:
    segments = []
    current = []
    previous_stamp = None

    for item in actions:
        stamp = item[0]

        if (
            previous_stamp is not None
            and stamp - previous_stamp > max_gap_sec
        ):
            if current:
                segments.append(current)
            current = []

        current.append(item)
        previous_stamp = stamp

    if current:
        segments.append(current)

    if not segments:
        return []

    return max(
        segments,
        key=len,
    )


def _nearest_index(times: np.ndarray, stamp: float) -> int:
    index = bisect.bisect_left(
        times,
        stamp,
    )

    if index <= 0:
        return 0

    if index >= len(times):
        return len(times) - 1

    before = index - 1

    if abs(times[before] - stamp) <= abs(times[index] - stamp):
        return before

    return index


def _aligned_targets(
    times: np.ndarray,
    xyz_mm: np.ndarray,
    actions: list[tuple[float, list[float]]],
    horizon_sec: float,
    translation_scale_to_mm: float,
    max_translation_step_mm: float,
    axis_sign: np.ndarray,
):
    action_times = []
    current_points = []
    target_points = []
    future_points = []
    deltas = []
    gripper_values = []
    target_errors = []
    baseline_errors = []
    cosines = []
    clipped_count = 0

    for stamp, values in actions:
        if stamp < times[0]:
            continue

        if stamp + horizon_sec > times[-1]:
            continue

        current_index = _nearest_index(
            times,
            stamp,
        )
        future_index = _nearest_index(
            times,
            stamp + horizon_sec,
        )

        current = xyz_mm[current_index]
        future = xyz_mm[future_index]

        raw_delta = (
            np.asarray(
                values[:3],
                dtype=np.float64,
            )
            * axis_sign
            * translation_scale_to_mm
        )

        if np.any(np.abs(raw_delta) > max_translation_step_mm):
            clipped_count += 1

        delta = np.clip(
            raw_delta,
            -max_translation_step_mm,
            max_translation_step_mm,
        )

        target = current + delta
        future_delta = future - current

        action_times.append(stamp)
        current_points.append(current)
        target_points.append(target)
        future_points.append(future)
        deltas.append(delta)
        gripper_values.append(float(values[6]))
        target_errors.append(_norm(target - future))
        baseline_errors.append(_norm(future_delta))

        if _norm(delta) > 1.0e-9 and _norm(future_delta) > 1.0e-9:
            cosines.append(
                float(
                    np.dot(delta, future_delta)
                    / (_norm(delta) * _norm(future_delta))
                )
            )
        else:
            cosines.append(float("nan"))

    if not action_times:
        raise RuntimeError("No aligned action samples found")

    return {
        "times": np.asarray(action_times, dtype=np.float64),
        "current": np.asarray(current_points, dtype=np.float64),
        "target": np.asarray(target_points, dtype=np.float64),
        "future": np.asarray(future_points, dtype=np.float64),
        "delta": np.asarray(deltas, dtype=np.float64),
        "gripper": np.asarray(gripper_values, dtype=np.float64),
        "target_errors": np.asarray(target_errors, dtype=np.float64),
        "baseline_errors": np.asarray(baseline_errors, dtype=np.float64),
        "cosines": np.asarray(cosines, dtype=np.float64),
        "clipped_count": clipped_count,
    }


def _make_plot(
    path_times: np.ndarray,
    path_xyz_mm: np.ndarray,
    aligned,
    horizon_sec: float,
    output_png: Path,
    summary: dict,
) -> None:
    action_times = aligned["times"]
    target = aligned["target"]
    current = aligned["current"]

    start = max(
        path_times[0],
        float(action_times.min()) - 1.0,
    )
    end = min(
        path_times[-1],
        float(action_times.max()) + 1.0,
    )
    mask = np.logical_and(
        path_times >= start,
        path_times <= end,
    )
    path = path_xyz_mm[mask]

    relative_time = action_times - action_times.min()

    figure = plt.figure(
        figsize=(14, 10),
        constrained_layout=True,
    )

    grid = figure.add_gridspec(
        2,
        2,
    )

    ax_xy = figure.add_subplot(grid[0, 0])
    ax_xz = figure.add_subplot(grid[0, 1])
    ax_3d = figure.add_subplot(
        grid[1, 0],
        projection="3d",
    )
    ax_error = figure.add_subplot(grid[1, 1])

    ax_xy.plot(
        path[:, 0],
        path[:, 1],
        color="#1f77b4",
        linewidth=2.0,
        label="Actual link_6 path",
    )
    ax_xy.plot(
        target[:, 0],
        target[:, 1],
        color="#d62728",
        linewidth=1.4,
        marker="o",
        markersize=2.4,
        alpha=0.78,
        label="OpenVLA target points",
    )
    ax_xy.scatter(
        current[0, 0],
        current[0, 1],
        color="green",
        s=45,
        label="segment start",
    )
    ax_xy.set_title("XY path")
    ax_xy.set_xlabel("X [mm]")
    ax_xy.set_ylabel("Y [mm]")
    ax_xy.axis("equal")
    ax_xy.grid(True, alpha=0.25)
    ax_xy.legend(fontsize=8)

    ax_xz.plot(
        path[:, 0],
        path[:, 2],
        color="#1f77b4",
        linewidth=2.0,
        label="Actual link_6 path",
    )
    ax_xz.plot(
        target[:, 0],
        target[:, 2],
        color="#d62728",
        linewidth=1.4,
        marker="o",
        markersize=2.4,
        alpha=0.78,
        label="OpenVLA target points",
    )
    ax_xz.scatter(
        current[0, 0],
        current[0, 2],
        color="green",
        s=45,
        label="segment start",
    )
    ax_xz.set_title("XZ path")
    ax_xz.set_xlabel("X [mm]")
    ax_xz.set_ylabel("Z [mm]")
    ax_xz.axis("equal")
    ax_xz.grid(True, alpha=0.25)
    ax_xz.legend(fontsize=8)

    ax_3d.plot(
        path[:, 0],
        path[:, 1],
        path[:, 2],
        color="#1f77b4",
        linewidth=2.0,
        label="Actual link_6 path",
    )
    ax_3d.plot(
        target[:, 0],
        target[:, 1],
        target[:, 2],
        color="#d62728",
        linewidth=1.1,
        marker="o",
        markersize=2.0,
        alpha=0.75,
        label="OpenVLA target points",
    )
    ax_3d.set_title("3D path")
    ax_3d.set_xlabel("X [mm]")
    ax_3d.set_ylabel("Y [mm]")
    ax_3d.set_zlabel("Z [mm]")
    ax_3d.legend(fontsize=8)

    ax_error.plot(
        relative_time,
        aligned["target_errors"],
        color="#d62728",
        label=f"target vs actual +{horizon_sec:.1f}s",
    )
    ax_error.plot(
        relative_time,
        aligned["baseline_errors"],
        color="#7f7f7f",
        alpha=0.8,
        label="baseline: current vs actual future",
    )
    ax_error.set_title("One-step tracking error")
    ax_error.set_xlabel("Action segment time [s]")
    ax_error.set_ylabel("Distance [mm]")
    ax_error.grid(True, alpha=0.25)
    ax_error.legend(fontsize=8)

    figure.suptitle(
        "OpenVLA Doosan Tracking Check\n"
        f"aligned actions={summary['action_samples_aligned']} | "
        f"mean target error={summary['mean_target_error_mm']:.1f} mm | "
        f"mean direction cosine={summary['mean_direction_cosine']:.2f}",
        fontsize=13,
    )

    figure.savefig(
        output_png,
        dpi=160,
    )
    plt.close(figure)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--bag-db", type=Path, required=True)
    parser.add_argument("--log", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--summary", type=Path, required=True)
    parser.add_argument("--horizon-sec", type=float, default=0.5)
    parser.add_argument("--translation-scale-to-mm", type=float, default=1000.0)
    parser.add_argument("--max-translation-step-mm", type=float, default=10.0)
    parser.add_argument("--moving-threshold-mm", type=float, default=2.0)
    parser.add_argument(
        "--axis-sign",
        type=float,
        nargs=3,
        default=[1.0, 1.0, 1.0],
    )
    parser.add_argument("--max-action-gap-sec", type=float, default=2.0)
    args = parser.parse_args()

    times, xyz_mm = _read_link6_path(
        args.bag_db.expanduser().resolve()
    )
    actions = _read_actions(
        args.log.expanduser().resolve()
    )
    actions = _longest_contiguous_segment(
        actions,
        max_gap_sec=args.max_action_gap_sec,
    )

    aligned = _aligned_targets(
        times=times,
        xyz_mm=xyz_mm,
        actions=actions,
        horizon_sec=args.horizon_sec,
        translation_scale_to_mm=args.translation_scale_to_mm,
        max_translation_step_mm=args.max_translation_step_mm,
        axis_sign=np.asarray(args.axis_sign, dtype=np.float64),
    )

    target_errors = aligned["target_errors"]
    baseline_errors = aligned["baseline_errors"]
    cosines = aligned["cosines"]
    step_lengths = np.linalg.norm(
        aligned["delta"],
        axis=1,
    )
    moving_mask = baseline_errors >= args.moving_threshold_mm
    moving_target_errors = target_errors[moving_mask]
    moving_baseline_errors = baseline_errors[moving_mask]
    moving_cosines = cosines[moving_mask]

    summary = {
        "bag_db": str(args.bag_db.expanduser().resolve()),
        "log": str(args.log.expanduser().resolve()),
        "plot": str(args.output.expanduser().resolve()),
        "tf_samples": int(len(times)),
        "action_samples_aligned": int(len(aligned["times"])),
        "action_time_range": [
            float(aligned["times"].min()),
            float(aligned["times"].max()),
        ],
        "horizon_sec": float(args.horizon_sec),
        "translation_scale_to_mm": float(args.translation_scale_to_mm),
        "max_translation_step_mm": float(args.max_translation_step_mm),
        "moving_threshold_mm": float(args.moving_threshold_mm),
        "axis_sign": [float(value) for value in args.axis_sign],
        "mean_target_error_mm": float(np.mean(target_errors)),
        "median_target_error_mm": float(np.median(target_errors)),
        "p90_target_error_mm": float(np.quantile(target_errors, 0.9)),
        "mean_baseline_future_motion_mm": float(np.mean(baseline_errors)),
        "median_baseline_future_motion_mm": float(np.median(baseline_errors)),
        "improved_over_baseline_ratio": float(
            np.mean(target_errors < baseline_errors)
        ),
        "mean_direction_cosine": _finite_mean(cosines),
        "positive_direction_cosine_ratio": _finite_positive_ratio(cosines),
        "moving_samples": int(np.sum(moving_mask)),
        "moving_sample_ratio": float(np.mean(moving_mask)),
        "moving_mean_target_error_mm": float(np.mean(moving_target_errors))
        if len(moving_target_errors)
        else None,
        "moving_median_target_error_mm": float(np.median(moving_target_errors))
        if len(moving_target_errors)
        else None,
        "moving_mean_baseline_future_motion_mm": float(
            np.mean(moving_baseline_errors)
        )
        if len(moving_baseline_errors)
        else None,
        "moving_improved_over_baseline_ratio": float(
            np.mean(moving_target_errors < moving_baseline_errors)
        )
        if len(moving_target_errors)
        else None,
        "moving_mean_direction_cosine": _finite_mean(moving_cosines),
        "moving_positive_direction_cosine_ratio": _finite_positive_ratio(
            moving_cosines
        ),
        "clipped_ratio": float(
            aligned["clipped_count"] / len(aligned["times"])
        ),
        "median_step_mm": float(np.median(step_lengths)),
        "gripper_open_ratio": float(np.mean(aligned["gripper"] >= 0.7)),
    }

    args.output.expanduser().resolve().parent.mkdir(
        parents=True,
        exist_ok=True,
    )
    args.summary.expanduser().resolve().parent.mkdir(
        parents=True,
        exist_ok=True,
    )

    _make_plot(
        path_times=times,
        path_xyz_mm=xyz_mm,
        aligned=aligned,
        horizon_sec=args.horizon_sec,
        output_png=args.output.expanduser().resolve(),
        summary=summary,
    )

    args.summary.expanduser().resolve().write_text(
        json.dumps(summary, indent=2),
        encoding="utf-8",
    )

    print(json.dumps(summary, indent=2))


if __name__ == "__main__":
    main()
