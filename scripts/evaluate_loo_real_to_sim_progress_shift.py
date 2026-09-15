#!/usr/bin/env python3
from __future__ import annotations

import argparse
import csv
import json
import math
import re
from pathlib import Path
from typing import Any

import numpy as np


ROOT = Path("/home/ubuntu/a0509_vla_linux_field_bundle_20260903")
DEFAULT_ANALYSIS_ROOT = ROOT / "outputs" / "token_distribution_analysis" / "5_episodes"
DEFAULT_OUTPUT_DIR = ROOT / "lhj" / "phase1_audit" / "loo_real_to_sim_progress_shift"
FEATURES = ["vision_backbone.output", "projector.output"]


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--analysis-root", type=Path, default=DEFAULT_ANALYSIS_ROOT)
    parser.add_argument("--output-dir", type=Path, default=DEFAULT_OUTPUT_DIR)
    parser.add_argument("--window", type=int, default=11)
    return parser.parse_args()


def read_json(path: Path) -> Any:
    return json.loads(path.read_text(encoding="utf-8"))


def write_json(path: Path, payload: Any) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")


def frame_id(record: dict[str, Any]) -> str:
    return Path(record["source_image"]).stem


def records_by_frame(manifest_path: Path) -> dict[str, dict[str, Any]]:
    manifest = read_json(manifest_path)
    base = manifest_path.parent
    return {
        frame_id(record): {**record, "_feature_path": str(base / record["feature_file"])}
        for record in manifest["records"]
    }


def episode_id_from_frame(frame: str) -> str:
    match = re.match(r"(episode_\d{6})_", str(frame))
    if not match:
        raise ValueError(f"Could not parse episode id from frame {frame!r}")
    return match.group(1)


def progress_from_frame(frame: str) -> int:
    match = re.search(r"(\d+)$", str(frame))
    if not match:
        raise ValueError(f"Could not parse progress from frame {frame!r}")
    return int(match.group(1))


def load_pooled(path: Path, feature: str) -> np.ndarray:
    with np.load(path, allow_pickle=True) as data:
        arr = np.asarray(data[feature], dtype=np.float64)
    if arr.ndim == 3:
        return arr.mean(axis=(0, 1))
    if arr.ndim == 2:
        return arr.mean(axis=0)
    return arr.reshape(-1)


def cosine_distance(left: np.ndarray, right: np.ndarray) -> float:
    denom = float(np.linalg.norm(left) * np.linalg.norm(right))
    if denom <= 1e-12:
        return 1.0
    return float(1.0 - np.dot(left, right) / denom)


def squared_distances(left: np.ndarray, right: np.ndarray) -> np.ndarray:
    left_norm = np.sum(left * left, axis=1, keepdims=True)
    right_norm = np.sum(right * right, axis=1, keepdims=True).T
    return np.maximum(left_norm + right_norm - 2.0 * left @ right.T, 0.0)


def mmd_rbf(left: np.ndarray, right: np.ndarray) -> float:
    combined = np.vstack([left, right])
    sq = squared_distances(combined, combined)
    offdiag = sq[~np.eye(sq.shape[0], dtype=bool)]
    positive = offdiag[offdiag > 0]
    sigma2 = float(np.median(positive)) if positive.size else 1.0
    if not math.isfinite(sigma2) or sigma2 <= 1e-12:
        sigma2 = 1.0
    gamma = 1.0 / (2.0 * sigma2)
    return float(
        np.exp(-gamma * squared_distances(left, left)).mean()
        + np.exp(-gamma * squared_distances(right, right)).mean()
        - 2.0 * np.exp(-gamma * squared_distances(left, right)).mean()
    )


def summarize(values: list[float]) -> dict[str, float | int]:
    arr = np.asarray(values, dtype=np.float64)
    return {
        "count": int(arr.size),
        "mean": float(arr.mean()),
        "std": float(arr.std()),
        "min": float(arr.min()),
        "p50": float(np.percentile(arr, 50)),
        "p90": float(np.percentile(arr, 90)),
        "max": float(arr.max()),
    }


def nearest_shift(
    train_progress: np.ndarray,
    train_shift: np.ndarray,
    test_progress_value: int,
    window: int,
) -> np.ndarray:
    half = window // 2
    near = np.where(np.abs(train_progress - test_progress_value) <= half)[0]
    if near.size == 0:
        near = np.asarray([int(np.argmin(np.abs(train_progress - test_progress_value)))])
    return train_shift[near].mean(axis=0)


def main() -> int:
    args = parse_args()
    if args.window < 3 or args.window % 2 == 0:
        raise ValueError("--window must be odd and >= 3")
    analysis_root = args.analysis_root.expanduser().resolve()
    output_dir = args.output_dir.expanduser().resolve()
    output_dir.mkdir(parents=True, exist_ok=True)

    real_records = records_by_frame(analysis_root / "features" / "real" / "feature_manifest.json")
    sim_records = records_by_frame(analysis_root / "features" / "sim" / "feature_manifest.json")
    frames = sorted(set(real_records) & set(sim_records))
    episode_ids = sorted({episode_id_from_frame(frame) for frame in frames})

    all_rows: list[dict[str, Any]] = []
    summary: dict[str, Any] = {
        "method": "leave-one-episode-out real_to_sim progress shift",
        "analysis_root": str(analysis_root),
        "output_dir": str(output_dir),
        "window": args.window,
        "episode_ids": episode_ids,
        "paired_frame_count": len(frames),
        "features": {},
    }

    for feature in FEATURES:
        vectors: dict[str, tuple[np.ndarray, np.ndarray]] = {}
        for frame in frames:
            vectors[frame] = (
                load_pooled(Path(real_records[frame]["_feature_path"]), feature),
                load_pooled(Path(sim_records[frame]["_feature_path"]), feature),
            )

        feature_rows: list[dict[str, Any]] = []
        before_cos_all: list[float] = []
        after_cos_all: list[float] = []
        before_l2_all: list[float] = []
        after_l2_all: list[float] = []
        corrected_all: list[np.ndarray] = []
        sim_all: list[np.ndarray] = []
        real_all: list[np.ndarray] = []

        for test_episode in episode_ids:
            train_frames = [frame for frame in frames if episode_id_from_frame(frame) != test_episode]
            test_frames = [frame for frame in frames if episode_id_from_frame(frame) == test_episode]
            train_progress = np.asarray([progress_from_frame(frame) for frame in train_frames], dtype=np.int64)
            train_real = np.vstack([vectors[frame][0] for frame in train_frames])
            train_sim = np.vstack([vectors[frame][1] for frame in train_frames])
            train_shift = train_sim - train_real

            for frame in test_frames:
                progress = progress_from_frame(frame)
                real, sim = vectors[frame]
                shift = nearest_shift(train_progress, train_shift, progress, args.window)
                corrected = real + shift
                before_cos = cosine_distance(real, sim)
                after_cos = cosine_distance(corrected, sim)
                before_l2 = float(np.linalg.norm(real - sim))
                after_l2 = float(np.linalg.norm(corrected - sim))
                row = {
                    "feature": feature,
                    "frame": frame,
                    "episode_id": test_episode,
                    "progress": progress,
                    "before_cosine_distance": before_cos,
                    "after_cosine_distance": after_cos,
                    "cosine_improvement": before_cos - after_cos,
                    "before_l2": before_l2,
                    "after_l2": after_l2,
                    "l2_improvement": before_l2 - after_l2,
                    "shift_norm": float(np.linalg.norm(shift)),
                }
                feature_rows.append(row)
                all_rows.append(row)
                before_cos_all.append(before_cos)
                after_cos_all.append(after_cos)
                before_l2_all.append(before_l2)
                after_l2_all.append(after_l2)
                corrected_all.append(corrected)
                real_all.append(real)
                sim_all.append(sim)

        real_matrix = np.vstack(real_all)
        sim_matrix = np.vstack(sim_all)
        corrected_matrix = np.vstack(corrected_all)
        summary["features"][feature] = {
            "before_frame_cosine": summarize(before_cos_all),
            "after_frame_cosine": summarize(after_cos_all),
            "frame_cosine_improvement": summarize([b - a for b, a in zip(before_cos_all, after_cos_all)]),
            "before_frame_l2": summarize(before_l2_all),
            "after_frame_l2": summarize(after_l2_all),
            "frame_l2_improvement": summarize([b - a for b, a in zip(before_l2_all, after_l2_all)]),
            "before_mmd": mmd_rbf(real_matrix, sim_matrix),
            "after_mmd": mmd_rbf(corrected_matrix, sim_matrix),
        }

    with (output_dir / "loo_frame_metrics.csv").open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(all_rows[0].keys()))
        writer.writeheader()
        writer.writerows(all_rows)
    write_json(output_dir / "loo_summary.json", summary)
    print(json.dumps(summary["features"], indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
