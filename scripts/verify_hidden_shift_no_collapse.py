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
DEFAULT_ACTION_METRICS = (
    ROOT
    / "lhj"
    / "phase1_action_gap"
    / "policy_relevant_progress_shift"
    / "policy_relevant_progress_shift_frame_metrics.csv"
)
DEFAULT_OUTPUT_DIR = ROOT / "lhj" / "phase2_validation" / "hidden_shift_no_collapse"
FEATURE = "action_hidden_states.input"


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--analysis-root", type=Path, default=DEFAULT_ANALYSIS_ROOT)
    parser.add_argument("--action-metrics", type=Path, default=DEFAULT_ACTION_METRICS)
    parser.add_argument("--output-dir", type=Path, default=DEFAULT_OUTPUT_DIR)
    parser.add_argument("--window", type=int, default=11)
    parser.add_argument("--max-pairwise-frames", type=int, default=225)
    return parser.parse_args()


def read_json(path: Path) -> Any:
    return json.loads(path.read_text(encoding="utf-8"))


def write_json(path: Path, payload: Any) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")


def frame_from_record(record: dict[str, Any]) -> str:
    return Path(record["source_image"]).stem


def records_by_frame(manifest_path: Path) -> dict[str, dict[str, Any]]:
    manifest = read_json(manifest_path)
    base = manifest_path.parent
    return {
        frame_from_record(record): {
            **record,
            "_feature_path": str(base / record["feature_file"]),
        }
        for record in manifest["records"]
    }


def episode_id_from_frame(frame: str) -> str:
    match = re.match(r"(episode_\d{6})_", frame)
    if not match:
        raise ValueError(f"Could not parse episode id from {frame!r}")
    return match.group(1)


def progress_from_frame(frame: str) -> int:
    match = re.search(r"(\d+)$", frame)
    if not match:
        raise ValueError(f"Could not parse progress from {frame!r}")
    return int(match.group(1))


def load_feature(record: dict[str, Any]) -> np.ndarray:
    with np.load(record["_feature_path"], allow_pickle=True) as data:
        return np.asarray(data[FEATURE], dtype=np.float32)


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


def summarize(values: np.ndarray | list[float]) -> dict[str, float | int]:
    arr = np.asarray(values, dtype=np.float64).reshape(-1)
    return {
        "count": int(arr.size),
        "mean": float(arr.mean()),
        "std": float(arr.std()),
        "min": float(arr.min()),
        "p50": float(np.percentile(arr, 50)),
        "p90": float(np.percentile(arr, 90)),
        "max": float(arr.max()),
    }


def cosine_distance_rows(left: np.ndarray, right: np.ndarray) -> np.ndarray:
    denom = np.linalg.norm(left, axis=1) * np.linalg.norm(right, axis=1)
    denom = np.maximum(denom, 1e-12)
    return 1.0 - np.sum(left * right, axis=1) / denom


def pairwise_distances(matrix: np.ndarray) -> np.ndarray:
    sq_norm = np.sum(matrix * matrix, axis=1, keepdims=True)
    sq = np.maximum(sq_norm + sq_norm.T - 2.0 * matrix @ matrix.T, 0.0)
    return np.sqrt(sq)


def upper_triangle_values(matrix: np.ndarray) -> np.ndarray:
    idx = np.triu_indices(matrix.shape[0], k=1)
    return matrix[idx]


def pearson(left: np.ndarray, right: np.ndarray) -> float | None:
    x = np.asarray(left, dtype=np.float64).reshape(-1)
    y = np.asarray(right, dtype=np.float64).reshape(-1)
    mask = np.isfinite(x) & np.isfinite(y)
    x = x[mask]
    y = y[mask]
    if x.size < 3:
        return None
    x = x - x.mean()
    y = y - y.mean()
    denom = float(np.linalg.norm(x) * np.linalg.norm(y))
    if denom <= 1e-12:
        return None
    return float(np.dot(x, y) / denom)


def read_action_metrics(path: Path) -> list[dict[str, Any]]:
    with path.open("r", encoding="utf-8", newline="") as handle:
        return list(csv.DictReader(handle))


def group_action_improvements(rows: list[dict[str, Any]], key: str) -> list[dict[str, Any]]:
    groups: dict[str, list[dict[str, Any]]] = {}
    for row in rows:
        groups.setdefault(str(row[key]), []).append(row)
    output = []
    for group, items in sorted(groups.items()):
        raw = np.asarray([float(item["raw_chunk_mean_l2"]) for item in items], dtype=np.float64)
        shifted = np.asarray([float(item["hidden_shift_chunk_mean_l2"]) for item in items], dtype=np.float64)
        output.append(
            {
                key: group,
                "count": len(items),
                "raw_chunk_mean_l2": float(raw.mean()),
                "hidden_shift_chunk_mean_l2": float(shifted.mean()),
                "absolute_reduction": float(raw.mean() - shifted.mean()),
                "relative_reduction": float((raw.mean() - shifted.mean()) / raw.mean())
                if abs(raw.mean()) > 1e-12
                else None,
                "worse_frame_count": int(np.sum(shifted > raw)),
            }
        )
    return output


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
    episodes = sorted({episode_id_from_frame(frame) for frame in frames})

    real = {frame: load_feature(real_records[frame]) for frame in frames}
    sim = {frame: load_feature(sim_records[frame]) for frame in frames}
    corrected: dict[str, np.ndarray] = {}
    shift_norms: dict[str, float] = {}

    for test_episode in episodes:
        train_frames = [frame for frame in frames if episode_id_from_frame(frame) != test_episode]
        test_frames = [frame for frame in frames if episode_id_from_frame(frame) == test_episode]
        train_progress = np.asarray([progress_from_frame(frame) for frame in train_frames], dtype=np.int64)
        train_shift = np.stack([(sim[frame] - real[frame]).reshape(-1) for frame in train_frames])
        for frame in test_frames:
            shift = nearest_shift(
                train_progress,
                train_shift,
                progress_from_frame(frame),
                args.window,
            ).reshape(real[frame].shape)
            corrected[frame] = real[frame] + shift
            shift_norms[frame] = float(np.linalg.norm(shift))

    real_flat = np.stack([real[frame].reshape(-1) for frame in frames]).astype(np.float64)
    sim_flat = np.stack([sim[frame].reshape(-1) for frame in frames]).astype(np.float64)
    corrected_flat = np.stack([corrected[frame].reshape(-1) for frame in frames]).astype(np.float64)

    before_delta = real_flat - sim_flat
    after_delta = corrected_flat - sim_flat
    paired = {
        "before_l2": summarize(np.linalg.norm(before_delta, axis=1)),
        "after_l2": summarize(np.linalg.norm(after_delta, axis=1)),
        "before_cosine_distance": summarize(cosine_distance_rows(real_flat, sim_flat)),
        "after_cosine_distance": summarize(cosine_distance_rows(corrected_flat, sim_flat)),
        "shift_norm": summarize(list(shift_norms.values())),
        "worse_l2_frame_count": int(np.sum(np.linalg.norm(after_delta, axis=1) > np.linalg.norm(before_delta, axis=1))),
    }

    variance = {}
    for name, matrix in (("real", real_flat), ("sim", sim_flat), ("corrected", corrected_flat)):
        feature_var = np.var(matrix, axis=0)
        frame_norm = np.linalg.norm(matrix - matrix.mean(axis=0, keepdims=True), axis=1)
        variance[name] = {
            "feature_variance_mean": float(feature_var.mean()),
            "feature_variance_sum": float(feature_var.sum()),
            "feature_variance_min": float(feature_var.min()),
            "feature_variance_max": float(feature_var.max()),
            "centered_frame_norm": summarize(frame_norm),
        }

    pairwise = {}
    limit = min(args.max_pairwise_frames, len(frames))
    subset = np.linspace(0, len(frames) - 1, limit, dtype=int)
    real_pair = upper_triangle_values(pairwise_distances(real_flat[subset]))
    sim_pair = upper_triangle_values(pairwise_distances(sim_flat[subset]))
    corrected_pair = upper_triangle_values(pairwise_distances(corrected_flat[subset]))
    pairwise["distance_summary"] = {
        "real": summarize(real_pair),
        "sim": summarize(sim_pair),
        "corrected": summarize(corrected_pair),
    }
    pairwise["structure_correlation"] = {
        "corrected_vs_real": pearson(corrected_pair, real_pair),
        "corrected_vs_sim": pearson(corrected_pair, sim_pair),
        "real_vs_sim": pearson(real_pair, sim_pair),
    }

    action_rows = read_action_metrics(args.action_metrics.expanduser().resolve())
    action_summary = {
        "by_episode": group_action_improvements(action_rows, "episode_id"),
        "worse_frame_count": int(
            sum(
                float(row["hidden_shift_chunk_mean_l2"]) > float(row["raw_chunk_mean_l2"])
                for row in action_rows
            )
        ),
        "total_frames": len(action_rows),
    }

    summary = {
        "method": "hidden_shift_no_collapse_check",
        "analysis_root": str(analysis_root),
        "feature": FEATURE,
        "window": args.window,
        "frame_count": len(frames),
        "episodes": episodes,
        "paired_gap": paired,
        "variance": variance,
        "pairwise_structure": pairwise,
        "action_gap": action_summary,
        "interpretation_rules": [
            "No collapse if corrected feature variance remains nonzero and comparable to Real/Sim.",
            "Pairwise structure correlation checks whether frame-to-frame geometry of the representation is preserved.",
            "This remains offline evidence only.",
        ],
    }
    write_json(output_dir / "hidden_shift_no_collapse_summary.json", summary)
    print(json.dumps(summary, ensure_ascii=False, indent=2)[:6000])
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
