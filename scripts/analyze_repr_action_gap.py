#!/usr/bin/env python3
from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path
from typing import Any

import numpy as np


ROOT = Path("/home/ubuntu/a0509_vla_linux_field_bundle_20260903")
DEFAULT_ANALYSIS_ROOT = ROOT / "outputs" / "token_distribution_analysis" / "5_episodes"
DEFAULT_OUTPUT_DIR = ROOT / "lhj" / "phase1_action_gap"


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--analysis-root", type=Path, default=DEFAULT_ANALYSIS_ROOT)
    parser.add_argument("--output-dir", type=Path, default=DEFAULT_OUTPUT_DIR)
    return parser.parse_args()


def read_json(path: Path) -> Any:
    return json.loads(path.read_text(encoding="utf-8"))


def write_json(path: Path, payload: Any) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")


def read_csv(path: Path) -> list[dict[str, str]]:
    with path.open("r", encoding="utf-8", newline="") as handle:
        return list(csv.DictReader(handle))


def frame_from_record(record: dict[str, Any]) -> str:
    return Path(record["source_image"]).stem


def records_by_frame(manifest_path: Path) -> dict[str, dict[str, Any]]:
    manifest = read_json(manifest_path)
    return {frame_from_record(record): record for record in manifest["records"]}


def episode_from_frame(frame: str) -> str:
    return "_".join(frame.split("_")[:2])


def progress_from_frame(frame: str) -> int:
    return int(frame.split("_")[-1])


def action_chunk(record: dict[str, Any]) -> np.ndarray:
    response = record.get("response", {})
    actions = response.get("actions")
    if actions is None:
        actions = [response.get("action")]
    arr = np.asarray(actions, dtype=np.float64)
    if arr.ndim == 1:
        arr = arr.reshape(1, -1)
    if arr.ndim != 2 or arr.shape[1] != 7:
        raise ValueError(f"Invalid action shape for {record.get('source_image')}: {arr.shape}")
    return arr


def l2(values: np.ndarray) -> float:
    return float(np.linalg.norm(values))


def cosine_similarity(left: np.ndarray, right: np.ndarray) -> float:
    denom = float(np.linalg.norm(left) * np.linalg.norm(right))
    if denom <= 1e-12:
        return 0.0
    return float(np.dot(left, right) / denom)


def ranks(values: np.ndarray) -> np.ndarray:
    order = np.argsort(values, kind="mergesort")
    ranks_out = np.empty_like(order, dtype=np.float64)
    i = 0
    while i < len(values):
        j = i + 1
        while j < len(values) and values[order[j]] == values[order[i]]:
            j += 1
        rank = (i + j - 1) / 2.0
        ranks_out[order[i:j]] = rank
        i = j
    return ranks_out


def pearson(left: list[float], right: list[float]) -> float | None:
    x = np.asarray(left, dtype=np.float64)
    y = np.asarray(right, dtype=np.float64)
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


def spearman(left: list[float], right: list[float]) -> float | None:
    x = np.asarray(left, dtype=np.float64)
    y = np.asarray(right, dtype=np.float64)
    mask = np.isfinite(x) & np.isfinite(y)
    x = x[mask]
    y = y[mask]
    if x.size < 3:
        return None
    return pearson(ranks(x).tolist(), ranks(y).tolist())


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


def group_summary(rows: list[dict[str, Any]], group_key: str, metric_keys: list[str]) -> list[dict[str, Any]]:
    groups: dict[str, list[dict[str, Any]]] = {}
    for row in rows:
        groups.setdefault(str(row[group_key]), []).append(row)
    output = []
    for group, items in sorted(groups.items()):
        result: dict[str, Any] = {group_key: group, "count": len(items)}
        for key in metric_keys:
            vals = [float(item[key]) for item in items if item.get(key) is not None]
            if vals:
                result[f"{key}_mean"] = float(np.mean(vals))
                result[f"{key}_max"] = float(np.max(vals))
        output.append(result)
    return output


def main() -> int:
    args = parse_args()
    analysis_root = args.analysis_root.expanduser().resolve()
    output_dir = args.output_dir.expanduser().resolve()
    output_dir.mkdir(parents=True, exist_ok=True)

    real_records = records_by_frame(analysis_root / "features" / "real" / "feature_manifest.json")
    sim_records = records_by_frame(analysis_root / "features" / "sim" / "feature_manifest.json")
    metric_rows = read_csv(analysis_root / "metrics" / "frame_pair_metrics.csv")
    phase_map = {
        pair["pair_id"]: pair.get("planner_phase", "")
        for pair in read_json(analysis_root / "paired_inputs" / "paired_manifest.json")["pairs"]
    }
    metric_by_frame = {row["frame"]: row for row in metric_rows}
    frames = sorted(set(real_records) & set(sim_records) & set(metric_by_frame))
    rows: list[dict[str, Any]] = []

    for frame in frames:
        real_actions = action_chunk(real_records[frame])
        sim_actions = action_chunk(sim_records[frame])
        chunk_count = min(len(real_actions), len(sim_actions))
        real_actions = real_actions[:chunk_count]
        sim_actions = sim_actions[:chunk_count]
        delta = real_actions - sim_actions
        first_delta = delta[0]
        row: dict[str, Any] = {
            "frame": frame,
            "episode_id": episode_from_frame(frame),
            "progress": progress_from_frame(frame),
            "planner_phase": phase_map.get(frame, ""),
            "action_chunk_count": chunk_count,
            "action_first_l2": l2(first_delta),
            "action_first_translation_l2": l2(first_delta[:3]),
            "action_first_rotation_l2": l2(first_delta[3:6]),
            "action_first_gripper_abs": float(abs(first_delta[6])),
            "action_chunk_mean_l2": float(np.mean(np.linalg.norm(delta, axis=1))),
            "action_chunk_max_l2": float(np.max(np.linalg.norm(delta, axis=1))),
            "real_first_translation_norm": l2(real_actions[0, :3]),
            "sim_first_translation_norm": l2(sim_actions[0, :3]),
            "translation_direction_cosine": cosine_similarity(real_actions[0, :3], sim_actions[0, :3]),
            "real_first_gripper": float(real_actions[0, 6]),
            "sim_first_gripper": float(sim_actions[0, 6]),
        }
        for key, value in metric_by_frame[frame].items():
            if key == "frame":
                continue
            row[key] = float(value)
        rows.append(row)

    fieldnames = list(rows[0].keys())
    with (output_dir / "frame_repr_action_gap.csv").open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)

    repr_keys = [
        "vision_backbone_output_pooled_cosine_distance",
        "vision_backbone_output_pooled_l2",
        "vision_backbone_output_token_mean_l2_per_token",
        "projector_output_pooled_cosine_distance",
        "projector_output_pooled_l2",
        "projector_output_token_mean_l2_per_token",
    ]
    action_keys = [
        "action_first_l2",
        "action_first_translation_l2",
        "action_first_rotation_l2",
        "action_first_gripper_abs",
        "action_chunk_mean_l2",
        "action_chunk_max_l2",
    ]
    correlations: list[dict[str, Any]] = []
    for repr_key in repr_keys:
        for action_key in action_keys:
            correlations.append(
                {
                    "repr_metric": repr_key,
                    "action_metric": action_key,
                    "pearson": pearson([row[repr_key] for row in rows], [row[action_key] for row in rows]),
                    "spearman": spearman([row[repr_key] for row in rows], [row[action_key] for row in rows]),
                }
            )
    with (output_dir / "repr_action_correlations.csv").open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(correlations[0].keys()))
        writer.writeheader()
        writer.writerows(correlations)

    summary = {
        "analysis_root": str(analysis_root),
        "output_dir": str(output_dir),
        "paired_frame_count": len(rows),
        "action_gap_summary": {key: summarize([row[key] for row in rows]) for key in action_keys},
        "episode_summary": group_summary(rows, "episode_id", action_keys + repr_keys),
        "phase_summary": group_summary(rows, "planner_phase", action_keys + repr_keys),
        "top_spearman_correlations": sorted(
            correlations,
            key=lambda item: abs(item["spearman"]) if item["spearman"] is not None else -1,
            reverse=True,
        )[:12],
        "notes": [
            "Actions are existing offline policy responses stored in feature manifests.",
            "This evaluates Real-observation policy response vs Sim-observation policy response.",
            "It does not evaluate corrected representations because correction has not yet been injected into the actual policy forward path.",
            "Correlation is not causality.",
        ],
    }
    write_json(output_dir / "repr_action_gap_summary.json", summary)
    print(json.dumps(summary["top_spearman_correlations"][:8], indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
