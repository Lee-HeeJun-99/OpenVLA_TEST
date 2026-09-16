#!/usr/bin/env python3
from __future__ import annotations

import argparse
import csv
import json
import os
import re
import sys
from pathlib import Path
from typing import Any

import numpy as np
import torch


ROOT = Path("/home/ubuntu/a0509_vla_linux_field_bundle_20260903")
DEFAULT_ANALYSIS_ROOT = ROOT / "outputs" / "token_distribution_analysis" / "5_episodes"
DEFAULT_PHASE_CSV = DEFAULT_ANALYSIS_ROOT / "episode_phase_summary" / "frame_metrics_enriched.csv"
DEFAULT_CHECKPOINT = ROOT / "runtime_state" / "oft_mixed480_step28560_merged"
DEFAULT_OUTPUT_DIR = ROOT / "lhj" / "phase4_policy_relevance" / "action_hidden_direction_perturbation"
OFT_REPO = ROOT / "runtime" / "openvla-oft"
DATASET_KEY = "a0509_sim_cube_pick"
ACTION_DIM = 7
CHUNK = 5
FEATURE = "action_hidden_states.input"


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--analysis-root", type=Path, default=DEFAULT_ANALYSIS_ROOT)
    parser.add_argument("--phase-csv", type=Path, default=DEFAULT_PHASE_CSV)
    parser.add_argument("--checkpoint", type=Path, default=DEFAULT_CHECKPOINT)
    parser.add_argument("--output-dir", type=Path, default=DEFAULT_OUTPUT_DIR)
    parser.add_argument("--alphas", default="-0.5,0,0.25,0.5,0.75,1.0,1.5")
    parser.add_argument("--seed", type=int, default=20260916)
    parser.add_argument("--batch-size", type=int, default=32)
    return parser.parse_args()


def read_json(path: Path) -> Any:
    return json.loads(path.read_text(encoding="utf-8"))


def write_json(path: Path, payload: Any) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")


def write_csv(path: Path, rows: list[dict[str, Any]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    if not rows:
        path.write_text("", encoding="utf-8")
        return
    with path.open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(rows[0].keys()))
        writer.writeheader()
        writer.writerows(rows)


def read_csv(path: Path) -> list[dict[str, str]]:
    with path.open("r", encoding="utf-8", newline="") as handle:
        return list(csv.DictReader(handle))


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


def action_chunk(record: dict[str, Any]) -> np.ndarray:
    response = record.get("response", {})
    actions = response.get("actions")
    if actions is None:
        actions = [response.get("action")]
    arr = np.asarray(actions, dtype=np.float64)
    if arr.shape != (CHUNK, ACTION_DIM):
        raise ValueError(f"Unexpected action shape for {record.get('source_image')}: {arr.shape}")
    return arr


def load_npz_array(record: dict[str, Any], key: str) -> np.ndarray:
    with np.load(record["_feature_path"], allow_pickle=True) as data:
        return np.asarray(data[key], dtype=np.float32)


def unnormalize_actions(normalized_actions: np.ndarray, stats: dict[str, Any]) -> np.ndarray:
    norm = np.asarray(normalized_actions, dtype=np.float64)
    mask = np.asarray(stats.get("mask", np.ones_like(stats["q01"], dtype=bool)), dtype=bool)
    high = np.asarray(stats["q99"], dtype=np.float64)
    low = np.asarray(stats["q01"], dtype=np.float64)
    return np.where(mask, 0.5 * (norm + 1.0) * (high - low + 1e-8) + low, norm)


def chunk_metrics(left: np.ndarray, right: np.ndarray, prefix: str) -> dict[str, float]:
    delta = np.asarray(left, dtype=np.float64) - np.asarray(right, dtype=np.float64)
    first = delta[0]
    chunk_l2 = np.linalg.norm(delta, axis=1)
    return {
        f"{prefix}_first_l2": float(np.linalg.norm(first)),
        f"{prefix}_first_translation_l2": float(np.linalg.norm(first[:3])),
        f"{prefix}_first_rotation_l2": float(np.linalg.norm(first[3:6])),
        f"{prefix}_first_gripper_abs": float(abs(first[6])),
        f"{prefix}_chunk_mean_l2": float(chunk_l2.mean()),
        f"{prefix}_chunk_max_l2": float(chunk_l2.max()),
    }


def summarize(values: list[float] | np.ndarray) -> dict[str, float | int]:
    arr = np.asarray(values, dtype=np.float64).reshape(-1)
    return {
        "count": int(arr.size),
        "mean": float(arr.mean()),
        "std": float(arr.std()),
        "min": float(arr.min()),
        "p25": float(np.percentile(arr, 25)),
        "p50": float(np.percentile(arr, 50)),
        "p75": float(np.percentile(arr, 75)),
        "p90": float(np.percentile(arr, 90)),
        "max": float(arr.max()),
    }


def spearman(x: list[float], y: list[float]) -> float | None:
    left = np.asarray(x, dtype=np.float64)
    right = np.asarray(y, dtype=np.float64)
    mask = np.isfinite(left) & np.isfinite(right)
    left = left[mask]
    right = right[mask]
    if left.size < 3:
        return None
    return pearson(rankdata(left), rankdata(right))


def pearson(left: np.ndarray, right: np.ndarray) -> float | None:
    x = np.asarray(left, dtype=np.float64)
    y = np.asarray(right, dtype=np.float64)
    if x.size < 3:
        return None
    x = x - x.mean()
    y = y - y.mean()
    denom = float(np.linalg.norm(x) * np.linalg.norm(y))
    if denom <= 1e-12:
        return None
    return float(np.dot(x, y) / denom)


def rankdata(values: np.ndarray) -> np.ndarray:
    order = np.argsort(values, kind="mergesort")
    ranks = np.empty_like(order, dtype=np.float64)
    i = 0
    while i < len(values):
        j = i + 1
        while j < len(values) and values[order[j]] == values[order[i]]:
            j += 1
        ranks[order[i:j]] = (i + j - 1) / 2.0
        i = j
    return ranks


def phase_map(path: Path) -> dict[str, str]:
    rows = read_csv(path)
    return {row["pair_id"]: row["planner_phase"] for row in rows}


def load_action_head(checkpoint: Path):
    os.environ.setdefault("A0509_ACTION_CHUNK_SIZE", str(CHUNK))
    if str(OFT_REPO) not in sys.path:
        sys.path.insert(0, str(OFT_REPO))
    from prismatic.models.action_heads import L1RegressionActionHead

    training_config = read_json(checkpoint / "a0509_training_config.json")
    action_head = L1RegressionActionHead(
        input_dim=4096,
        hidden_dim=4096,
        action_dim=ACTION_DIM,
        bounded_gripper=bool(training_config.get("bounded_gripper", False)),
        gripper_loss_weight=float(training_config.get("gripper_loss_weight", 3.0)),
    )
    matches = sorted(checkpoint.glob("action_head*checkpoint*.pt"))
    if len(matches) != 1:
        raise FileNotFoundError(f"Expected one action_head checkpoint in {checkpoint}, found {len(matches)}")
    state_dict = torch.load(matches[0], map_location="cpu", weights_only=True)
    state_dict = {
        (key[7:] if key.startswith("module.") else key): value
        for key, value in state_dict.items()
    }
    action_head.load_state_dict(state_dict)
    device = torch.device("cuda:0" if torch.cuda.is_available() else "cpu")
    dtype = torch.bfloat16 if device.type == "cuda" else torch.float32
    return action_head.to(device=device, dtype=dtype).eval(), device, dtype, str(matches[0])


def predict_batches(
    action_head: torch.nn.Module,
    device: torch.device,
    dtype: torch.dtype,
    hidden_batch: list[np.ndarray],
    batch_size: int,
) -> list[np.ndarray]:
    outputs: list[np.ndarray] = []
    with torch.inference_mode():
        for start in range(0, len(hidden_batch), batch_size):
            batch_np = np.concatenate(hidden_batch[start : start + batch_size], axis=0)
            tensor = torch.as_tensor(batch_np, device=device, dtype=dtype)
            pred = action_head.predict_action(tensor)
            pred = pred.reshape(-1, CHUNK, ACTION_DIM).float().cpu().numpy()
            outputs.extend([pred[i].astype(np.float64) for i in range(pred.shape[0])])
    return outputs


def make_control_directions(delta: np.ndarray, rng: np.random.Generator) -> dict[str, np.ndarray]:
    delta64 = delta.astype(np.float64, copy=False)
    norm = float(np.linalg.norm(delta64.reshape(-1)))
    if norm <= 1e-12:
        zero = np.zeros_like(delta64)
        return {"real_to_sim": delta64, "random": zero, "orthogonal": zero}

    random = rng.standard_normal(delta64.shape).astype(np.float64)
    random *= norm / max(float(np.linalg.norm(random.reshape(-1))), 1e-12)

    orth = rng.standard_normal(delta64.shape).astype(np.float64)
    flat_delta = delta64.reshape(-1)
    flat_orth = orth.reshape(-1)
    projection = float(np.dot(flat_orth, flat_delta) / max(np.dot(flat_delta, flat_delta), 1e-12))
    orth = orth - projection * delta64
    orth *= norm / max(float(np.linalg.norm(orth.reshape(-1))), 1e-12)
    return {"real_to_sim": delta64, "random": random, "orthogonal": orth}


def aggregate(rows: list[dict[str, Any]], group_keys: list[str]) -> list[dict[str, Any]]:
    grouped: dict[tuple[Any, ...], list[dict[str, Any]]] = {}
    for row in rows:
        key = tuple(row[k] for k in group_keys)
        grouped.setdefault(key, []).append(row)
    metrics = [
        "gap_to_sim_chunk_mean_l2",
        "gap_to_sim_chunk_max_l2",
        "gap_to_sim_first_translation_l2",
        "gap_to_sim_first_rotation_l2",
        "gap_to_sim_first_gripper_abs",
        "change_from_real_chunk_mean_l2",
        "change_from_real_first_translation_l2",
        "change_from_real_first_rotation_l2",
        "change_from_real_first_gripper_abs",
        "gap_reduction_chunk_mean_l2",
        "gap_reduction_ratio_chunk_mean_l2",
    ]
    out = []
    for key, items in sorted(grouped.items(), key=lambda kv: kv[0]):
        record = {name: value for name, value in zip(group_keys, key)}
        record["count"] = len(items)
        for metric in metrics:
            vals = [float(item[metric]) for item in items if np.isfinite(float(item[metric]))]
            if vals:
                stats = summarize(vals)
                record[f"{metric}_mean"] = stats["mean"]
                record[f"{metric}_p50"] = stats["p50"]
                record[f"{metric}_p90"] = stats["p90"]
        record["improved_count"] = int(sum(float(item["gap_reduction_chunk_mean_l2"]) > 0 for item in items))
        record["worsened_count"] = int(sum(float(item["gap_reduction_chunk_mean_l2"]) < 0 for item in items))
        out.append(record)
    return out


def main() -> int:
    args = parse_args()
    analysis_root = args.analysis_root.expanduser().resolve()
    output_dir = args.output_dir.expanduser().resolve()
    checkpoint = args.checkpoint.expanduser().resolve()
    output_dir.mkdir(parents=True, exist_ok=True)
    alphas = [float(value) for value in args.alphas.split(",")]
    rng = np.random.default_rng(args.seed)

    real_records = records_by_frame(analysis_root / "features" / "real" / "feature_manifest.json")
    sim_records = records_by_frame(analysis_root / "features" / "sim" / "feature_manifest.json")
    frames = sorted(set(real_records) & set(sim_records))
    phases = phase_map(args.phase_csv.expanduser().resolve())
    action_stats = read_json(checkpoint / "dataset_statistics.json")[DATASET_KEY]["action"]
    action_head, device, dtype, action_head_checkpoint = load_action_head(checkpoint)

    base_rows = []
    hidden_inputs = []
    hidden_meta = []

    for frame in frames:
        real_hidden = load_npz_array(real_records[frame], FEATURE)
        sim_hidden = load_npz_array(sim_records[frame], FEATURE)
        real_action = action_chunk(real_records[frame])
        sim_action = action_chunk(sim_records[frame])
        raw_gap = chunk_metrics(real_action, sim_action, "raw_gap")
        delta = sim_hidden - real_hidden
        directions = make_control_directions(delta, rng)
        direction_norm = float(np.linalg.norm(delta.reshape(-1)))
        for direction_name, direction in directions.items():
            for alpha in alphas:
                hidden_inputs.append((real_hidden.astype(np.float64) + alpha * direction).astype(np.float32))
                hidden_meta.append(
                    {
                        "frame": frame,
                        "episode_id": episode_id_from_frame(frame),
                        "progress": progress_from_frame(frame),
                        "planner_phase": phases.get(frame, "UNKNOWN"),
                        "direction": direction_name,
                        "alpha": alpha,
                        "direction_norm": direction_norm,
                        **raw_gap,
                    }
                )
        base_rows.append(
            {
                "frame": frame,
                "episode_id": episode_id_from_frame(frame),
                "progress": progress_from_frame(frame),
                "planner_phase": phases.get(frame, "UNKNOWN"),
                "delta_hidden_norm": direction_norm,
                **raw_gap,
            }
        )

    norm_predictions = predict_batches(action_head, device, dtype, hidden_inputs, args.batch_size)
    rows = []
    for meta, norm_action in zip(hidden_meta, norm_predictions):
        frame = meta["frame"]
        real_action = action_chunk(real_records[frame])
        sim_action = action_chunk(sim_records[frame])
        pred_action = unnormalize_actions(norm_action, action_stats)
        gap_to_sim = chunk_metrics(pred_action, sim_action, "gap_to_sim")
        change_from_real = chunk_metrics(pred_action, real_action, "change_from_real")
        row = {**meta, **gap_to_sim, **change_from_real}
        raw_chunk = float(meta["raw_gap_chunk_mean_l2"])
        gap_chunk = float(gap_to_sim["gap_to_sim_chunk_mean_l2"])
        row["gap_reduction_chunk_mean_l2"] = raw_chunk - gap_chunk
        row["gap_reduction_ratio_chunk_mean_l2"] = (raw_chunk - gap_chunk) / raw_chunk if abs(raw_chunk) > 1e-12 else 0.0
        row["gap_to_sim_minus_raw_chunk_mean_l2"] = gap_chunk - raw_chunk
        rows.append(row)

    write_csv(output_dir / "direction_perturbation_frame_metrics.csv", rows)
    write_csv(output_dir / "direction_perturbation_base_frame_metrics.csv", base_rows)

    overall = aggregate(rows, ["direction", "alpha"])
    by_episode = aggregate(rows, ["episode_id", "direction", "alpha"])
    by_phase = aggregate(rows, ["planner_phase", "direction", "alpha"])
    write_csv(output_dir / "direction_perturbation_summary_overall.csv", overall)
    write_csv(output_dir / "direction_perturbation_summary_by_episode.csv", by_episode)
    write_csv(output_dir / "direction_perturbation_summary_by_phase.csv", by_phase)

    # Compact JSON summary with key alpha=1 comparisons and monotonicity diagnostics.
    alpha1 = [row for row in overall if float(row["alpha"]) == 1.0]
    direction_effect = {
        row["direction"]: {
            "gap_to_sim_chunk_mean_l2": row.get("gap_to_sim_chunk_mean_l2_mean"),
            "gap_reduction_ratio_chunk_mean_l2": row.get("gap_reduction_ratio_chunk_mean_l2_mean"),
            "change_from_real_chunk_mean_l2": row.get("change_from_real_chunk_mean_l2_mean"),
            "improved_count": row.get("improved_count"),
            "worsened_count": row.get("worsened_count"),
        }
        for row in alpha1
    }
    real_to_sim_rows = [row for row in rows if row["direction"] == "real_to_sim"]
    correlations = {
        "alpha_vs_gap_to_sim_chunk_mean_l2_spearman": spearman(
            [float(row["alpha"]) for row in real_to_sim_rows],
            [float(row["gap_to_sim_chunk_mean_l2"]) for row in real_to_sim_rows],
        ),
        "alpha_vs_change_from_real_chunk_mean_l2_spearman": spearman(
            [float(row["alpha"]) for row in real_to_sim_rows],
            [float(row["change_from_real_chunk_mean_l2"]) for row in real_to_sim_rows],
        ),
    }
    summary = {
        "experiment": "action_hidden_states.input Real-to-Sim direction perturbation",
        "policy_scope": "oftplus_h5_vision offline 225-pair dataset only",
        "analysis_root": str(analysis_root),
        "checkpoint": str(checkpoint),
        "action_head_checkpoint": action_head_checkpoint,
        "feature": FEATURE,
        "frames": len(frames),
        "alphas": alphas,
        "directions": ["real_to_sim", "random", "orthogonal"],
        "seed": args.seed,
        "device": str(device),
        "dtype": str(dtype),
        "phase_csv": str(args.phase_csv),
        "alpha_1_direction_effect": direction_effect,
        "real_to_sim_correlations": correlations,
        "outputs": {
            "frame_metrics": str(output_dir / "direction_perturbation_frame_metrics.csv"),
            "overall_summary": str(output_dir / "direction_perturbation_summary_overall.csv"),
            "by_episode": str(output_dir / "direction_perturbation_summary_by_episode.csv"),
            "by_phase": str(output_dir / "direction_perturbation_summary_by_phase.csv"),
        },
        "interpretation_limits": [
            "This is offline Level-2 policy/action evidence only.",
            "It uses oftplus_h5_vision and must not be mixed with current ROS proprio deployment.",
            "Random and orthogonal controls are sampled controls, not exhaustive policy-null decompositions.",
        ],
    }
    write_json(output_dir / "direction_perturbation_summary.json", summary)
    print(json.dumps(summary, ensure_ascii=False, indent=2)[:8000])
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
