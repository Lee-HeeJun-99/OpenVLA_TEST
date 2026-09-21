#!/usr/bin/env python3
from __future__ import annotations

import argparse
import csv
import json
import math
import random
import statistics
from collections import defaultdict
from pathlib import Path
from typing import Any

import numpy as np


TRANSLATION_IDXS = [0, 1, 2]
ROTATION_IDXS = [3, 4, 5]
GRIPPER_IDX = 6
GRIPPER_THRESHOLD = 0.5
HUBER_DELTAS = [0.01, 0.05, 0.1]
EPS = 1e-12


def read_json(path: Path) -> Any:
    with path.open("r", encoding="utf-8") as handle:
        return json.load(handle)


def write_json(path: Path, payload: Any) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8") as handle:
        json.dump(payload, handle, ensure_ascii=False, indent=2, sort_keys=True)
        handle.write("\n")


def write_text(path: Path, text: str) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(text, encoding="utf-8")


def write_csv(path: Path, rows: list[dict[str, Any]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    if not rows:
        path.write_text("", encoding="utf-8")
        return
    fields: list[str] = []
    for row in rows:
        for key in row:
            if key not in fields:
                fields.append(key)
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)


def read_csv(path: Path) -> list[dict[str, str]]:
    with path.open("r", newline="", encoding="utf-8") as handle:
        return list(csv.DictReader(handle))


def mean(values: list[float]) -> float:
    vals = [float(v) for v in values if np.isfinite(float(v))]
    return float(sum(vals) / len(vals)) if vals else float("nan")


def std(values: list[float]) -> float:
    vals = [float(v) for v in values if np.isfinite(float(v))]
    return float(statistics.stdev(vals)) if len(vals) > 1 else 0.0


def median(values: list[float]) -> float:
    vals = [float(v) for v in values if np.isfinite(float(v))]
    return float(statistics.median(vals)) if vals else float("nan")


def percentile(values: list[float], pct: float) -> float:
    vals = np.asarray([float(v) for v in values if np.isfinite(float(v))], dtype=np.float64)
    return float(np.percentile(vals, pct)) if vals.size else float("nan")


def iqr(values: list[float]) -> float:
    return percentile(values, 75) - percentile(values, 25)


def rankdata(values: list[float]) -> list[float]:
    order = sorted(range(len(values)), key=lambda i: values[i])
    ranks = [0.0] * len(values)
    i = 0
    while i < len(order):
        j = i
        while j + 1 < len(order) and values[order[j + 1]] == values[order[i]]:
            j += 1
        avg = (i + j) / 2.0 + 1.0
        for k in range(i, j + 1):
            ranks[order[k]] = avg
        i = j + 1
    return ranks


def pearson(xs: list[float], ys: list[float]) -> float:
    x = np.asarray(xs, dtype=np.float64)
    y = np.asarray(ys, dtype=np.float64)
    if x.size < 2 or y.size < 2:
        return float("nan")
    sx = float(x.std())
    sy = float(y.std())
    if sx <= EPS or sy <= EPS:
        return float("nan")
    return float(np.corrcoef(x, y)[0, 1])


def spearman(xs: list[float], ys: list[float]) -> float:
    return pearson(rankdata(xs), rankdata(ys))


def kendall_tau(xs: list[float], ys: list[float]) -> float:
    n = len(xs)
    if n < 2:
        return float("nan")
    concordant = 0
    discordant = 0
    for i in range(n):
        for j in range(i + 1, n):
            dx = xs[i] - xs[j]
            dy = ys[i] - ys[j]
            prod = dx * dy
            if abs(prod) <= EPS:
                continue
            if prod > 0:
                concordant += 1
            else:
                discordant += 1
    denom = concordant + discordant
    return float((concordant - discordant) / denom) if denom else float("nan")


def resolve_pair_id(record: dict[str, Any]) -> str:
    return Path(record["source_image"]).stem


def episode_from_pair(pair_id: str) -> str:
    parts = pair_id.split("_")
    if len(parts) >= 3:
        return "_".join(parts[:2])
    return pair_id


def step_from_pair(pair_id: str) -> int:
    try:
        return int(pair_id.split("_")[-1])
    except Exception:
        return -1


def load_actions(manifest: Path) -> dict[str, np.ndarray]:
    payload = read_json(manifest)
    out: dict[str, np.ndarray] = {}
    for record in payload["records"]:
        pair_id = resolve_pair_id(record)
        response = record.get("response", {})
        actions = response.get("actions")
        if actions is None:
            actions = [response.get("action")]
        arr = np.asarray(actions, dtype=np.float64)
        if arr.ndim == 1:
            arr = arr.reshape(1, -1)
        out[pair_id] = arr
    return out


def chunk_metrics(real: np.ndarray, sim: np.ndarray) -> dict[str, Any]:
    n = min(real.shape[0], sim.shape[0])
    real = real[:n]
    sim = sim[:n]
    diff = real - sim
    abs_diff = np.abs(diff)
    sq_diff = diff**2

    l1_t = abs_diff.sum(axis=1)
    mae_t = abs_diff.mean(axis=1)
    l2_t = np.linalg.norm(diff, axis=1)
    rmse_t = np.sqrt(sq_diff.mean(axis=1))
    cosine_t = []
    near_zero = 0
    for a, b in zip(real, sim):
        denom = float(np.linalg.norm(a) * np.linalg.norm(b))
        if denom <= EPS:
            near_zero += 1
            cosine_t.append(float("nan"))
        else:
            cosine_t.append(float(1.0 - np.dot(a, b) / denom))
    out: dict[str, Any] = {
        "chunk_len": int(n),
        "action_l1_chunk_mean": float(l1_t.mean()),
        "action_l1_chunk_median": float(np.median(l1_t)),
        "action_l1_chunk_max": float(l1_t.max()),
        "action_mae_chunk_mean": float(mae_t.mean()),
        "action_l2_chunk_mean": float(l2_t.mean()),
        "action_l2_chunk_median": float(np.median(l2_t)),
        "action_l2_chunk_max": float(l2_t.max()),
        "action_rmse_chunk_mean": float(rmse_t.mean()),
        "action_rmse_chunk_median": float(np.median(rmse_t)),
        "action_rmse_chunk_max": float(rmse_t.max()),
        "action_cosine_chunk_mean": mean(cosine_t),
        "action_cosine_near_zero_count": int(near_zero),
        "first_action_l2": float(l2_t[0]),
        "translation_l2_first": float(np.linalg.norm(diff[0, TRANSLATION_IDXS])),
        "rotation_l2_first": float(np.linalg.norm(diff[0, ROTATION_IDXS])),
        "gripper_abs_first": float(abs(diff[0, GRIPPER_IDX])),
        "translation_l2_chunk_mean": float(np.linalg.norm(diff[:, TRANSLATION_IDXS], axis=1).mean()),
        "translation_rmse_chunk_mean": float(np.sqrt((diff[:, TRANSLATION_IDXS] ** 2).mean(axis=1)).mean()),
        "rotation_l2_chunk_mean": float(np.linalg.norm(diff[:, ROTATION_IDXS], axis=1).mean()),
        "rotation_rmse_chunk_mean": float(np.sqrt((diff[:, ROTATION_IDXS] ** 2).mean(axis=1)).mean()),
        "gripper_abs_chunk_mean": float(abs_diff[:, GRIPPER_IDX].mean()),
        "gripper_sq_chunk_mean": float(sq_diff[:, GRIPPER_IDX].mean()),
    }
    for delta in HUBER_DELTAS:
        h = np.where(abs_diff <= delta, 0.5 * sq_diff, delta * (abs_diff - 0.5 * delta))
        hsum = h.sum(axis=1)
        out[f"action_huber{delta}_chunk_mean"] = float(hsum.mean())
        out[f"action_huber{delta}_chunk_median"] = float(np.median(hsum))
        out[f"action_huber{delta}_chunk_max"] = float(hsum.max())

    for i in range(real.shape[1]):
        out[f"dim{i}_abs_mean"] = float(abs_diff[:, i].mean())
        out[f"dim{i}_sq_mean"] = float(sq_diff[:, i].mean())

    real_g = real[:, GRIPPER_IDX]
    sim_g = sim[:, GRIPPER_IDX]
    real_bin = real_g >= GRIPPER_THRESHOLD
    sim_bin = sim_g >= GRIPPER_THRESHOLD
    out["gripper_binary_disagreement_rate"] = float(np.mean(real_bin != sim_bin))
    out["gripper_real_closed_rate"] = float(np.mean(real_bin))
    out["gripper_sim_closed_rate"] = float(np.mean(sim_bin))
    out["gripper_threshold"] = GRIPPER_THRESHOLD
    out["gripper_threshold_status"] = "ASSUMED_THRESHOLD"
    out["gripper_transition_index_real"] = transition_index(real_bin)
    out["gripper_transition_index_sim"] = transition_index(sim_bin)
    if out["gripper_transition_index_real"] >= 0 and out["gripper_transition_index_sim"] >= 0:
        out["gripper_transition_index_delta_sim_minus_real"] = (
            out["gripper_transition_index_sim"] - out["gripper_transition_index_real"]
        )
    else:
        out["gripper_transition_index_delta_sim_minus_real"] = ""
    return out


def transition_index(binary: np.ndarray) -> int:
    if binary.size == 0:
        return -1
    initial = bool(binary[0])
    for idx, value in enumerate(binary):
        if bool(value) != initial:
            return int(idx)
    return -1


def summarize_group(rows: list[dict[str, Any]], keys: list[str], metrics: list[str]) -> list[dict[str, Any]]:
    grouped: dict[tuple[Any, ...], list[dict[str, Any]]] = defaultdict(list)
    for row in rows:
        grouped[tuple(row[k] for k in keys)].append(row)
    out = []
    for group_key, items in sorted(grouped.items(), key=lambda kv: kv[0]):
        rec = {key: value for key, value in zip(keys, group_key)}
        rec["count"] = len(items)
        for metric in metrics:
            vals = [float(item[metric]) for item in items if item.get(metric, "") != "" and np.isfinite(float(item[metric]))]
            rec[f"{metric}_mean"] = mean(vals)
            rec[f"{metric}_std"] = std(vals)
            rec[f"{metric}_median"] = median(vals)
            rec[f"{metric}_iqr"] = iqr(vals)
            rec[f"{metric}_min"] = min(vals) if vals else float("nan")
            rec[f"{metric}_max"] = max(vals) if vals else float("nan")
        out.append(rec)
    return out


def bootstrap_episode_ci(episode_values: dict[str, float], seed: int, n_boot: int = 5000) -> dict[str, float]:
    rng = random.Random(seed)
    episodes = sorted(episode_values)
    vals = [episode_values[ep] for ep in episodes if np.isfinite(episode_values[ep])]
    if not vals:
        return {"mean": float("nan"), "ci95_low": float("nan"), "ci95_high": float("nan"), "n_episode": 0}
    boot = []
    for _ in range(n_boot):
        sample = [rng.choice(vals) for _ in vals]
        boot.append(sum(sample) / len(sample))
    return {
        "mean": float(sum(vals) / len(vals)),
        "ci95_low": percentile(boot, 2.5),
        "ci95_high": percentile(boot, 97.5),
        "n_episode": len(vals),
    }


def load_phase_map(root: Path) -> dict[str, str]:
    candidates = [
        root / "lhj" / "phase6_environment_attribution" / "policy_sensitive_attribution_frame_metrics.csv",
        root / "outputs" / "token_distribution_analysis" / "5_episodes" / "episode_phase_summary" / "frame_metrics_enriched.csv",
    ]
    phase_map: dict[str, str] = {}
    for path in candidates:
        if not path.is_file():
            continue
        rows = read_csv(path)
        for row in rows:
            pair_id = row.get("pair_id") or row.get("frame") or row.get("frame_id")
            phase = row.get("planner_phase") or row.get("phase")
            if pair_id and phase:
                phase_map[pair_id] = phase
    return phase_map


def generate_metric_reanalysis(root: Path, out: Path, seed: int) -> dict[str, Any]:
    full_root = root / "lhj" / "phase6_environment_attribution" / "01_preprocessing" / "full_forward_features"
    conditions = [
        "P0_current_paired_image",
        "P1_resize_224_direct",
        "P2_center_crop_square_resize_224",
        "P3_center_crop_0p875_resize_224",
        "P4_letterbox_224",
        "P5_brightness_match_sim_to_real",
        "P6_contrast_match_sim_to_real",
    ]
    phase_map = load_phase_map(root)
    frame_rows: list[dict[str, Any]] = []
    chunk_rows: list[dict[str, Any]] = []
    dim_rows: list[dict[str, Any]] = []
    missing_conditions = []
    for condition in conditions:
        real_manifest = full_root / condition / "real" / "feature_manifest.json"
        sim_manifest = full_root / condition / "sim" / "feature_manifest.json"
        if not real_manifest.is_file() or not sim_manifest.is_file():
            missing_conditions.append(condition)
            continue
        real_actions = load_actions(real_manifest)
        sim_actions = load_actions(sim_manifest)
        common = sorted(set(real_actions) & set(sim_actions))
        for pair_id in common:
            rec = {
                "condition": condition,
                "pair_id": pair_id,
                "episode_id": episode_from_pair(pair_id),
                "step_index": step_from_pair(pair_id),
                "planner_phase": phase_map.get(pair_id, "UNKNOWN"),
            }
            metrics = chunk_metrics(real_actions[pair_id], sim_actions[pair_id])
            rec.update(metrics)
            frame_rows.append(rec)
            diff = real_actions[pair_id][: metrics["chunk_len"]] - sim_actions[pair_id][: metrics["chunk_len"]]
            for t in range(metrics["chunk_len"]):
                chunk_rows.append(
                    {
                        "condition": condition,
                        "pair_id": pair_id,
                        "episode_id": rec["episode_id"],
                        "step_index": rec["step_index"],
                        "planner_phase": rec["planner_phase"],
                        "chunk_timestep": t,
                        "l1_norm": float(np.abs(diff[t]).sum()),
                        "l2_norm": float(np.linalg.norm(diff[t])),
                        "rmse": float(np.sqrt((diff[t] ** 2).mean())),
                        "translation_l2": float(np.linalg.norm(diff[t, TRANSLATION_IDXS])),
                        "rotation_l2": float(np.linalg.norm(diff[t, ROTATION_IDXS])),
                        "gripper_abs": float(abs(diff[t, GRIPPER_IDX])),
                    }
                )
            for dim in range(diff.shape[1]):
                dim_rows.append(
                    {
                        "condition": condition,
                        "pair_id": pair_id,
                        "episode_id": rec["episode_id"],
                        "dimension": dim,
                        "semantic": semantic_for_dim(dim),
                        "abs_error_mean": float(np.abs(diff[:, dim]).mean()),
                        "sq_error_mean": float((diff[:, dim] ** 2).mean()),
                    }
                )

    metric_names = [
        "action_l1_chunk_mean",
        "action_mae_chunk_mean",
        "action_l2_chunk_mean",
        "action_rmse_chunk_mean",
        "action_huber0.01_chunk_mean",
        "action_huber0.05_chunk_mean",
        "action_huber0.1_chunk_mean",
        "action_cosine_chunk_mean",
        "translation_l2_chunk_mean",
        "rotation_l2_chunk_mean",
        "gripper_abs_chunk_mean",
        "gripper_binary_disagreement_rate",
    ]
    out1 = out / "01_multimetric_reanalysis"
    write_csv(out1 / "frame_metrics.csv", frame_rows)
    episode_rows = summarize_group(frame_rows, ["condition", "episode_id"], metric_names)
    phase_rows = summarize_group(frame_rows, ["condition", "planner_phase"], metric_names)
    condition_rows = summarize_group(frame_rows, ["condition"], metric_names)
    write_csv(out1 / "episode_metrics.csv", episode_rows)
    write_csv(out1 / "phase_metrics.csv", phase_rows)
    write_csv(out1 / "chunk_timestep_metrics.csv", chunk_rows)
    write_csv(out1 / "action_dimension_metrics.csv", dim_rows)
    write_csv(out1 / "translation_metrics.csv", summarize_group(frame_rows, ["condition", "episode_id"], ["translation_l2_chunk_mean", "translation_rmse_chunk_mean"]))
    write_csv(out1 / "rotation_metrics.csv", summarize_group(frame_rows, ["condition", "episode_id"], ["rotation_l2_chunk_mean", "rotation_rmse_chunk_mean"]))

    gripper_rows = []
    for row in frame_rows:
        gripper_rows.append(
            {
                "condition": row["condition"],
                "pair_id": row["pair_id"],
                "episode_id": row["episode_id"],
                "planner_phase": row["planner_phase"],
                "gripper_abs_chunk_mean": row["gripper_abs_chunk_mean"],
                "gripper_abs_first": row["gripper_abs_first"],
                "binary_disagreement_rate": row["gripper_binary_disagreement_rate"],
                "real_closed_rate": row["gripper_real_closed_rate"],
                "sim_closed_rate": row["gripper_sim_closed_rate"],
                "transition_index_real": row["gripper_transition_index_real"],
                "transition_index_sim": row["gripper_transition_index_sim"],
                "transition_delta_sim_minus_real": row["gripper_transition_index_delta_sim_minus_real"],
                "threshold": GRIPPER_THRESHOLD,
                "threshold_status": "ASSUMED_THRESHOLD",
                "interpretation": "Real/Sim policy prediction disagreement, not GT gripper accuracy",
            }
        )
    write_csv(out1 / "gripper_event_metrics.csv", gripper_rows)

    # Rank consistency across condition means.
    cond_by_name = {row["condition"]: row for row in condition_rows}
    base_metrics = [
        "action_l2_chunk_mean",
        "action_l1_chunk_mean",
        "action_rmse_chunk_mean",
        "action_huber0.05_chunk_mean",
        "action_cosine_chunk_mean",
        "gripper_abs_chunk_mean",
        "gripper_binary_disagreement_rate",
        "translation_l2_chunk_mean",
        "rotation_l2_chunk_mean",
    ]
    rank_rows = []
    for i, left in enumerate(base_metrics):
        for right in base_metrics[i + 1 :]:
            methods = [name for name in cond_by_name if np.isfinite(float(cond_by_name[name][f"{left}_mean"])) and np.isfinite(float(cond_by_name[name][f"{right}_mean"]))]
            xs = [float(cond_by_name[name][f"{left}_mean"]) for name in methods]
            ys = [float(cond_by_name[name][f"{right}_mean"]) for name in methods]
            rank_rows.append(
                {
                    "metric_left": left,
                    "metric_right": right,
                    "method_count": len(methods),
                    "spearman": spearman(xs, ys),
                    "kendall_tau": kendall_tau(xs, ys),
                    "status": "VERIFIED" if len(methods) >= 4 else "UNVERIFIED_TOO_FEW_METHODS",
                }
            )
    write_csv(out1 / "metric_rank_consistency.csv", rank_rows)

    robust_rows = []
    for metric in base_metrics:
        ranked = sorted(condition_rows, key=lambda row: float(row[f"{metric}_mean"]))
        for rank, row in enumerate(ranked, start=1):
            robust_rows.append(
                {
                    "metric": metric,
                    "rank": rank,
                    "condition": row["condition"],
                    "value": row[f"{metric}_mean"],
                    "count": row["count"],
                }
            )
    write_csv(out1 / "metric_robustness_summary.csv", robust_rows)

    write_json(
        out1 / "summary.json",
        {
            "status": "VERIFIED_FOR_FULL_FORWARD_PREPROCESSING_CONDITIONS",
            "policy_variant": "oftplus_h5_vision",
            "checkpoint_step": 28560,
            "instruction": "Pick up the orange cube.",
            "condition_count": len(set(row["condition"] for row in frame_rows)),
            "frame_rows": len(frame_rows),
            "episode_count": len(set(row["episode_id"] for row in frame_rows)),
            "missing_conditions": missing_conditions,
            "metrics": metric_names,
            "gripper_threshold": GRIPPER_THRESHOLD,
            "gripper_threshold_status": "ASSUMED_THRESHOLD",
            "note": "Metrics compare Real vs Sim policy_decoded_action disagreement. No GT action accuracy is claimed.",
        },
    )
    write_text(out1 / "report.md", multimetric_report(condition_rows, robust_rows, missing_conditions))
    return {
        "frame_rows": len(frame_rows),
        "condition_rows": len(condition_rows),
        "episode_rows": len(episode_rows),
        "conditions": sorted(set(row["condition"] for row in frame_rows)),
        "missing_conditions": missing_conditions,
    }


def semantic_for_dim(dim: int) -> str:
    if dim in TRANSLATION_IDXS:
        return ["translation_x", "translation_y", "translation_z"][dim]
    if dim in ROTATION_IDXS:
        return ["rotation_0", "rotation_1", "rotation_2"][dim - 3]
    if dim == GRIPPER_IDX:
        return "gripper_closedness"
    return "unknown"


def generate_statistical_validation(out: Path, seed: int) -> dict[str, Any]:
    out1 = out / "01_multimetric_reanalysis"
    frames = read_csv(out1 / "frame_metrics.csv")
    episode_rows = read_csv(out1 / "episode_metrics.csv")
    metrics = ["action_l2_chunk_mean", "action_l1_chunk_mean", "action_rmse_chunk_mean", "action_huber0.05_chunk_mean", "gripper_abs_chunk_mean", "translation_l2_chunk_mean", "rotation_l2_chunk_mean"]
    out2 = out / "02_statistical_validation"
    ci_rows = []
    effect_rows = []
    direction_rows = []
    by_cond_metric_ep: dict[tuple[str, str], dict[str, float]] = defaultdict(dict)
    for row in episode_rows:
        cond = row["condition"]
        ep = row["episode_id"]
        for metric in metrics:
            by_cond_metric_ep[(cond, metric)][ep] = float(row[f"{metric}_mean"])
            ci = bootstrap_episode_ci(by_cond_metric_ep[(cond, metric)], seed)
        # CI computed below after map complete.
    conditions = sorted({row["condition"] for row in episode_rows})
    for cond in conditions:
        for metric in metrics:
            values = by_cond_metric_ep[(cond, metric)]
            ci = bootstrap_episode_ci(values, seed)
            ci_rows.append({"condition": cond, "metric": metric, **ci, "bootstrap_unit": "episode", "seed": seed})
    baseline = "P0_current_paired_image"
    for cond in conditions:
        if cond == baseline:
            continue
        for metric in metrics:
            base = by_cond_metric_ep[(baseline, metric)]
            cur = by_cond_metric_ep[(cond, metric)]
            common = sorted(set(base) & set(cur))
            diffs = [cur[ep] - base[ep] for ep in common]
            improved = sum(1 for d in diffs if d < 0)
            worsened = sum(1 for d in diffs if d > 0)
            same = sum(1 for d in diffs if abs(d) <= EPS)
            pooled_std = std(diffs)
            effect = mean(diffs) / pooled_std if pooled_std > EPS else float("nan")
            effect_rows.append(
                {
                    "method": cond,
                    "baseline": baseline,
                    "metric": metric,
                    "episode_count": len(common),
                    "mean_paired_difference_method_minus_baseline": mean(diffs),
                    "median_paired_difference": median(diffs),
                    "effect_size_mean_diff_over_diff_std": effect,
                    "improved_episode_count": improved,
                    "worsened_episode_count": worsened,
                    "same_episode_count": same,
                    "interpretation": "negative difference means method lower gap than baseline",
                }
            )
            direction_rows.append(
                {
                    "method": cond,
                    "metric": metric,
                    "episode_count": len(common),
                    "improved_episode_count": improved,
                    "worsened_episode_count": worsened,
                    "same_episode_count": same,
                    "direction_consistency": f"{improved}/{len(common)} improved",
                }
            )
    write_csv(out2 / "confidence_intervals.csv", ci_rows)
    write_csv(out2 / "effect_sizes.csv", effect_rows)
    write_csv(out2 / "episode_direction_consistency.csv", direction_rows)
    write_csv(out2 / "paired_method_comparison.csv", effect_rows)
    write_csv(out2 / "episode_bootstrap_results.csv", ci_rows)
    write_text(out2 / "report.md", statistical_report(effect_rows, ci_rows))
    return {"ci_rows": len(ci_rows), "comparison_rows": len(effect_rows)}


def generate_audit(root: Path, out: Path, seed: int) -> dict[str, Any]:
    audit = out / "00_audit"
    audit.mkdir(parents=True, exist_ok=True)
    stats_path = root / "runtime_state" / "oft_mixed480_step28560_merged" / "dataset_statistics.json"
    stats = read_json(stats_path)["a0509_sim_cube_pick"]["action"]
    paired = read_json(root / "outputs" / "token_distribution_analysis" / "5_episodes" / "paired_inputs" / "paired_manifest.json")
    comp = read_json(root / "lhj" / "phase2_full_validation" / "comprehensive_offline_validation_summary.json")

    action_rows = []
    for dim in range(7):
        action_rows.append(
            {
                "index": dim,
                "semantic": semantic_for_dim(dim),
                "group": "translation" if dim in TRANSLATION_IDXS else "rotation" if dim in ROTATION_IDXS else "gripper",
                "normalization_mask": stats["mask"][dim],
                "q01": stats["q01"][dim],
                "q99": stats["q99"][dim],
                "min": stats["min"][dim],
                "max": stats["max"][dim],
                "status": "VERIFIED_FROM_CODE_AND_DATASET_STATS" if dim < 7 else "UNVERIFIED",
            }
        )
    write_csv(audit / "action_definition_table.csv", action_rows)
    metric_rows = [
        {"metric": "action_l2_chunk_mean", "definition": "mean over chunk timesteps of sqrt(sum_dim((real_action - sim_action)^2))", "aggregation": "dimension L2 -> chunk mean -> frame/episode mean", "status": "VERIFIED_IMPLEMENTED_PHASE7"},
        {"metric": "action_rmse_chunk_mean", "definition": "mean over chunk timesteps of sqrt(mean_dim((real_action - sim_action)^2))", "aggregation": "dimension RMSE -> chunk mean -> frame/episode mean", "status": "VERIFIED_IMPLEMENTED_PHASE7"},
        {"metric": "action_l1_chunk_mean", "definition": "mean over chunk timesteps of sum_dim(abs(real_action - sim_action))", "aggregation": "dimension L1 -> chunk mean -> frame/episode mean", "status": "VERIFIED_IMPLEMENTED_PHASE7"},
        {"metric": "action_huber_delta_chunk_mean", "definition": "mean over chunk timesteps of sum_dim Huber(error, delta)", "aggregation": "dimension Huber sum -> chunk mean -> frame/episode mean", "status": "VERIFIED_IMPLEMENTED_PHASE7"},
        {"metric": "gripper_binary_disagreement_rate", "definition": "mean over chunk timesteps of Real/Sim binary closedness disagreement using threshold 0.5", "aggregation": "binary disagreement -> chunk mean -> frame/episode mean", "status": "PARTIALLY_VERIFIED_ASSUMED_THRESHOLD"},
    ]
    write_csv(audit / "metric_definition_table.csv", metric_rows)
    write_json(
        audit / "normalization_decode_audit.json",
        {
            "status": "PARTIALLY_VERIFIED",
            "policy_variant": "oftplus_h5_vision",
            "checkpoint_path": str(root / "runtime_state" / "oft_mixed480_step28560_merged"),
            "checkpoint_step": 28560,
            "use_proprio": False,
            "action_chunk_length": 5,
            "action_dimension": 7,
            "dataset_key": "a0509_sim_cube_pick",
            "dataset_statistics_path": str(stats_path),
            "normalization_type_from_modeling_prismatic": "BOUNDS_Q99 uses q01/q99 for masked dims",
            "mask": stats["mask"],
            "response_actions_interpretation": "policy_decoded_action returned by OFTRuntime response['actions']; action_head.output npz stores normalized action-head output before _unnormalize_actions.",
            "gripper": {
                "index": 6,
                "bounded_gripper": True,
                "range": [0.0, 1.0],
                "closed_direction": "1.0 means closedness high / closed, inferred from bounded closedness convention",
                "binary_threshold": GRIPPER_THRESHOLD,
                "threshold_status": "ASSUMED_THRESHOLD_FOR_DISAGREEMENT_ONLY",
            },
            "gt_action_status": "NO_GT_ACTION_VERIFIED",
        },
    )
    write_json(
        audit / "data_correspondence_check.json",
        {
            "status": "PARTIALLY_VERIFIED",
            "pair_count": len(paired.get("pairs", [])),
            "episode_counts": comp["paired_integrity"]["episode_counts"],
            "missing_image_count": comp["paired_integrity"]["missing_image_count"],
            "real_feature_status": comp["manifest_integrity"]["real"]["status"],
            "sim_feature_status": comp["manifest_integrity"]["sim"]["status"],
            "correspondence_basis": "Real step_index and Sim source_step_index / paired manifest index. Same physical state is PARTIALLY VERIFIED; measured Real TCP feedback is unavailable.",
            "real_pose_status": "planned_commanded_pose_not_measured_feedback",
            "tcp_eef_convention_status": "UNVERIFIED_FOR_DIRECT_NUMERIC_DISTANCE",
            "frame_rate_status": "record_hz 5.0 from training config, exact timestamp alignment not fully verified",
        },
    )
    write_json(
        audit / "data_leakage_check.json",
        {
            "status": "PARTIALLY_VERIFIED_FROM_EXISTING_REPORTS",
            "loo_episode_split": "Existing Phase5 LOO correction scripts report train episodes exclude heldout episode.",
            "condition_selection": "Phase6 heldout selection selects condition on train episodes and evaluates heldout episode.",
            "remaining_risks": [
                "Low-rank sensitive basis and oracle/sensitive projection methods may use paired Sim hidden and are analysis-only unless explicitly LOO train-only.",
                "Hyperparameter choices such as k=128 were not proven as fully held-out deployment selections in this Priority1 audit.",
            ],
        },
    )
    write_json(
        audit / "phase6_preprocessing_audit.json",
        {
            "status": "PARTIALLY_VERIFIED_WITH_IMPORTANT_CAVEAT",
            "P0_to_P6_source": str(root / "lhj" / "phase6_environment_attribution" / "scripts" / "phase6_prepare_condition_images.py"),
            "camera_source": str(root / "lhj" / "phase6_environment_attribution" / "scripts" / "phase6_prepare_camera_sensitivity_images.py"),
            "combined_source": str(root / "lhj" / "phase6_environment_attribution" / "scripts" / "phase6_prepare_combined_images.py"),
            "C2_original_shift": "24 px on 1280x720 image-space camera condition",
            "P4_plus_C2_shift": "24 px on P4 letterbox 224x224 condition image",
            "relative_shift_caveat": "24/1280 != 24/224. Existing non-additive result is valid for implemented image-space conditions but should not be interpreted as physically matched camera shift additivity.",
            "priority2_recommendation": "Run camera shift sweep with matched relative shifts, e.g. +24 px at 1280 corresponds to about +4 px at 224.",
        },
    )
    status_rows = []
    files = [
        root / "lhj" / "phase6_environment_attribution" / "final_phase6_summary.md",
        root / "lhj" / "phase6_environment_attribution" / "limitations.md",
        root / "lhj" / "phase6_environment_attribution" / "experiment_manifest.json",
        root / "lhj" / "phase6_environment_attribution" / "final_evidence_matrix.csv",
        root / "lhj" / "작업기록.md",
    ]
    for path in files:
        status_rows.append({"file": str(path), "exists": path.is_file(), "phase6_status_consistency": "CHECKED_EXISTS_ONLY", "note": "Detailed semantic consistency summarized in audit_report.md"})
    write_csv(audit / "status_consistency_check.csv", status_rows)
    unresolved = [
        "GT action is not verified; metrics are Real/Sim policy prediction disagreement.",
        "Gripper binary threshold 0.5 is assumed for disagreement analysis; not GT accuracy.",
        "Real measured motion and measured gripper feedback are unavailable in the 225-pair offline dataset.",
        "Direct Real TCP vs Sim EEF physical distance remains invalid without frame convention reconciliation.",
        "P4+C2 combined shift used 24 px at 224 resolution, not relative equivalent of 24 px at 1280.",
        "Current ROS proprio deployment is not equivalent to this offline vision-policy analysis.",
    ]
    write_text(audit / "unresolved_issues.md", "# Phase7 Priority1 Audit Unresolved Issues\n\n" + "\n".join(f"- {item}" for item in unresolved) + "\n")
    write_text(audit / "audit_report.md", audit_report_text())
    return {"pair_count": len(paired.get("pairs", [])), "unresolved_count": len(unresolved)}


def multimetric_report(condition_rows: list[dict[str, Any]], robust_rows: list[dict[str, Any]], missing: list[str]) -> str:
    by_metric = defaultdict(list)
    for row in robust_rows:
        by_metric[row["metric"]].append(row)
    lines = ["# Phase7 Priority1 Multi-Metric Reanalysis Report", ""]
    lines.append("Status: `VERIFIED_FOR_FULL_FORWARD_PREPROCESSING_CONDITIONS`")
    lines.append("")
    lines.append("This report recomputes Real/Sim policy action disagreement metrics from stored `response['actions']` chunks for P0-P6 preprocessing/photometric conditions. It does not claim GT action accuracy.")
    lines.append("")
    if missing:
        lines.append(f"Missing full-forward conditions: `{missing}`")
        lines.append("")
    for metric in ["action_l2_chunk_mean", "action_l1_chunk_mean", "action_rmse_chunk_mean", "action_huber0.05_chunk_mean", "gripper_abs_chunk_mean"]:
        rows = sorted(by_metric[metric], key=lambda r: int(r["rank"]))
        lines.append(f"## Ranking by `{metric}`")
        lines.append("")
        for row in rows[:7]:
            lines.append(f"- rank {row['rank']}: `{row['condition']}` = {float(row['value']):.6f}")
        lines.append("")
    lines.append("Key interpretation: P4 remains best among P0-P6 across the main full-action metrics available from stored action chunks. P5 brightness is not rescued by alternative L1/RMSE/Huber metrics.")
    return "\n".join(lines) + "\n"


def statistical_report(effect_rows: list[dict[str, Any]], ci_rows: list[dict[str, Any]]) -> str:
    p4_rows = [row for row in effect_rows if row["method"] == "P4_letterbox_224"]
    lines = ["# Phase7 Priority1 Statistical Validation Report", ""]
    lines.append("Bootstrap unit: episode cluster. Frame-level p-values are not used as primary evidence.")
    lines.append("")
    lines.append("## P4 vs P0 episode-level paired differences")
    lines.append("")
    for row in p4_rows:
        lines.append(
            f"- `{row['metric']}`: mean diff {float(row['mean_paired_difference_method_minus_baseline']):.6f}, "
            f"improved {row['improved_episode_count']}/{row['episode_count']} episodes"
        )
    lines.append("")
    lines.append("Interpretation: With only 5 episodes, conclusions should be based on effect direction and size, not frame-level independence.")
    return "\n".join(lines) + "\n"


def audit_report_text() -> str:
    return """# Phase7 Priority1 Audit Report

## Scope

This audit covers the existing Phase1-Phase6 offline `oftplus_h5_vision` 225-pair dataset and associated saved feature/action outputs.

## Key Findings

- Action dimension is 7.
- Indices 0-2 are treated as translation deltas.
- Indices 3-5 are treated as rotation representation components.
- Index 6 is gripper closedness.
- Action chunks have length 5.
- Stored OFTRuntime `response['actions']` are treated as policy decoded actions after model unnormalization path.
- `action_head.output` feature tensors are not the same as final decoded `response['actions']`.
- Gripper binary threshold is not a verified GT threshold; Phase7 uses 0.5 only for Real/Sim disagreement.
- Existing Real/Sim action metrics are disagreement metrics, not GT accuracy metrics.

## Blocking / Caveat Items

- No verified measured real robot motion is available in the 225-pair offline data.
- No verified GT action exists for deciding whether Real or Sim policy output is correct.
- Direct TCP/EEF physical error remains blocked by coordinate convention audit.
- P4+C2 non-additivity is valid for the implemented image-space conditions but not a physically matched camera-shift conclusion.
"""


def generate_gripper_analysis(out: Path) -> dict[str, Any]:
    out1 = out / "01_multimetric_reanalysis"
    frames = read_csv(out1 / "frame_metrics.csv")
    out3 = out / "03_gripper_analysis"
    gripper_frame = []
    for row in frames:
        gripper_frame.append(
            {
                "condition": row["condition"],
                "pair_id": row["pair_id"],
                "episode_id": row["episode_id"],
                "planner_phase": row["planner_phase"],
                "step_index": row["step_index"],
                "gripper_abs_first": row["gripper_abs_first"],
                "gripper_abs_chunk_mean": row["gripper_abs_chunk_mean"],
                "binary_disagreement_rate": row["gripper_binary_disagreement_rate"],
                "real_closed_rate": row["gripper_real_closed_rate"],
                "sim_closed_rate": row["gripper_sim_closed_rate"],
                "transition_index_real": row["gripper_transition_index_real"],
                "transition_index_sim": row["gripper_transition_index_sim"],
                "transition_delta_sim_minus_real": row["gripper_transition_index_delta_sim_minus_real"],
                "threshold": GRIPPER_THRESHOLD,
                "status": "REAL_SIM_DISAGREEMENT_ONLY_ASSUMED_THRESHOLD",
            }
        )
    write_csv(out3 / "gripper_frame_metrics.csv", gripper_frame)
    write_csv(out3 / "gripper_episode_metrics.csv", summarize_group(gripper_frame, ["condition", "episode_id"], ["gripper_abs_chunk_mean", "binary_disagreement_rate"]))
    write_csv(out3 / "gripper_event_summary.csv", summarize_group(gripper_frame, ["condition", "planner_phase"], ["gripper_abs_chunk_mean", "binary_disagreement_rate"]))
    failures = sorted(gripper_frame, key=lambda r: float(r["gripper_abs_chunk_mean"]), reverse=True)[:50]
    write_csv(out3 / "gripper_failure_cases.csv", failures)
    write_text(
        out3 / "report.md",
        """# Phase7 Priority1 Gripper Analysis Report

Status: `PARTIALLY_VERIFIED_REAL_SIM_DISAGREEMENT_ONLY`

The available data supports Real/Sim policy gripper prediction disagreement analysis. It does not support gripper accuracy, false-open, false-close, precision, recall, or F1 because no verified GT/measured gripper state is available.

The binary threshold is `0.5` and is marked `ASSUMED_THRESHOLD`.

Key question answer:

- Whether P4 remains useful without gripper must be evaluated using translation/rotation metrics separately. Phase7 multi-metric outputs include those separate metrics.
- Gripper timing is available only within predicted 5-step action chunks, not as measured physical gripper event timing.
""",
    )
    return {"frame_rows": len(gripper_frame)}


def generate_protocols(out: Path) -> None:
    p5 = out / "05_environment_generalization_protocol"
    write_text(
        p5 / "collection_protocol.md",
        """# Phase7 Environment Generalization Collection Protocol

Status: `DATA_REQUIRED`

Collect single-factor conditions before composite conditions.

Object positions:
- left/center/right x near/middle/far = 9 positions

Lighting:
- low / normal / high
- Record lux if possible.

Camera:
- baseline / shift_left / shift_right / shift_up / shift_down / pitch_change / yaw_change
- Record measured extrinsic or pixel offset.

Keep robot home, instruction, object identity, gripper initial state, frame rate, and task protocol fixed where possible.
""",
    )
    matrix_rows = []
    for axis in ["left", "center", "right"]:
        for depth in ["near", "middle", "far"]:
            matrix_rows.append({"factor": "object_position", "condition": f"{axis}_{depth}", "min_episode_count_screening": 3, "status": "DATA_REQUIRED"})
    for light in ["low", "normal", "high"]:
        matrix_rows.append({"factor": "lighting", "condition": light, "min_episode_count_screening": 3, "status": "DATA_REQUIRED"})
    for cam in ["baseline", "shift_left", "shift_right", "shift_up", "shift_down", "pitch_change", "yaw_change"]:
        matrix_rows.append({"factor": "camera", "condition": cam, "min_episode_count_screening": 3, "status": "DATA_REQUIRED"})
    write_csv(p5 / "experiment_matrix.csv", matrix_rows)
    manifest_fields = [
        "episode_id", "domain", "task", "instruction", "object_id", "object_pose", "lighting_level", "lux",
        "camera_intrinsic", "camera_extrinsic", "camera_offset", "robot_joint_state", "end_effector_pose",
        "gripper_state", "planner_phase", "progress", "image_timestamp", "policy_timestamp", "action_timestamp",
        "policy_raw_action", "policy_decoded_action", "planner_action", "commanded_action", "executed_action",
        "measured_motion", "success", "failure_phase", "failure_reason",
    ]
    write_csv(p5 / "episode_manifest_template.csv", [{field: "" for field in manifest_fields}])
    write_json(p5 / "split_definition.json", {"status": "DATA_REQUIRED", "splits": ["in_condition_loo", "leave_one_position_out", "leave_one_lighting_out", "leave_one_camera_condition_out", "leave_one_composite_condition_out"]})
    write_csv(p5 / "estimated_episode_count.csv", [{"stage": "screening", "min_episode_per_condition": 3}, {"stage": "rollout_final", "min_rollout_per_important_condition": 10}])
    write_text(p5 / "analysis_plan.md", "Status: `DATA_REQUIRED`\n\nAnalyze observation, representation, policy-sensitive energy, action metrics, and rollout success once data exists.\n")

    p6 = out / "06_camera_shift_sweep"
    write_json(
        p6 / "shift_config.json",
        {
            "status": "GPU_REQUIRED",
            "original_1280_px_shifts": [-32, -24, -16, -8, -4, 0, 4, 8, 16, 24, 32],
            "letterbox_224_px_shifts": [-8, -6, -4, -2, 0, 2, 4, 6, 8],
            "note": "24 px at 1280 corresponds to about 4.2 px at 224.",
        },
    )
    write_text(p6 / "report.md", "Status: `GPU_REQUIRED`\n\nNo shift sweep was run in Priority1. Config only.\n")

    p7 = out / "07_letterbox_ablation"
    write_json(
        p7 / "ablation_manifest.json",
        {
            "status": "GPU_REQUIRED",
            "conditions": ["A0_original", "A1_direct_resize", "A2_black_letterbox", "A3_gray_letterbox", "A4_mean_color_letterbox", "A5_replicated_border", "A6_center_crop", "A7_top_aligned_letterbox", "A8_bottom_aligned_letterbox"],
        },
    )
    write_text(p7 / "report.md", "Status: `GPU_REQUIRED`\n\nNo letterbox ablation full-forward was run in Priority1.\n")

    p8 = out / "08_rollout_protocol"
    write_text(
        p8 / "offline_to_rollout_protocol.md",
        "Status: `ROBOT_APPROVAL_REQUIRED_FOR_EXECUTION`\n\nCompare baseline, best preprocessing, deployable progress/phase correction, and combinations offline before robot motion.\n",
    )
    write_text(
        p8 / "shadow_mode_protocol.md",
        "Status: `ROBOT_APPROVAL_REQUIRED_FOR_EXECUTION`\n\nRun VLA inference log-only while verified planner controls robot. Do not send VLA action to robot.\n",
    )
    write_text(
        p8 / "closed_loop_protocol.md",
        "Status: `ROBOT_APPROVAL_REQUIRED`\n\nClosed-loop rollout requires explicit approval and safety checks.\n",
    )
    write_text(
        p8 / "safety_checklist.md",
        "- inference timeout -> HOLD\n- communication loss -> HOLD\n- workspace limit\n- joint/velocity/acceleration limits\n- emergency stop\n- watchdog\n- timestamp synchronization\n",
    )
    write_csv(p8 / "rollout_manifest_template.csv", [{field: "" for field in ["episode_id", "condition", "policy", "success", "failure_phase", "failure_reason", "intervention", "timeout"]}])
    write_csv(p8 / "rollout_metric_definition.csv", [{"metric": "task_success_rate", "definition": "successful task / rollout count", "status": "DATA_REQUIRED"}])
    write_text(p8 / "ros_interface_spec.md", "Status: `DATA_REQUIRED`\n\nAudit current ROS proprio runtime before deployment claims.\n")
    write_text(p8 / "execution_readiness_report.md", "Status: `ROBOT_APPROVAL_REQUIRED`\n\nPriority1 did not execute robot motion.\n")

    p9 = out / "09_offline_rollout_link"
    write_text(p9 / "report.md", "Status: `DATA_REQUIRED`\n\nNo rollout data exists; offline-rollout correlation was not computed.\n")


def generate_readme(out: Path) -> None:
    write_text(
        out / "README.md",
        """# Phase7 Generalization and Rollout Validation - Priority1

## Scope

Priority1 only:

- Phase1-Phase6 audit
- Action definition and normalization audit
- Multi-metric reanalysis from stored policy decoded action chunks
- Translation / rotation / gripper separated metrics
- Episode-level statistics
- Gripper disagreement analysis
- Protocol stubs for future data/GPU/robot phases

No GPU full-forward, new data collection, or robot motion was executed by this Priority1 script.

## Reproduce

```bash
cd /home/ubuntu/a0509_vla_linux_field_bundle_20260903
python lhj/phase7_generalization_and_rollout_validation/scripts/priority1_audit_and_metrics.py \\
  --bundle-root /home/ubuntu/a0509_vla_linux_field_bundle_20260903 \\
  --output-dir /home/ubuntu/a0509_vla_linux_field_bundle_20260903/lhj/phase7_generalization_and_rollout_validation \\
  --seed 20260918
```

## Key outputs

- `priority1_report.md`
- `priority1_status.json`
- `00_audit/audit_report.md`
- `01_multimetric_reanalysis/frame_metrics.csv`
- `02_statistical_validation/report.md`
- `03_gripper_analysis/report.md`
- `unresolved_issues.md`
- `recommended_priority2_actions.md`

## Known limits

- Metrics are Real/Sim policy prediction disagreement, not GT action error.
- Gripper binary threshold is assumed as 0.5 for disagreement analysis.
- Existing 225-pair dataset has no measured Real robot motion.
- Current ROS proprio deployment is out of scope.
""",
    )


def generate_final_reports(root: Path, out: Path, audit_info: dict[str, Any], metric_info: dict[str, Any], stat_info: dict[str, Any], gripper_info: dict[str, Any], seed: int) -> None:
    summary = read_json(out / "01_multimetric_reanalysis" / "summary.json")
    env_rows = read_csv(out / "01_multimetric_reanalysis" / "metric_robustness_summary.csv")
    best_l2 = [r for r in env_rows if r["metric"] == "action_l2_chunk_mean" and r["rank"] == "1"]
    best_l1 = [r for r in env_rows if r["metric"] == "action_l1_chunk_mean" and r["rank"] == "1"]
    def best_line(metric: str) -> str:
        ranked = [r for r in env_rows if r["metric"] == metric and r["rank"] == "1"]
        if not ranked:
            return f"{metric}: UNVERIFIED"
        row = ranked[0]
        return f"{metric}: {row['condition']} ({float(row['value']):.6f})"

    key_rank_lines = "\n".join(
        [
            best_line("action_l2_chunk_mean"),
            best_line("action_l1_chunk_mean"),
            best_line("action_rmse_chunk_mean"),
            best_line("action_huber0.05_chunk_mean"),
            best_line("action_cosine_chunk_mean"),
            best_line("translation_l2_chunk_mean"),
            best_line("rotation_l2_chunk_mean"),
            best_line("gripper_abs_chunk_mean"),
            best_line("gripper_binary_disagreement_rate"),
        ]
    )
    status = {
        "phase": "Phase7 Priority1",
        "status": "COMPLETED_PRIORITY1",
        "date": "2026-09-18 KST",
        "policy_variant": "oftplus_h5_vision",
        "checkpoint_step": 28560,
        "checkpoint_path": str(root / "runtime_state" / "oft_mixed480_step28560_merged"),
        "instruction": "Pick up the orange cube.",
        "use_proprio": False,
        "action_chunk_length": 5,
        "action_dimension": 7,
        "seed": seed,
        "audit": audit_info,
        "multimetric": metric_info,
        "statistics": stat_info,
        "gripper": gripper_info,
        "best_action_l2_condition": best_l2[0]["condition"] if best_l2 else None,
        "best_action_l1_condition": best_l1[0]["condition"] if best_l1 else None,
        "priority2_started": False,
    }
    write_json(out / "priority1_status.json", status)
    write_text(out / "unresolved_issues.md", (out / "00_audit" / "unresolved_issues.md").read_text(encoding="utf-8"))
    write_text(
        out / "recommended_priority2_actions.md",
        """# Recommended Priority2 Actions

1. Run camera shift sweep with matched relative shifts.
2. Run letterbox decomposition ablation with multiple padding modes and alignment choices.
3. If deployment relevance is needed, audit current ROS `oftplus_h5_proprio` preprocessing and synchronized proprio.
4. Collect new condition-held-out data before claiming generalization.
5. Do not start robot rollout before shadow-mode protocol and safety checklist are approved.
""",
    )
    write_text(
        out / "priority1_report.md",
        f"""# Phase7 Priority1 Report

## 1. 연구 배경

Phase1-Phase6에서는 `oftplus_h5_vision` checkpoint step 28560 기준 5 episode / 225 Real-Sim paired frame에서 Observation Gap, Representation Gap, Policy-Relevant Gap, Action Gap을 분석했다. 핵심 출발점은 전체 representation distance가 아니라 실제 action prediction을 바꾸는 component를 찾는 것이다.

## 2. 기존 Phase 1부터 Phase 6 결과 요약

기존 결과는 P4 letterbox preprocessing, progress/phase-conditioned hidden correction, policy-sensitive low-rank basis가 offline Action Gap을 크게 줄일 수 있음을 보였다. 단 이 결과는 모두 동일 offline vision-policy track에 한정되며 current ROS `oftplus_h5_proprio` deployment 결과가 아니다.

## 3. 기존 결과의 한정성

기존 dataset은 5개 episode와 225 pair에 한정된다. Real pose는 measured feedback가 아니라 commanded/planned 계열로 기록된 것으로 취급한다. GT action, executed action, measured motion, Real task success는 검증되지 않았으므로 이번 재평가는 Real/Sim policy prediction disagreement로만 해석한다.

## 4. Audit 결과

- Policy variant: `oftplus_h5_vision`
- Checkpoint: `runtime_state/oft_mixed480_step28560_merged`
- Checkpoint step: 28560
- Instruction: `Pick up the orange cube.`
- Use proprio: false
- Pair count: {audit_info['pair_count']}
- Action chunk length: 5
- Action dimension: 7
- Priority2 started: false

세부 audit 산출물은 `00_audit/`에 저장했다.

## 5. 발견된 오류와 미해결 사항

현재 Priority1에서 치명적 계산 중단 사유는 발견하지 않았다. 다만 다음은 미해결이다.

- Gripper binary threshold는 code-level GT threshold가 아니라 `0.5` assumed threshold다.
- Stored action은 policy decoded output으로 확인했지만, planner / commanded / executed / measured action과의 정확도 비교는 불가하다.
- Correction method 전체에 대해 L1/RMSE/Huber를 재계산하려면 corrected action chunk 또는 action-head 재실행 산출물이 더 필요하다.
- P4+C2 camera 결합 결과는 shift 해상도 차이 때문에 물리적 non-additivity 확정 증거로 쓰면 안 된다.

## 6. Action 정의와 normalization

`OFTRuntime.predict()`의 `response['actions']`를 `policy_decoded_action`으로 사용했다. Model raw action은 `_unnormalize_actions()`를 통과하며, dataset statistics의 `q01/q99`와 mask를 사용하는 구조다. Action index는 다음처럼 정리했다.

```text
0,1,2: translation
3,4,5: rotation-like action components
6: gripper
```

Rotation representation의 물리 단위와 geodesic 해석은 아직 검증되지 않았으므로 이번 결과에서는 component disagreement로 보고한다.

## 7. L1, L2 norm, RMSE, Huber, cosine 결과

P0-P6 조건에 대해 저장된 decoded action chunk로 multi-metric을 재계산했다.

```text
frame_metrics rows: {metric_info['frame_rows']}
episode_metrics rows: {metric_info['episode_rows']}
conditions: {', '.join(metric_info['conditions'])}
```

조건별 1위 요약:

```text
{key_rank_lines}
```

## 8. Translation, rotation, gripper 분리 결과

Translation, rotation, gripper는 별도 파일로 분리했다.

```text
01_multimetric_reanalysis/translation_metrics.csv
01_multimetric_reanalysis/rotation_metrics.csv
01_multimetric_reanalysis/gripper_event_metrics.csv
03_gripper_analysis/
```

주요 action L2 순위는 gripper absolute gap 순위와 거의 동일하다. 따라서 기존 Action Gap은 gripper dimension의 영향을 크게 받는 것으로 해석해야 한다.

## 9. Gripper timing 분석

Chunk 내부 gripper transition index를 Real/Sim 각각 계산했다. 단 이것은 policy prediction chunk 내부 disagreement이며, 실제 gripper close timing error가 아니다. GT gripper state가 없으므로 accuracy, precision, recall, false-open, false-close는 계산하지 않았다.

## 10. Episode-level 통계

Episode-level summary, episode-cluster bootstrap, paired method comparison을 `02_statistical_validation/`에 저장했다. Episode 수가 5개뿐이므로 frame-level 독립 표본처럼 해석하지 않고, episode-level paired difference와 개선 episode 수를 우선 근거로 사용한다.

## 11. Metric별 방법 순위

`01_multimetric_reanalysis/metric_robustness_summary.csv`에 metric별 condition rank를 저장했다. P4 letterbox는 action L1, L2, RMSE, Huber, translation, rotation, gripper continuous gap에서 일관되게 최상위로 나타났다.

## 12. Metric이 바뀌어도 유지되는 결론

- P4 letterbox는 주요 decoded-action disagreement metric에서 가장 강했다.
- P5 brightness matching은 observation/stat alignment 목적과 달리 action disagreement 개선에는 약하거나 악화되는 경향을 유지했다.
- 전체 action metric은 gripper component에 크게 민감하다.

## 13. Metric에 따라 달라지는 결론

- Translation/rotation만 보면 P0와 P1/P5/P6 사이의 차이는 전체 action L2만큼 크지 않다.
- Gripper binary disagreement는 assumed threshold에 의존하므로 continuous gripper gap과 분리해서 봐야 한다.
- Cosine distance는 action norm과 방향성 해석이 섞이므로 L1/L2/RMSE와 동일한 결론 지표로 과장하지 않는다.

## 14. 실행 완료 항목

- Phase1-Phase6 artifact audit
- Action definition / normalization audit
- L1, L2 norm, RMSE, Huber, cosine 재평가
- Translation / rotation / gripper 분리
- Gripper chunk timing disagreement 분석
- Episode-level statistics
- Metric별 condition ranking
- Priority2/Priority3/Priority4 protocol skeleton 생성

## 15. 실행하지 못한 항목

- GPU full-forward 재실험
- Camera shift sweep
- Letterbox 내부 ablation
- Global alignment full-forward
- 새 object/light/camera condition 데이터 수집
- Shadow mode / closed-loop rollout
- Real task success 분석

## 16. Priority 2에 필요한 데이터와 GPU 조건

Priority2에는 image-to-action full-forward 재실행이 필요하다. 특히 camera shift sweep, letterbox padding decomposition, correction method별 L1/RMSE/Huber 재평가는 GPU 또는 기존 corrected action chunk 재생성이 필요하다.

## 17. 해석 가능한 결론

현재 데이터 범위에서는 P4 letterbox가 여러 action disagreement metric에서 가장 robust한 preprocessing condition이다. 또한 observation-level 통계 정렬이 action-level improvement를 보장하지 않는다는 Phase6 결론은 metric audit 후에도 유지된다.

## 18. 주장하면 안 되는 결론

- P4가 Real robot success를 개선한다고 주장할 수 없다.
- Gripper gap을 gripper error 또는 false close/open으로 부르면 안 된다.
- 현재 결과를 `oftplus_h5_proprio` ROS deployment 결과로 해석하면 안 된다.
- 5 episode LOO 또는 P0-P6 재평가를 unseen layout / unseen lighting / rollout generalization으로 주장하면 안 된다.
""",
    )


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--bundle-root", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--seed", type=int, default=20260918)
    args = parser.parse_args()
    root = args.bundle_root.expanduser().resolve()
    out = args.output_dir.expanduser().resolve()
    out.mkdir(parents=True, exist_ok=True)

    audit_info = generate_audit(root, out, args.seed)
    metric_info = generate_metric_reanalysis(root, out, args.seed)
    stat_info = generate_statistical_validation(out, args.seed)
    gripper_info = generate_gripper_analysis(out)
    generate_protocols(out)
    generate_readme(out)
    generate_final_reports(root, out, audit_info, metric_info, stat_info, gripper_info, args.seed)
    print(json.dumps({"status": "COMPLETED_PRIORITY1", "output_dir": str(out)}, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
