#!/usr/bin/env python3
"""Analyze Phase 8 policy response action gap by component/chunk/phase."""

from __future__ import annotations

import argparse
import csv
import json
import math
from pathlib import Path
from statistics import mean, median, pstdev
from typing import Any

import numpy as np


DEFAULT_BUNDLE = Path("/home/ubuntu/a0509_vla_linux_field_bundle_20260903")
GRIPPER_THRESHOLD = 0.5


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


def read_json(path: Path) -> Any:
    return json.loads(path.read_text(encoding="utf-8"))


def frame_id_from_path(path: str) -> int:
    return int(Path(path).stem)


def episode_id_from_path(path: str) -> str | None:
    for part in Path(path).parts:
        if part.startswith("episode_"):
            return part
    return None


def load_records(manifest_path: Path, domain: str, output_root: Path) -> dict[tuple[str, int], dict[str, Any]]:
    manifest = read_json(manifest_path)
    records: dict[tuple[str, int], dict[str, Any]] = {}
    for record in manifest["records"]:
        source_image = record["source_image"]
        episode_id = episode_id_from_path(source_image)
        if episode_id is None and domain == "sim":
            episode_id = "episode_000004"
        if episode_id is None:
            raise ValueError(f"Could not infer episode id from {source_image}")
        frame = frame_id_from_path(source_image)
        feature_file = output_root / record["feature_file"]
        records[(episode_id, frame)] = {**record, "_feature_path": str(feature_file)}
    return records


def action_chunk(record: dict[str, Any]) -> np.ndarray:
    response = record.get("response", {})
    actions = response.get("actions")
    if actions is None:
        actions = [response.get("action")]
    arr = np.asarray(actions, dtype=np.float64)
    if arr.ndim == 1:
        arr = arr.reshape(1, -1)
    if arr.ndim != 2 or arr.shape[1] != 7:
        raise ValueError(f"Unexpected action shape for {record.get('source_image')}: {arr.shape}")
    return arr


def load_raw_action_head(record: dict[str, Any]) -> np.ndarray:
    with np.load(record["_feature_path"], allow_pickle=True) as data:
        arr = np.asarray(data["action_head.output"], dtype=np.float64)
    if arr.ndim == 3 and arr.shape[0] == 1:
        arr = arr[0]
    if arr.ndim != 2 or arr.shape[1] != 7:
        raise ValueError(f"Unexpected raw action_head.output shape: {arr.shape}")
    return arr


def l2(vec: np.ndarray) -> float:
    return float(np.linalg.norm(vec))


def rmse(vec: np.ndarray) -> float:
    return float(np.sqrt(np.mean(vec * vec)))


def cosine_distance(a: np.ndarray, b: np.ndarray) -> float | None:
    denom = float(np.linalg.norm(a) * np.linalg.norm(b))
    if denom <= 1e-12:
        return None
    return float(1.0 - np.dot(a, b) / denom)


def first_close_index(values: np.ndarray) -> int | None:
    binary = values >= GRIPPER_THRESHOLD
    idx = np.flatnonzero(binary)
    if idx.size == 0:
        return None
    return int(idx[0])


def safe_mean(values: list[float]) -> float | None:
    values = [v for v in values if math.isfinite(v)]
    return mean(values) if values else None


def aggregate(rows: list[dict[str, Any]], group_keys: list[str], metric_keys: list[str]) -> list[dict[str, Any]]:
    groups: dict[tuple[Any, ...], list[dict[str, Any]]] = {}
    for row in rows:
        key = tuple(row.get(k) for k in group_keys)
        groups.setdefault(key, []).append(row)
    out: list[dict[str, Any]] = []
    for key, subset in sorted(groups.items(), key=lambda item: tuple(str(x) for x in item[0])):
        result = {k: v for k, v in zip(group_keys, key)}
        result["count"] = len(subset)
        for metric in metric_keys:
            vals = []
            for row in subset:
                value = row.get(metric)
                try:
                    value = float(value)
                except Exception:
                    continue
                if math.isfinite(value):
                    vals.append(value)
            if vals:
                result[f"{metric}_mean"] = mean(vals)
                result[f"{metric}_median"] = median(vals)
                result[f"{metric}_std"] = pstdev(vals) if len(vals) > 1 else 0.0
                result[f"{metric}_min"] = min(vals)
                result[f"{metric}_max"] = max(vals)
            else:
                result[f"{metric}_mean"] = None
                result[f"{metric}_median"] = None
                result[f"{metric}_std"] = None
                result[f"{metric}_min"] = None
                result[f"{metric}_max"] = None
        out.append(result)
    return out


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--phase8-root", type=Path, default=DEFAULT_BUNDLE / "lhj" / "phase8_shadow_mode_distribution_analysis")
    args = parser.parse_args()

    phase8 = args.phase8_root.expanduser().resolve()
    feature_root = phase8 / "06_representation_gap" / "full_forward_features"
    real_manifest = feature_root / "real" / "feature_manifest.json"
    sim_manifest = feature_root / "sim_episode_000004_reference" / "feature_manifest.json"
    if not real_manifest.exists() or not sim_manifest.exists():
        raise FileNotFoundError("Feature manifests are missing. Run Phase 8 feature extraction first.")

    real_records = load_records(real_manifest, "real", real_manifest.parent)
    sim_records = load_records(sim_manifest, "sim", sim_manifest.parent)
    pairs = [
        row for row in read_csv(phase8 / "03_pair_alignment" / "aligned_pairs.csv")
        if row.get("pair_valid", "").lower() == "true"
    ]

    def record_for(domain: str, episode_id: str, frame: int) -> dict[str, Any]:
        if domain == "sim":
            return sim_records[("episode_000004", frame)]
        return real_records[(episode_id, frame)]

    frame_rows: list[dict[str, Any]] = []
    chunk_rows: list[dict[str, Any]] = []
    for pair in pairs:
        left_record = record_for(pair["left_domain"], pair["left_episode"], int(pair["left_frame"]))
        right_record = record_for(pair["right_domain"], pair["right_episode"], int(pair["right_frame"]))
        left_actions = action_chunk(left_record)
        right_actions = action_chunk(right_record)
        chunk_count = min(len(left_actions), len(right_actions))
        left_actions = left_actions[:chunk_count]
        right_actions = right_actions[:chunk_count]
        delta = left_actions - right_actions

        raw_left = load_raw_action_head(left_record)[:chunk_count]
        raw_right = load_raw_action_head(right_record)[:chunk_count]
        raw_delta = raw_left - raw_right

        per_chunk_total = np.linalg.norm(delta, axis=1)
        per_chunk_translation = np.linalg.norm(delta[:, :3], axis=1)
        per_chunk_rotation = np.linalg.norm(delta[:, 3:6], axis=1)
        per_chunk_gripper = np.abs(delta[:, 6])
        left_gripper_binary = left_actions[:, 6] >= GRIPPER_THRESHOLD
        right_gripper_binary = right_actions[:, 6] >= GRIPPER_THRESHOLD
        gripper_disagreement = left_gripper_binary != right_gripper_binary
        left_close = first_close_index(left_actions[:, 6])
        right_close = first_close_index(right_actions[:, 6])
        if left_close is None or right_close is None:
            close_timing_error = None
        else:
            close_timing_error = abs(left_close - right_close)

        first_delta = delta[0]
        first_left = left_actions[0]
        first_right = right_actions[0]
        frame_row = {
            "comparison_id": pair["comparison_id"],
            "phase": pair["left_phase"],
            "left_domain": pair["left_domain"],
            "left_episode": pair["left_episode"],
            "left_frame": int(pair["left_frame"]),
            "right_domain": pair["right_domain"],
            "right_episode": pair["right_episode"],
            "right_frame": int(pair["right_frame"]),
            "chunk_count": chunk_count,
            "chunk_mean_l2": float(per_chunk_total.mean()),
            "chunk_max_l2": float(per_chunk_total.max()),
            "chunk_mean_translation_l2": float(per_chunk_translation.mean()),
            "chunk_max_translation_l2": float(per_chunk_translation.max()),
            "chunk_mean_rotation_l2": float(per_chunk_rotation.mean()),
            "chunk_max_rotation_l2": float(per_chunk_rotation.max()),
            "chunk_mean_gripper_abs": float(per_chunk_gripper.mean()),
            "chunk_max_gripper_abs": float(per_chunk_gripper.max()),
            "first_action_l2": l2(first_delta),
            "first_translation_l2": l2(first_delta[:3]),
            "first_rotation_l2": l2(first_delta[3:6]),
            "first_gripper_abs": float(abs(first_delta[6])),
            "first_translation_cosine_distance": cosine_distance(first_left[:3], first_right[:3]),
            "first_rotation_cosine_distance": cosine_distance(first_left[3:6], first_right[3:6]),
            "gripper_disagreement_rate": float(gripper_disagreement.mean()),
            "gripper_any_disagreement": bool(gripper_disagreement.any()),
            "left_first_close_chunk": left_close,
            "right_first_close_chunk": right_close,
            "gripper_close_timing_abs_error_chunks": close_timing_error,
            "raw_action_head_chunk_mean_l2": float(np.linalg.norm(raw_delta, axis=1).mean()),
            "raw_action_head_first_l2": l2(raw_delta[0]),
        }
        frame_rows.append(frame_row)

        for chunk_index in range(chunk_count):
            d = delta[chunk_index]
            chunk_rows.append({
                "comparison_id": pair["comparison_id"],
                "phase": pair["left_phase"],
                "left_episode": pair["left_episode"],
                "left_frame": int(pair["left_frame"]),
                "right_episode": pair["right_episode"],
                "right_frame": int(pair["right_frame"]),
                "chunk_index": chunk_index,
                "action_l1": float(np.abs(d).sum()),
                "action_l2": l2(d),
                "action_rmse": rmse(d),
                "translation_l2": l2(d[:3]),
                "rotation_l2": l2(d[3:6]),
                "gripper_abs": float(abs(d[6])),
                "x_abs": float(abs(d[0])),
                "y_abs": float(abs(d[1])),
                "z_abs": float(abs(d[2])),
                "rx_abs": float(abs(d[3])),
                "ry_abs": float(abs(d[4])),
                "rz_abs": float(abs(d[5])),
                "left_gripper": float(left_actions[chunk_index, 6]),
                "right_gripper": float(right_actions[chunk_index, 6]),
                "gripper_binary_disagreement": bool(gripper_disagreement[chunk_index]),
            })

    metric_keys = [
        "chunk_mean_l2",
        "chunk_max_l2",
        "chunk_mean_translation_l2",
        "chunk_max_translation_l2",
        "chunk_mean_rotation_l2",
        "chunk_max_rotation_l2",
        "chunk_mean_gripper_abs",
        "chunk_max_gripper_abs",
        "first_action_l2",
        "first_translation_l2",
        "first_rotation_l2",
        "first_gripper_abs",
        "gripper_disagreement_rate",
        "gripper_close_timing_abs_error_chunks",
        "raw_action_head_chunk_mean_l2",
        "raw_action_head_first_l2",
    ]
    chunk_metric_keys = [
        "action_l1",
        "action_l2",
        "action_rmse",
        "translation_l2",
        "rotation_l2",
        "gripper_abs",
        "x_abs",
        "y_abs",
        "z_abs",
        "rx_abs",
        "ry_abs",
        "rz_abs",
    ]

    out_dir = phase8 / "07_shadow_action_analysis"
    out_dir.mkdir(parents=True, exist_ok=True)
    write_csv(out_dir / "action_frame_metrics.csv", frame_rows)
    write_csv(out_dir / "action_chunk_metrics.csv", chunk_rows)
    write_csv(out_dir / "action_comparison_summary.csv", aggregate(frame_rows, ["comparison_id"], metric_keys))
    write_csv(out_dir / "action_phase_summary.csv", aggregate(frame_rows, ["comparison_id", "phase"], metric_keys))
    write_csv(out_dir / "action_chunk_index_summary.csv", aggregate(chunk_rows, ["comparison_id", "chunk_index"], chunk_metric_keys))

    comparison_summary = aggregate(frame_rows, ["comparison_id"], metric_keys)
    summary = {
        "status": "COMPLETED",
        "action_source": "response.actions from oftplus_h5_vision full-forward feature_manifest",
        "raw_action_head_output": "also summarized as raw_action_head_* but not used as primary decoded action metric",
        "gripper_threshold": GRIPPER_THRESHOLD,
        "frame_rows": len(frame_rows),
        "chunk_rows": len(chunk_rows),
        "comparison_count": len(comparison_summary),
        "outputs": {
            "frame_metrics": str(out_dir / "action_frame_metrics.csv"),
            "chunk_metrics": str(out_dir / "action_chunk_metrics.csv"),
            "comparison_summary": str(out_dir / "action_comparison_summary.csv"),
            "phase_summary": str(out_dir / "action_phase_summary.csv"),
            "chunk_index_summary": str(out_dir / "action_chunk_index_summary.csv"),
        },
        "interpretation_guardrail": "This is offline policy output disagreement, not planner-relative correctness or rollout success.",
    }
    (out_dir / "summary.json").write_text(json.dumps(summary, indent=2), encoding="utf-8")

    report = [
        "# Phase 8 Action Gap Analysis",
        "",
        "## Method",
        "",
        "- Primary action source: `response.actions` from `oftplus_h5_vision` full-forward feature manifests.",
        "- Action shape: `K=5`, `dim=7`.",
        "- Dimensions: translation xyz, rotation rx/ry/rz, gripper.",
        f"- Binary gripper disagreement uses threshold `{GRIPPER_THRESHOLD}` and should be treated as an analysis convention.",
        "",
        "## Comparison Summary",
        "",
        "| Comparison | Chunk mean L2 | Translation | Rotation | Gripper abs | First L2 | Gripper disagreement |",
        "|---|---:|---:|---:|---:|---:|---:|",
    ]
    for row in comparison_summary:
        report.append(
            f"| {row['comparison_id']} | {row['chunk_mean_l2_mean']:.6f} | "
            f"{row['chunk_mean_translation_l2_mean']:.6f} | {row['chunk_mean_rotation_l2_mean']:.6f} | "
            f"{row['chunk_mean_gripper_abs_mean']:.6f} | {row['first_action_l2_mean']:.6f} | "
            f"{row['gripper_disagreement_rate_mean']:.6f} |"
        )
    report.extend([
        "",
        "## Interpretation Guardrails",
        "",
        "- This measures policy output disagreement between two image conditions.",
        "- It does not say which output is correct unless planner or rollout ground truth is introduced.",
        "- Component-level differences should be interpreted with representation and observation gaps, not alone.",
    ])
    (out_dir / "action_gap_report.md").write_text("\n".join(report) + "\n", encoding="utf-8")
    print(json.dumps(summary, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
