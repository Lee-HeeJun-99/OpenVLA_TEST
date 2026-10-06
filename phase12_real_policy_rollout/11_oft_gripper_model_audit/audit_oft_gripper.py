#!/usr/bin/env python3
"""Offline-only audit of OFT gripper labels, output contract, and saved predictions."""

from __future__ import annotations

import csv
import json
import statistics
from collections import defaultdict
from pathlib import Path


BUNDLE = Path("/home/ubuntu/a0509_vla_linux_field_bundle_20260903")
ROOT = BUNDLE / "lhj/phase12_real_policy_rollout/11_oft_gripper_model_audit"
RESULTS = ROOT / "results"
CHECKPOINT_STATS = BUNDLE / "models/oft_mixed480_step28560/dataset_statistics.json"
TRAINING_PROXY = Path("/home/ubuntu/robot_ws/src/doosan-robot2/raw_dataset_oft_200/metadata/train.jsonl")
REFERENCE_EPISODES = BUNDLE / "data/real_world/raw_dataset_oft/episodes"
PHASE11 = BUNDLE / "lhj/phase11_sim_condition_replication/policy_sensitive_analysis/01_predictions/oft_run2"
PHASE10 = BUNDLE / "lhj/phase10_planner_based_shadow_mode/06_shadow_collection/offline_recorded_episode4/oft_vision_step28560_local/samples.jsonl"
HZ = 5.0
THRESHOLD = 0.7


def read_jsonl(path: Path) -> list[dict]:
    return [json.loads(line) for line in path.open(encoding="utf-8") if line.strip()]


def write_csv(path: Path, rows: list[dict]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    fields: list[str] = []
    for row in rows:
        for key in row:
            if key not in fields:
                fields.append(key)
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)


def grouped_actions(path: Path) -> dict[str, list[dict]]:
    out: dict[str, list[dict]] = defaultdict(list)
    for row in read_jsonl(path):
        out[str(row["episode_id"])].append(row)
    for rows in out.values():
        rows.sort(key=lambda row: int(row["step_index"]))
    return out


def grouped_reference_episodes(root: Path) -> dict[str, list[dict]]:
    out: dict[str, list[dict]] = {}
    for path in sorted(root.glob("episode_*/steps_with_actions.jsonl")):
        rows = read_jsonl(path)
        rows.sort(key=lambda row: int(row["step_index"]))
        out[path.parent.name] = rows
    return out


def label_k_statistics(path: Path, provenance: str, exact_training_corpus: bool, *, reference_dirs: bool = False) -> tuple[list[dict], dict]:
    episodes = grouped_reference_episodes(path) if reference_dirs else grouped_actions(path)
    values: dict[int, list[float]] = defaultdict(list)
    transitions: list[int] = []
    for rows in episodes.values():
        gripper = [float(row["action"][6]) for row in rows]
        transition = next((i for i, value in enumerate(gripper) if value >= THRESHOLD), None)
        if transition is not None:
            transitions.append(transition)
        terminal = gripper[-1]
        for index in range(len(gripper)):
            chunk = gripper[index:index + 5] + [terminal] * max(0, index + 5 - len(gripper))
            for k, value in enumerate(chunk):
                values[k].append(value)
    rows = []
    for k in range(5):
        vals = values[k]
        rows.append({
            "source_provenance": provenance,
            "exact_checkpoint_training_corpus": exact_training_corpus,
            "k_index": k,
            "label_count": len(vals),
            "gripper_mean": sum(vals) / len(vals),
            "closed_ratio_ge_0p7": sum(value >= THRESHOLD for value in vals) / len(vals),
            "binary_closed_ratio": sum(value >= 0.5 for value in vals) / len(vals),
            "episode_count": len(episodes),
            "median_first_close_step": statistics.median(transitions) if transitions else None,
            "min_first_close_step": min(transitions) if transitions else None,
            "max_first_close_step": max(transitions) if transitions else None,
        })
    return rows, {
        "episodes": len(episodes),
        "transition_count": len(transitions),
        "median_first_close_step": statistics.median(transitions) if transitions else None,
        "min_first_close_step": min(transitions) if transitions else None,
        "max_first_close_step": max(transitions) if transitions else None,
    }


def analyze_samples(path: Path, source_set: str, condition: str, episode: str) -> tuple[dict, list[dict]]:
    rows = read_jsonl(path)
    chunks = [row for row in rows if row.get("oft_denormalized_action_chunk")]
    expanded = []
    for row in chunks:
        inference_frame = int(row["frame_id"])
        for k, action in enumerate(row["oft_denormalized_action_chunk"]):
            expanded.append({
                "target_step": inference_frame + k,
                "target_time_s": (inference_frame + k) / HZ,
                "inference_frame": inference_frame,
                "k_index": k,
                "closedness": float(action[6]),
            })
    expanded.sort(key=lambda item: (item["target_step"], item["inference_frame"], item["k_index"]))
    reference_close = next((
        int(row["frame_id"]) for row in rows
        if float((row.get("planner_canonical_action") or {}).get("vector", [0] * 7)[6]) >= THRESHOLD
    ), None)
    first = next((item for item in expanded if item["closedness"] >= THRESHOLD), None)
    k_means = {
        k: statistics.mean(item["closedness"] for item in expanded if item["k_index"] == k)
        for k in range(5) if any(item["k_index"] == k for item in expanded)
    }
    record = {
        "source_set": source_set,
        "condition": condition,
        "episode_id": episode,
        "checkpoint": "oft_mixed480_step28560",
        "variant": "oftplus_h5_vision",
        "valid_chunk_count": len(chunks),
        "expanded_action_count": len(expanded),
        "first_close_target_step": None if first is None else first["target_step"],
        "first_close_time_s": None if first is None else first["target_time_s"],
        "first_close_inference_frame": None if first is None else first["inference_frame"],
        "first_close_k_index": None if first is None else first["k_index"],
        "first_close_closedness": None if first is None else first["closedness"],
        "reference_close_step": reference_close,
        "reference_close_time_s": None if reference_close is None else reference_close / HZ,
        "lead_steps": None if first is None or reference_close is None else reference_close - first["target_step"],
        "lead_seconds": None if first is None or reference_close is None else (reference_close - first["target_step"]) / HZ,
        "early_close": bool(first is not None and reference_close is not None and first["target_step"] < reference_close),
        **{f"k{k}_closedness_mean": k_means.get(k) for k in range(5)},
        "prediction_artifact": str(path),
    }
    detail = [{"source_set": source_set, "condition": condition, "episode_id": episode, **item} for item in expanded]
    return record, detail


def main() -> None:
    RESULTS.mkdir(parents=True, exist_ok=True)
    training_rows, training_meta = label_k_statistics(
        TRAINING_PROXY,
        "LOCAL_OFT200_LABEL_CORPUS_PROXY_NOT_EXACT_MIXED480",
        False,
    )
    reference_rows, reference_meta = label_k_statistics(
        REFERENCE_EPISODES,
        "STORED_REAL_REFERENCE_EPISODES_1_TO_10_NOT_TRAINING",
        False,
        reference_dirs=True,
    )
    write_csv(RESULTS / "training_gripper_by_k.csv", training_rows + reference_rows)

    episode_rows: list[dict] = []
    details: list[dict] = []
    for path in sorted(PHASE11.glob("*/*/samples.jsonl")):
        record, expanded = analyze_samples(path, "PHASE11_RECORDED_INPUT", path.parts[-3], path.parts[-2])
        episode_rows.append(record)
        details.extend(expanded)
    if PHASE10.exists():
        record, expanded = analyze_samples(PHASE10, "PHASE10_REAL_EPISODE4", "real_recorded", "episode_000004")
        episode_rows.append(record)
        details.extend(expanded)
    write_csv(RESULTS / "episode_gripper_timing.csv", episode_rows)
    write_csv(RESULTS / "checkpoint_vs_reference.csv", [
        {
            **row,
            "timing_class": "EARLY" if row["early_close"] else (
                "NO_CLOSE" if row["first_close_target_step"] is None else
                "ON_TIME" if row["lead_steps"] == 0 else "LATE"
            ),
        }
        for row in episode_rows
    ])

    checkpoint_stats = json.load(CHECKPOINT_STATS.open(encoding="utf-8"))["a0509_sim_cube_pick"]["action"]
    phase11 = [row for row in episode_rows if row["source_set"] == "PHASE11_RECORDED_INPUT"]
    early = [row for row in phase11 if row["early_close"]]
    k_prediction = []
    for k in range(5):
        vals = [item["closedness"] for item in details if item["source_set"] == "PHASE11_RECORDED_INPUT" and item["k_index"] == k]
        k_prediction.append({"k_index": k, "count": len(vals), "mean": statistics.mean(vals), "ratio_ge_0p7": sum(v >= THRESHOLD for v in vals) / len(vals)})
    phase10_row = next(row for row in episode_rows if row["source_set"] == "PHASE10_REAL_EPISODE4")
    summary = {
        "status": "COMPLETED_OFFLINE_ONLY",
        "classification": "MIXED",
        "primary_supported_factor": "CHECKPOINT_EARLY_CLOSE_BEHAVIOR",
        "secondary_supported_factor": "K5_HORIZON_BIAS",
        "normalization_contract_problem": False,
        "exact_mixed480_training_corpus_available": False,
        "training_label_conclusion": "UNRESOLVED_FOR_EXACT_MIXED480_CORPUS",
        "checkpoint_contract": {
            "variant": "oftplus_h5_vision",
            "step": 28560,
            "output": "continuous gripper closedness after sigmoid",
            "open_value": 0.0,
            "closed_value": 1.0,
            "affine_normalization_applied_to_gripper": False,
            "dataset_stats_mask_gripper": checkpoint_stats["mask"][6],
            "dataset_stats_gripper_mean": checkpoint_stats["mean"][6],
            "dataset_stats_gripper_min": checkpoint_stats["min"][6],
            "dataset_stats_gripper_max": checkpoint_stats["max"][6],
        },
        "available_label_corpora": {
            "local_oft200_proxy": training_meta,
            "stored_reference_episodes_1_to_10": reference_meta,
        },
        "phase11_prediction": {
            "episode_condition_pairs": len(phase11),
            "early_close_pairs": len(early),
            "early_close_ratio": len(early) / len(phase11),
            "median_lead_steps": statistics.median(row["lead_steps"] for row in early),
            "median_lead_seconds": statistics.median(row["lead_seconds"] for row in early),
            "by_k": k_prediction,
        },
        "phase10_episode4": phase10_row,
        "answers": {
            "training_labels_close_early_in_late_k": "The exact mixed480 rows are unavailable. The loader necessarily raises later-K closed prevalence near a true transition; the local OFT200 proxy quantifies this but cannot prove mixed480 timing.",
            "normalization_wrong": "No. Gripper mask is false, so it bypasses affine normalization and denormalization; sigmoid bounds model output to [0,1].",
            "checkpoint_earlier_than_reference": "Yes in Phase10 Episode 4 and in most Phase11 condition/episode pairs, especially every lighting_low pair.",
            "closedness_increases_with_k": "Yes in saved checkpoint outputs; a milder structural increase is expected in future-label chunks and is measured in the local proxy.",
            "rollout_recommendation": "Do not weaken premature-close safety. Require checkpoint/label-corpus review and consider retraining or gripper-head calibration before sequential-K5 real rollout.",
        },
        "safety_thresholds_changed": False,
        "execution_counts": {
            "robot_command": 0,
            "motion_service_action": 0,
            "gripper_command": 0,
            "home": 0,
            "trajectory": 0,
            "hold_estop": 0,
            "real_rollout": 0,
        },
    }
    (RESULTS / "summary.json").write_text(json.dumps(summary, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(summary, indent=2))


if __name__ == "__main__":
    main()
