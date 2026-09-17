#!/usr/bin/env python3
from __future__ import annotations

import csv
import json
import sys
from pathlib import Path
from typing import Any

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np


ROOT = Path("/home/ubuntu/a0509_vla_linux_field_bundle_20260903")
LHJ = ROOT / "lhj"
SCRIPT_DIR = LHJ / "scripts"
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

from action_hidden_token_sensitivity import (  # noqa: E402
    CHECKPOINT,
    DATASET_KEY,
    FEATURE,
    action_chunk,
    chunk_metrics,
    episode_id_from_frame,
    load_action_head,
    load_npz_array,
    predict_batch,
    progress_from_frame,
    read_json,
    records_by_frame,
    write_csv,
    write_json,
)


PHASE6 = LHJ / "phase6_environment_attribution"
FEATURE_ROOT = PHASE6 / "01_preprocessing" / "full_forward_features"
OUTPUT_DIR = PHASE6 / "05_combined" / "environment_x_hidden_correction"
PAIR_MANIFEST = ROOT / "outputs" / "token_distribution_analysis" / "5_episodes" / "paired_inputs" / "paired_manifest.json"
WINDOW = 11
PROGRESS_BIN = 4


def pair_metadata() -> dict[str, dict[str, Any]]:
    pairs = read_json(PAIR_MANIFEST)["pairs"]
    return {
        pair["pair_id"]: {
            "episode_id": pair["episode_id"],
            "progress": int(pair["step_index"]),
            "planner_phase": pair.get("planner_phase", ""),
        }
        for pair in pairs
    }


def load_condition(condition: str) -> list[dict[str, Any]]:
    real = records_by_frame(FEATURE_ROOT / condition / "real" / "feature_manifest.json")
    sim = records_by_frame(FEATURE_ROOT / condition / "sim" / "feature_manifest.json")
    meta = pair_metadata()
    frames = sorted(set(real) & set(sim) & set(meta))
    samples = []
    for frame in frames:
        real_hidden = load_npz_array(real[frame], FEATURE).astype(np.float64)
        sim_hidden = load_npz_array(sim[frame], FEATURE).astype(np.float64)
        real_action = action_chunk(real[frame])
        sim_action = action_chunk(sim[frame])
        samples.append(
            {
                "frame": frame,
                "episode_id": meta[frame]["episode_id"],
                "progress": meta[frame]["progress"],
                "planner_phase": meta[frame]["planner_phase"],
                "real_hidden": real_hidden,
                "sim_hidden": sim_hidden,
                "delta": sim_hidden - real_hidden,
                "real_action": real_action,
                "sim_action": sim_action,
            }
        )
    return samples


def mean_shift(samples: list[dict[str, Any]]) -> np.ndarray:
    return np.mean([sample["delta"] for sample in samples], axis=0)


def progress_phase_shift(train: list[dict[str, Any]], progress: int, phase: str) -> np.ndarray:
    bin_id = progress // PROGRESS_BIN
    selected = [
        sample
        for sample in train
        if sample["planner_phase"] == phase and int(sample["progress"]) // PROGRESS_BIN == bin_id
    ]
    if not selected:
        selected = [sample for sample in train if int(sample["progress"]) // PROGRESS_BIN == bin_id]
    if not selected:
        same_phase = [sample for sample in train if sample["planner_phase"] == phase]
        candidates = same_phase if same_phase else train
        selected = [sample for sample in candidates if abs(int(sample["progress"]) - progress) <= WINDOW // 2]
    if not selected:
        candidates = sorted(train, key=lambda sample: abs(int(sample["progress"]) - progress))
        selected = candidates[: max(1, min(10, len(candidates)))]
    return mean_shift(selected)


def aggregate(rows: list[dict[str, Any]], group_keys: list[str]) -> list[dict[str, Any]]:
    grouped: dict[tuple[Any, ...], list[dict[str, Any]]] = {}
    for row in rows:
        grouped.setdefault(tuple(row[key] for key in group_keys), []).append(row)
    metrics = [
        "raw_gap_chunk_mean_l2",
        "raw_gap_chunk_max_l2",
        "raw_gap_first_l2",
        "raw_gap_first_translation_l2",
        "raw_gap_first_rotation_l2",
        "raw_gap_first_gripper_abs",
        "gap_to_sim_chunk_mean_l2",
        "gap_to_sim_chunk_max_l2",
        "gap_to_sim_first_l2",
        "gap_to_sim_first_translation_l2",
        "gap_to_sim_first_rotation_l2",
        "gap_to_sim_first_gripper_abs",
        "repr_gap_reduction_ratio",
        "action_gap_reduction",
    ]
    out = []
    for key, items in sorted(grouped.items(), key=lambda kv: kv[0]):
        rec = {k: v for k, v in zip(group_keys, key)}
        rec["count"] = len(items)
        for metric in metrics:
            vals = np.asarray([float(item[metric]) for item in items], dtype=np.float64)
            rec[f"{metric}_mean"] = float(vals.mean())
            rec[f"{metric}_p50"] = float(np.percentile(vals, 50))
            rec[f"{metric}_p90"] = float(np.percentile(vals, 90))
        rec["improved_count"] = int(sum(float(item["action_gap_reduction"]) > 0 for item in items))
        rec["worsened_count"] = int(sum(float(item["action_gap_reduction"]) < 0 for item in items))
        out.append(rec)
    return out


def main() -> int:
    OUTPUT_DIR.mkdir(parents=True, exist_ok=True)
    action_stats = read_json(CHECKPOINT / "dataset_statistics.json")[DATASET_KEY]["action"]
    action_head, device, action_head_checkpoint = load_action_head(CHECKPOINT)

    original = load_condition("P0_current_paired_image")
    aligned = load_condition("P4_letterbox_224")
    by_condition = {
        "original_P0": original,
        "aligned_P4": aligned,
    }
    all_rows: list[dict[str, Any]] = []

    # Per-condition LOO correction: each condition estimates its own train-fold Sim target.
    for condition_name, samples in by_condition.items():
        episodes = sorted({sample["episode_id"] for sample in samples})
        for episode in episodes:
            train = [sample for sample in samples if sample["episode_id"] != episode]
            test = [sample for sample in samples if sample["episode_id"] == episode]
            corrected_hiddens = []
            shifts = []
            for sample in test:
                shift = progress_phase_shift(train, int(sample["progress"]), sample["planner_phase"])
                corrected_hiddens.append(sample["real_hidden"] + shift)
                shifts.append(shift)
            corrected_actions = predict_batch(action_head, np.stack(corrected_hiddens), action_stats, device)
            for sample, shift, corrected_action in zip(test, shifts, corrected_actions):
                raw_gap = chunk_metrics(sample["real_action"], sample["sim_action"], "raw_gap")
                corrected_gap = chunk_metrics(corrected_action, sample["sim_action"], "gap_to_sim")
                raw_repr = float(np.linalg.norm((sample["sim_hidden"] - sample["real_hidden"]).reshape(-1)))
                corrected_repr = float(np.linalg.norm((sample["sim_hidden"] - (sample["real_hidden"] + shift)).reshape(-1)))
                all_rows.append(
                    {
                        "condition": condition_name,
                        "correction_mode": "condition_specific_progress_phase",
                        "fold_episode": episode,
                        "frame": sample["frame"],
                        "episode_id": sample["episode_id"],
                        "progress": sample["progress"],
                        "planner_phase": sample["planner_phase"],
                        **raw_gap,
                        **corrected_gap,
                        "repr_gap_raw": raw_repr,
                        "repr_gap_corrected": corrected_repr,
                        "repr_gap_reduction_ratio": (raw_repr - corrected_repr) / raw_repr if raw_repr > 1e-12 else 0.0,
                        "action_gap_reduction": raw_gap["raw_gap_chunk_mean_l2"] - corrected_gap["gap_to_sim_chunk_mean_l2"],
                    }
                )

    # Transfer test: original-trained correction applied to P4 test samples.
    original_by_episode = {ep: [s for s in original if s["episode_id"] == ep] for ep in sorted({s["episode_id"] for s in original})}
    aligned_by_episode = {ep: [s for s in aligned if s["episode_id"] == ep] for ep in sorted({s["episode_id"] for s in aligned})}
    episodes = sorted(set(original_by_episode) & set(aligned_by_episode))
    for episode in episodes:
        train_original = [sample for sample in original if sample["episode_id"] != episode]
        test_aligned = aligned_by_episode[episode]
        corrected_hiddens = []
        shifts = []
        for sample in test_aligned:
            shift = progress_phase_shift(train_original, int(sample["progress"]), sample["planner_phase"])
            corrected_hiddens.append(sample["real_hidden"] + shift)
            shifts.append(shift)
        corrected_actions = predict_batch(action_head, np.stack(corrected_hiddens), action_stats, device)
        for sample, shift, corrected_action in zip(test_aligned, shifts, corrected_actions):
            raw_gap = chunk_metrics(sample["real_action"], sample["sim_action"], "raw_gap")
            corrected_gap = chunk_metrics(corrected_action, sample["sim_action"], "gap_to_sim")
            raw_repr = float(np.linalg.norm((sample["sim_hidden"] - sample["real_hidden"]).reshape(-1)))
            corrected_repr = float(np.linalg.norm((sample["sim_hidden"] - (sample["real_hidden"] + shift)).reshape(-1)))
            all_rows.append(
                {
                    "condition": "aligned_P4",
                    "correction_mode": "original_P0_trained_progress_phase_transfer",
                    "fold_episode": episode,
                    "frame": sample["frame"],
                    "episode_id": sample["episode_id"],
                    "progress": sample["progress"],
                    "planner_phase": sample["planner_phase"],
                    **raw_gap,
                    **corrected_gap,
                    "repr_gap_raw": raw_repr,
                    "repr_gap_corrected": corrected_repr,
                    "repr_gap_reduction_ratio": (raw_repr - corrected_repr) / raw_repr if raw_repr > 1e-12 else 0.0,
                    "action_gap_reduction": raw_gap["raw_gap_chunk_mean_l2"] - corrected_gap["gap_to_sim_chunk_mean_l2"],
                }
            )

    write_csv(OUTPUT_DIR / "environment_x_hidden_correction_frame_metrics.csv", all_rows)
    overall = aggregate(all_rows, ["condition", "correction_mode"])
    by_phase = aggregate(all_rows, ["condition", "correction_mode", "planner_phase"])
    write_csv(OUTPUT_DIR / "environment_x_hidden_correction_summary_overall.csv", overall)
    write_csv(OUTPUT_DIR / "environment_x_hidden_correction_summary_by_phase.csv", by_phase)

    # Also write the required root-level physical_x_hidden table.
    root_rows = []
    for row in overall:
        root_rows.append(
            {
                "environment_condition": row["condition"],
                "hidden_correction": row["correction_mode"],
                "action_gap": row["gap_to_sim_chunk_mean_l2_mean"],
                "action_gap_reduction_from_condition_raw": row["action_gap_reduction_mean"],
                "repr_gap_reduction_ratio": row["repr_gap_reduction_ratio_mean"],
                "worsened_frames": row["worsened_count"],
                "status": "VERIFIED_OFFLINE_FULL_FORWARD_ACTION_HEAD",
            }
        )
    write_csv(PHASE6 / "physical_x_hidden_correction.csv", root_rows)

    rows_by_key = {(row["condition"], row["correction_mode"]): row for row in overall}
    fig_rows = [
        ("Original raw", rows_by_key[("original_P0", "condition_specific_progress_phase")]["raw_gap_chunk_mean_l2_mean"]),
        ("Original + hidden", rows_by_key[("original_P0", "condition_specific_progress_phase")]["gap_to_sim_chunk_mean_l2_mean"]),
        ("P4 raw", rows_by_key[("aligned_P4", "condition_specific_progress_phase")]["raw_gap_chunk_mean_l2_mean"]),
        ("P4 + hidden", rows_by_key[("aligned_P4", "condition_specific_progress_phase")]["gap_to_sim_chunk_mean_l2_mean"]),
        ("P4 + P0 hidden", rows_by_key[("aligned_P4", "original_P0_trained_progress_phase_transfer")]["gap_to_sim_chunk_mean_l2_mean"]),
    ]
    plt.figure(figsize=(9.5, 4.8))
    plt.bar([label for label, _ in fig_rows], [value for _, value in fig_rows])
    plt.ylabel("chunk mean L2")
    plt.title("Environment Alignment x Hidden Correction 2x2")
    plt.xticks(rotation=20, ha="right")
    plt.tight_layout()
    (PHASE6 / "figures").mkdir(exist_ok=True)
    plt.savefig(PHASE6 / "figures" / "environment_x_hidden_correction_2x2.png", dpi=180)
    plt.close()

    summary = {
        "action_head_checkpoint": action_head_checkpoint,
        "rows": len(all_rows),
        "overall": overall,
        "status": "VERIFIED_OFFLINE_FULL_FORWARD_ACTION_HEAD",
        "notes": [
            "condition_specific_progress_phase estimates correction from the same environment condition train folds.",
            "original_P0_trained_progress_phase_transfer tests whether P0-trained correction transfers to P4 features.",
            "This is offline action-head injection, not runtime integrated correction.",
        ],
    }
    write_json(OUTPUT_DIR / "environment_x_hidden_correction_summary.json", summary)
    print(json.dumps(summary, ensure_ascii=False, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
