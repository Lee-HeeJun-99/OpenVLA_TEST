#!/usr/bin/env python3
"""LOO correction robustness ablation for action_hidden_states.input."""

from __future__ import annotations

import csv
import json
import math
import sys
from pathlib import Path
from typing import Any

import matplotlib.pyplot as plt
import numpy as np


ROOT = Path("/home/ubuntu/a0509_vla_linux_field_bundle_20260903")
LHJ = ROOT / "lhj"
SCRIPT_DIR = LHJ / "scripts"
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

from action_hidden_token_sensitivity import (  # noqa: E402
    ACTION_DIM,
    ANALYSIS_ROOT,
    CHECKPOINT,
    CHUNK,
    DATASET_KEY,
    FEATURE,
    PHASE_CSV,
    action_chunk,
    chunk_metrics,
    episode_id_from_frame,
    load_action_head,
    load_npz_array,
    phase_map,
    predict_batch,
    progress_from_frame,
    read_json,
    records_by_frame,
    write_csv,
    write_json,
)


PHASE5 = LHJ / "phase5_policy_relevant_completion"
OUTPUT_DIR = PHASE5 / "01_correction_robustness"
LOG_PATH = LHJ / "작업기록.md"
SEED = 20260916
WINDOW = 11
PROGRESS_BIN = 4


def mean_shift(samples: list[dict[str, Any]]) -> np.ndarray:
    return np.mean([sample["delta"] for sample in samples], axis=0)


def nearest_progress_shift(train: list[dict[str, Any]], progress: int, phase: str | None = None) -> np.ndarray:
    if phase is not None:
        candidates = [s for s in train if s["planner_phase"] == phase]
        if not candidates:
            candidates = train
    else:
        candidates = train
    selected = [s for s in candidates if abs(int(s["progress"]) - progress) <= WINDOW // 2]
    if not selected:
        selected = sorted(candidates, key=lambda s: abs(int(s["progress"]) - progress))[: max(1, min(10, len(candidates)))]
    return mean_shift(selected)


def shuffled_progress_train(train: list[dict[str, Any]], rng: np.random.Generator) -> list[dict[str, Any]]:
    """Return train samples whose progress/phase metadata is preserved but deltas are permuted."""
    deltas = [sample["delta"] for sample in train]
    perm = rng.permutation(len(deltas))
    shuffled = []
    for sample, delta_index in zip(train, perm):
        clone = dict(sample)
        clone["delta"] = deltas[int(delta_index)]
        shuffled.append(clone)
    return shuffled


def phase_shift(train: list[dict[str, Any]], phase: str) -> np.ndarray:
    selected = [s for s in train if s["planner_phase"] == phase]
    if not selected:
        selected = train
    return mean_shift(selected)


def progress_bin_shift(train: list[dict[str, Any]], progress: int, phase: str) -> np.ndarray:
    bin_id = progress // PROGRESS_BIN
    selected = [
        s for s in train
        if s["planner_phase"] == phase and int(s["progress"]) // PROGRESS_BIN == bin_id
    ]
    if not selected:
        selected = [s for s in train if int(s["progress"]) // PROGRESS_BIN == bin_id]
    if not selected:
        return nearest_progress_shift(train, progress, phase)
    return mean_shift(selected)


def random_same_norm(shape: tuple[int, ...], norm: float, rng: np.random.Generator) -> np.ndarray:
    arr = rng.standard_normal(shape).astype(np.float64)
    arr_norm = float(np.linalg.norm(arr.reshape(-1)))
    if arr_norm <= 1e-18:
        return np.zeros(shape, dtype=np.float64)
    return arr * (norm / arr_norm)


def aggregate(rows: list[dict[str, Any]], group_keys: list[str]) -> list[dict[str, Any]]:
    grouped: dict[tuple[Any, ...], list[dict[str, Any]]] = {}
    for row in rows:
        grouped.setdefault(tuple(row[k] for k in group_keys), []).append(row)
    metrics = [
        "gap_to_sim_chunk_mean_l2",
        "gap_to_sim_chunk_max_l2",
        "gap_to_sim_first_l2",
        "gap_to_sim_first_translation_l2",
        "gap_to_sim_first_rotation_l2",
        "gap_to_sim_first_gripper_abs",
        "shift_norm",
        "repr_gap_reduction_ratio",
        "action_gap_reduction",
        "action_gap_reduction_ratio",
    ]
    out = []
    for key, items in sorted(grouped.items(), key=lambda kv: kv[0]):
        rec = {k: v for k, v in zip(group_keys, key)}
        rec["count"] = len(items)
        for metric in metrics:
            vals = np.asarray([float(item[metric]) for item in items], dtype=np.float64)
            rec[f"{metric}_mean"] = float(vals.mean())
            rec[f"{metric}_std"] = float(vals.std())
            rec[f"{metric}_p50"] = float(np.percentile(vals, 50))
            rec[f"{metric}_p90"] = float(np.percentile(vals, 90))
        rec["improved_count"] = int(sum(float(item["action_gap_reduction"]) > 0 for item in items))
        rec["worsened_count"] = int(sum(float(item["action_gap_reduction"]) < 0 for item in items))
        out.append(rec)
    return out


def make_plots(overall: list[dict[str, Any]]) -> None:
    rows = {row["method"]: row for row in overall}
    order = [
        "no_correction",
        "global_mean",
        "progress",
        "phase",
        "progress_phase",
        "random_matched_norm",
        "shuffled_progress",
        "wrong_direction",
    ]
    methods = [m for m in order if m in rows]
    gaps = [float(rows[m]["gap_to_sim_chunk_mean_l2_mean"]) for m in methods]
    raw = float(rows["no_correction"]["gap_to_sim_chunk_mean_l2_mean"])
    action_red = [(raw - gap) / raw for gap in gaps]
    repr_red = [float(rows[m]["repr_gap_reduction_ratio_mean"]) for m in methods]
    x = np.arange(len(methods))

    plt.figure(figsize=(10.5, 4.8))
    plt.bar(x - 0.18, action_red, width=0.36, label="action reduction")
    plt.bar(x + 0.18, repr_red, width=0.36, label="representation reduction")
    plt.xticks(x, methods, rotation=25, ha="right")
    plt.ylabel("reduction ratio")
    plt.title("LOO Correction Robustness Ablation")
    plt.legend()
    plt.tight_layout()
    plt.savefig(OUTPUT_DIR / "loo_correction_ablation_action_vs_repr.png", dpi=160)
    plt.close()

    plt.figure(figsize=(10.5, 4.8))
    plt.bar(methods, gaps)
    plt.xticks(rotation=25, ha="right")
    plt.ylabel("gap to Sim chunk mean L2")
    plt.title("LOO Correction Robustness: Remaining Action Gap")
    plt.tight_layout()
    plt.savefig(OUTPUT_DIR / "loo_correction_ablation_remaining_gap.png", dpi=160)
    plt.close()


def make_report(overall: list[dict[str, Any]], by_phase: list[dict[str, Any]]) -> None:
    rows = {row["method"]: row for row in overall}
    order = [
        "no_correction",
        "global_mean",
        "progress",
        "phase",
        "progress_phase",
        "random_matched_norm",
        "shuffled_progress",
        "wrong_direction",
    ]
    raw = float(rows["no_correction"]["gap_to_sim_chunk_mean_l2_mean"])
    lines = []
    for method in order:
        row = rows[method]
        gap = float(row["gap_to_sim_chunk_mean_l2_mean"])
        lines.append(
            f"| {method} | {gap:.6f} | {(raw-gap)/raw:.6f} | "
            f"{float(row['repr_gap_reduction_ratio_mean']):.6f} | "
            f"{row['improved_count']} | {row['worsened_count']} |"
        )
    phase_lines = []
    for row in by_phase:
        if row["method"] not in {"progress", "phase", "progress_phase"}:
            continue
        phase_lines.append(
            f"| {row['planner_phase']} | {row['method']} | {row['gap_to_sim_chunk_mean_l2_mean']:.6f} | "
            f"{row['improved_count']} | {row['worsened_count']} |"
        )
    report = f"""# LOO Correction Robustness Ablation

[Purpose]
Test whether progress-conditioned hidden correction is genuinely stronger than simple/global/random/shuffled/wrong-direction controls under leave-one-episode-out estimation.

[Hypothesis]
Progress-conditioned correction should reduce held-out Action Gap more than global mean, random matched-norm, shuffled progress, or wrong-direction controls.

[Inputs]
- 225 verified Real/Sim pairs.
- Feature: `action_hidden_states.input`.
- Policy: `oftplus_h5_vision`, checkpoint step 28560.

[Checked]
- C0 no correction
- C1 global mean hidden shift
- C2 progress-conditioned hidden shift
- C3 phase-conditioned shift
- C4 progress+phase conditioned shift
- C5 random matched-norm shift
- C6 shuffled progress shift
- C7 wrong-direction/sign-flipped shift

[Results]
| Method | Gap to Sim chunk mean L2 | Aggregate action reduction | Repr reduction ratio | Improved | Worsened |
|---|---:|---:|---:|---:|---:|
{chr(10).join(lines)}

Phase summary for structured methods:
| Phase | Method | Gap | Improved | Worsened |
|---|---|---:|---:|---:|
{chr(10).join(phase_lines)}

[Status]
VERIFIED offline Level-2 correction robustness analysis.

[Problems]
- Only 5 episode folds.
- LOO episode split is not unseen-layout generalization.
- Correction still uses train Real/Sim paired calibration data.

[Decision]
Use the best robust method as the reference for Phase5 gating and deployability comparisons.

[Next]
Proceed to held-out gate generalization.
"""
    (OUTPUT_DIR / "loo_correction_ablation_report.md").write_text(report, encoding="utf-8")


def append_log(summary: dict[str, Any]) -> None:
    key = summary["overall_key_results"]
    entry = f"""

## [2026-09-16 / KST] Phase5 STEP 1 — LOO Correction Robustness

[Purpose]
기존 progress-conditioned hidden correction이 global/random/shuffled/wrong-direction control보다 실제로 강한지 LOO 기준으로 검증한다.

[Hypothesis]
Progress-conditioned correction은 held-out episode에서 simple/global/random/shuffled/wrong-direction보다 Action Gap을 더 줄여야 한다.

[Inputs]
- 225 verified Real/Sim pairs.
- Policy: `oftplus_h5_vision`, checkpoint step 28560.
- Feature: `action_hidden_states.input`.

[Checked]
- no correction, global mean, progress, phase, progress+phase, random matched-norm, shuffled progress, wrong-direction.
- Held-out episode leakage 방지: correction parameter는 train 4 episodes에서만 추정.

[Changes]
- Added Phase5 STEP 1 outputs under `{OUTPUT_DIR}`.

[Commands]
`/home/ubuntu/a0509_vla_linux_field_bundle_20260903/environment/a6000_ubuntu22_py310/bin/python /home/ubuntu/a0509_vla_linux_field_bundle_20260903/lhj/phase5_policy_relevant_completion/scripts/loo_correction_ablation.py`

[Outputs]
- `{OUTPUT_DIR / 'loo_correction_ablation_report.md'}`
- `{OUTPUT_DIR / 'loo_correction_ablation_frame_metrics.csv'}`
- `{OUTPUT_DIR / 'loo_correction_ablation_summary_overall.csv'}`
- `{OUTPUT_DIR / 'loo_correction_ablation_summary_by_phase.csv'}`
- `{OUTPUT_DIR / 'loo_correction_ablation_summary.json'}`

[Results]
- no correction gap: `{key['no_correction']['gap_to_sim_chunk_mean_l2_mean']:.6f}`.
- global mean gap: `{key['global_mean']['gap_to_sim_chunk_mean_l2_mean']:.6f}`.
- progress gap: `{key['progress']['gap_to_sim_chunk_mean_l2_mean']:.6f}`.
- phase gap: `{key['phase']['gap_to_sim_chunk_mean_l2_mean']:.6f}`.
- progress+phase gap: `{key['progress_phase']['gap_to_sim_chunk_mean_l2_mean']:.6f}`.
- random matched-norm gap: `{key['random_matched_norm']['gap_to_sim_chunk_mean_l2_mean']:.6f}`.
- shuffled progress gap: `{key['shuffled_progress']['gap_to_sim_chunk_mean_l2_mean']:.6f}`.
- wrong-direction gap: `{key['wrong_direction']['gap_to_sim_chunk_mean_l2_mean']:.6f}`.

[Status]
VERIFIED offline Level-2 correction robustness analysis.

[Problems]
- 5 episode LOO only, not unseen-layout generalization.
- Still calibration-pair based.

[Decision]
Use the strongest robust correction as reference for gate generalization and deployability comparisons.

[Next]
Run LOO gate generalization with threshold selected only on train folds.
"""
    with LOG_PATH.open("a", encoding="utf-8") as handle:
        handle.write(entry)


def main() -> int:
    OUTPUT_DIR.mkdir(parents=True, exist_ok=True)
    rng = np.random.default_rng(SEED)
    real_records = records_by_frame(ANALYSIS_ROOT / "features/real/feature_manifest.json")
    sim_records = records_by_frame(ANALYSIS_ROOT / "features/sim/feature_manifest.json")
    frames = sorted(set(real_records) & set(sim_records))
    phases = phase_map(PHASE_CSV)
    action_stats = read_json(CHECKPOINT / "dataset_statistics.json")[DATASET_KEY]["action"]
    action_head, device, action_head_checkpoint = load_action_head(CHECKPOINT)

    samples = []
    for frame in frames:
        real_hidden = load_npz_array(real_records[frame], FEATURE).astype(np.float64)
        sim_hidden = load_npz_array(sim_records[frame], FEATURE).astype(np.float64)
        samples.append(
            {
                "frame": frame,
                "episode_id": episode_id_from_frame(frame),
                "progress": progress_from_frame(frame),
                "planner_phase": phases.get(frame, "UNKNOWN"),
                "real_hidden": real_hidden,
                "sim_hidden": sim_hidden,
                "delta": sim_hidden - real_hidden,
                "real_action": action_chunk(real_records[frame]),
                "sim_action": action_chunk(sim_records[frame]),
            }
        )
    episodes = sorted({sample["episode_id"] for sample in samples})
    all_rows: list[dict[str, Any]] = []
    methods = [
        "no_correction",
        "global_mean",
        "progress",
        "phase",
        "progress_phase",
        "random_matched_norm",
        "shuffled_progress",
        "wrong_direction",
    ]

    for heldout in episodes:
        train = [s for s in samples if s["episode_id"] != heldout]
        test = [s for s in samples if s["episode_id"] == heldout]
        global_shift = mean_shift(train)
        shuffled_train = shuffled_progress_train(train, rng)

        batch = []
        meta = []
        for sample in test:
            progress_shift = nearest_progress_shift(train, int(sample["progress"]))
            shuffled_shift = nearest_progress_shift(shuffled_train, int(sample["progress"]))
            shifts = {
                "no_correction": np.zeros_like(sample["delta"]),
                "global_mean": global_shift,
                "progress": progress_shift,
                "phase": phase_shift(train, sample["planner_phase"]),
                "progress_phase": progress_bin_shift(train, int(sample["progress"]), sample["planner_phase"]),
                "random_matched_norm": random_same_norm(sample["delta"].shape, float(np.linalg.norm(progress_shift.reshape(-1))), rng),
                "shuffled_progress": shuffled_shift,
                "wrong_direction": -progress_shift,
            }
            for method in methods:
                shift = shifts[method].astype(np.float64)
                batch.append((sample["real_hidden"] + shift).astype(np.float32))
                meta.append((sample, method, shift))
        pred_actions = predict_batch(action_head, np.concatenate(batch, axis=0), action_stats, device)
        for (sample, method, shift), pred_action in zip(meta, pred_actions):
            raw_gap = chunk_metrics(sample["real_action"], sample["sim_action"], "raw_gap")
            gap = chunk_metrics(pred_action, sample["sim_action"], "gap_to_sim")
            delta_norm = float(np.linalg.norm(sample["delta"].reshape(-1)))
            hidden_after_gap = float(np.linalg.norm((sample["sim_hidden"] - (sample["real_hidden"] + shift)).reshape(-1)))
            raw_chunk = float(raw_gap["raw_gap_chunk_mean_l2"])
            gap_chunk = float(gap["gap_to_sim_chunk_mean_l2"])
            all_rows.append(
                {
                    "frame": sample["frame"],
                    "episode_id": sample["episode_id"],
                    "heldout_episode": heldout,
                    "progress": sample["progress"],
                    "planner_phase": sample["planner_phase"],
                    "method": method,
                    "delta_hidden_norm": delta_norm,
                    "shift_norm": float(np.linalg.norm(shift.reshape(-1))),
                    "hidden_after_gap": hidden_after_gap,
                    "repr_gap_reduction_ratio": (delta_norm - hidden_after_gap) / delta_norm if delta_norm > 1e-18 else 0.0,
                    **raw_gap,
                    **gap,
                    "action_gap_reduction": raw_chunk - gap_chunk,
                    "action_gap_reduction_ratio": (raw_chunk - gap_chunk) / raw_chunk if raw_chunk > 1e-18 else 0.0,
                }
            )
        print(f"processed heldout {heldout}", flush=True)

    overall = aggregate(all_rows, ["method"])
    by_phase = aggregate(all_rows, ["planner_phase", "method"])
    by_episode = aggregate(all_rows, ["heldout_episode", "method"])
    write_csv(OUTPUT_DIR / "loo_correction_ablation_frame_metrics.csv", all_rows)
    write_csv(OUTPUT_DIR / "loo_correction_ablation_summary_overall.csv", overall)
    write_csv(OUTPUT_DIR / "loo_correction_ablation_summary_by_phase.csv", by_phase)
    write_csv(OUTPUT_DIR / "loo_correction_ablation_summary_by_episode.csv", by_episode)
    make_plots(overall)
    make_report(overall, by_phase)
    key = {
        row["method"]: {
            "gap_to_sim_chunk_mean_l2_mean": float(row["gap_to_sim_chunk_mean_l2_mean"]),
            "repr_gap_reduction_ratio_mean": float(row["repr_gap_reduction_ratio_mean"]),
            "improved_count": int(row["improved_count"]),
            "worsened_count": int(row["worsened_count"]),
        }
        for row in overall
    }
    summary = {
        "experiment": "Phase5 STEP 1 LOO correction robustness ablation",
        "policy_scope": "oftplus_h5_vision offline 225-pair dataset only",
        "frames": len(samples),
        "feature": FEATURE,
        "window": WINDOW,
        "progress_bin": PROGRESS_BIN,
        "checkpoint": str(CHECKPOINT),
        "action_head_checkpoint": action_head_checkpoint,
        "device": str(device),
        "overall_key_results": key,
        "outputs": {
            "frame_metrics": str(OUTPUT_DIR / "loo_correction_ablation_frame_metrics.csv"),
            "overall": str(OUTPUT_DIR / "loo_correction_ablation_summary_overall.csv"),
            "by_phase": str(OUTPUT_DIR / "loo_correction_ablation_summary_by_phase.csv"),
            "by_episode": str(OUTPUT_DIR / "loo_correction_ablation_summary_by_episode.csv"),
            "report": str(OUTPUT_DIR / "loo_correction_ablation_report.md"),
        },
        "interpretation_limits": [
            "Five episode LOO only.",
            "Correction uses train Real/Sim calibration pairs.",
            "Offline action-head evidence only.",
        ],
    }
    write_json(OUTPUT_DIR / "loo_correction_ablation_summary.json", summary)
    append_log(summary)
    print(json.dumps(summary, ensure_ascii=False, indent=2)[:10000])
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
