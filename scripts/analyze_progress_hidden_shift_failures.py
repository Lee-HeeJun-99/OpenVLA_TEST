#!/usr/bin/env python3
"""Failure analysis for existing LOO progress hidden correction."""

from __future__ import annotations

import csv
import json
from pathlib import Path
from typing import Any

import matplotlib.pyplot as plt
import numpy as np


ROOT = Path("/home/ubuntu/a0509_vla_linux_field_bundle_20260903")
INPUT_CSV = ROOT / "lhj/phase1_action_gap/policy_relevant_progress_shift/policy_relevant_progress_shift_frame_metrics.csv"
PHASE_CSV = ROOT / "outputs/token_distribution_analysis/5_episodes/episode_phase_summary/frame_metrics_enriched.csv"
OUTPUT_DIR = ROOT / "lhj/phase4_policy_relevance/progress_hidden_shift_failure_analysis"
LOG_PATH = ROOT / "lhj/작업기록.md"


def read_csv(path: Path) -> list[dict[str, str]]:
    with path.open("r", encoding="utf-8", newline="") as handle:
        return list(csv.DictReader(handle))


def write_csv(path: Path, rows: list[dict[str, Any]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    if not rows:
        path.write_text("", encoding="utf-8")
        return
    with path.open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(rows[0].keys()))
        writer.writeheader()
        writer.writerows(rows)


def write_json(path: Path, payload: Any) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")


def phase_map() -> dict[str, str]:
    return {row["pair_id"]: row["planner_phase"] for row in read_csv(PHASE_CSV)}


def stats(values: list[float]) -> dict[str, float | int]:
    arr = np.asarray(values, dtype=np.float64)
    return {
        "count": int(arr.size),
        "mean": float(arr.mean()),
        "std": float(arr.std()),
        "p50": float(np.percentile(arr, 50)),
        "p90": float(np.percentile(arr, 90)),
        "min": float(arr.min()),
        "max": float(arr.max()),
    }


def aggregate(rows: list[dict[str, Any]], group_key: str) -> list[dict[str, Any]]:
    grouped: dict[str, list[dict[str, Any]]] = {}
    for row in rows:
        grouped.setdefault(str(row[group_key]), []).append(row)
    metrics = [
        "raw_chunk_mean_l2",
        "hidden_shift_chunk_mean_l2",
        "chunk_gap_reduction",
        "chunk_gap_reduction_ratio",
        "raw_first_translation_l2",
        "hidden_shift_first_translation_l2",
        "raw_first_rotation_l2",
        "hidden_shift_first_rotation_l2",
        "raw_first_gripper_abs",
        "hidden_shift_first_gripper_abs",
    ]
    out = []
    for key, items in sorted(grouped.items()):
        record: dict[str, Any] = {group_key: key, "count": len(items)}
        for metric in metrics:
            vals = [float(item[metric]) for item in items]
            s = stats(vals)
            record[f"{metric}_mean"] = s["mean"]
            record[f"{metric}_p50"] = s["p50"]
            record[f"{metric}_p90"] = s["p90"]
        record["improved_count"] = int(sum(float(item["chunk_gap_reduction"]) > 0 for item in items))
        record["worsened_count"] = int(sum(float(item["chunk_gap_reduction"]) < 0 for item in items))
        out.append(record)
    return out


def make_plots(rows: list[dict[str, Any]], by_phase: list[dict[str, Any]]) -> None:
    progress = [float(row["progress"]) for row in rows]
    raw = [float(row["raw_chunk_mean_l2"]) for row in rows]
    corrected = [float(row["hidden_shift_chunk_mean_l2"]) for row in rows]
    reduction = [float(row["chunk_gap_reduction"]) for row in rows]

    plt.figure(figsize=(9.2, 4.8))
    plt.scatter(progress, raw, s=12, alpha=0.55, label="raw")
    plt.scatter(progress, corrected, s=12, alpha=0.55, label="corrected")
    plt.xlabel("progress/frame index within episode")
    plt.ylabel("chunk mean action gap")
    plt.title("Progress Hidden Shift: Raw vs Corrected Gap")
    plt.legend()
    plt.tight_layout()
    plt.savefig(OUTPUT_DIR / "raw_vs_corrected_gap_by_progress.png", dpi=160)
    plt.close()

    plt.figure(figsize=(9.2, 4.8))
    plt.axhline(0.0, color="0.7", linewidth=1)
    plt.scatter(progress, reduction, s=12, alpha=0.65)
    plt.xlabel("progress/frame index within episode")
    plt.ylabel("raw gap - corrected gap")
    plt.title("Progress Hidden Shift: Improvement by Progress")
    plt.tight_layout()
    plt.savefig(OUTPUT_DIR / "gap_reduction_by_progress.png", dpi=160)
    plt.close()

    phases = [row["planner_phase"] for row in by_phase]
    values = [float(row["chunk_gap_reduction_mean"]) for row in by_phase]
    worsened = [int(row["worsened_count"]) for row in by_phase]
    plt.figure(figsize=(8.2, 4.8))
    x = np.arange(len(phases))
    plt.bar(x, values)
    for xi, w in zip(x, worsened):
        plt.text(xi, values[xi], f"w{w}", ha="center", va="bottom", fontsize=8)
    plt.xticks(x, phases, rotation=25, ha="right")
    plt.ylabel("mean chunk gap reduction")
    plt.title("Progress Hidden Shift: Improvement by Phase")
    plt.tight_layout()
    plt.savefig(OUTPUT_DIR / "gap_reduction_by_phase.png", dpi=160)
    plt.close()


def make_report(summary: dict[str, Any], by_phase: list[dict[str, Any]], top_worse: list[dict[str, Any]]) -> None:
    phase_lines = []
    for row in by_phase:
        phase_lines.append(
            f"| {row['planner_phase']} | {row['count']} | {row['chunk_gap_reduction_mean']:.6f} | "
            f"{row['hidden_shift_chunk_mean_l2_mean']:.6f} | {row['improved_count']} | {row['worsened_count']} |"
        )
    worse_lines = []
    for row in top_worse[:15]:
        worse_lines.append(
            f"| {row['frame']} | {row['planner_phase']} | {row['raw_chunk_mean_l2']:.6f} | "
            f"{row['hidden_shift_chunk_mean_l2']:.6f} | {row['chunk_gap_reduction']:.6f} | "
            f"{row['hidden_shift_first_gripper_abs']:.6f} |"
        )
    report = f"""# Progress Hidden Shift Failure Analysis

Experiment:
Failure analysis of existing leave-one-episode-out progress-conditioned hidden correction.

Purpose:
Identify where the strongest existing offline correction still fails or worsens Action Gap.

Hypothesis:
Remaining/worsened frames will be structured by phase/progress/action dimension rather than uniformly random.

Input:
- `{INPUT_CSV}`
- Phase labels from `{PHASE_CSV}`

Method:
Compute per-frame `raw_chunk_mean_l2 - hidden_shift_chunk_mean_l2`, then summarize by phase, progress, episode, and action dimensions.

Result:
- Raw chunk mean gap: `{summary['raw_chunk_mean_l2_mean']:.6f}`
- Corrected chunk mean gap: `{summary['hidden_shift_chunk_mean_l2_mean']:.6f}`
- Mean reduction: `{summary['chunk_gap_reduction_mean']:.6f}`
- Aggregate reduction ratio: `{summary['aggregate_reduction_ratio']:.6f}`
- Improved frames: `{summary['improved_count']}`
- Worsened frames: `{summary['worsened_count']}`

Phase summary:
| Phase | Frames | Mean reduction | Corrected gap | Improved | Worsened |
|---|---:|---:|---:|---:|---:|
{chr(10).join(phase_lines)}

Largest worsened frames:
| Frame | Phase | Raw gap | Corrected gap | Reduction | Corrected gripper gap |
|---|---|---:|---:|---:|---:|
{chr(10).join(worse_lines)}

Interpretation:
The progress-conditioned hidden correction is strong on average, but remaining failure frames should guide the next correction rather than blindly reducing representation distance.

Status:
VERIFIED offline Level-2 failure analysis.

Limitation:
This diagnoses action-head outputs only. It does not prove Real robot performance or environment causality.

Next decision:
Use phase/progress/action-dimension failure structure to design a targeted correction or failure-aware gating rule.
"""
    (OUTPUT_DIR / "progress_hidden_shift_failure_report.md").write_text(report, encoding="utf-8")


def append_log(summary: dict[str, Any]) -> None:
    entry = f"""

## [2026-09-16 / KST] Progress Hidden Shift Failure Analysis

[Purpose]
기존 LOO progress hidden correction이 평균적으로 강하게 동작한 뒤에도 어떤 frame/phase/action dimension에서 실패하거나 악화되는지 확인한다.

[Hypothesis]
남은 실패 frame은 phase/progress/action dimension에 따라 구조적으로 나타날 수 있다.

[Inputs]
- `{INPUT_CSV}`
- `{PHASE_CSV}`

[Checked]
- Per-frame raw vs corrected chunk mean Action Gap.
- Phase-wise improvement/worsening.
- Largest worsened frames.
- Translation/rotation/gripper residual gap.

[Changes]
- Added failure-analysis outputs under `{OUTPUT_DIR}`.

[Commands]
`python /home/ubuntu/a0509_vla_linux_field_bundle_20260903/lhj/scripts/analyze_progress_hidden_shift_failures.py`

[Outputs]
- `{OUTPUT_DIR / 'progress_hidden_shift_failure_report.md'}`
- `{OUTPUT_DIR / 'progress_hidden_shift_failure_frame_metrics.csv'}`
- `{OUTPUT_DIR / 'progress_hidden_shift_failure_summary_by_phase.csv'}`
- `{OUTPUT_DIR / 'progress_hidden_shift_failure_summary.json'}`
- `{OUTPUT_DIR / 'raw_vs_corrected_gap_by_progress.png'}`
- `{OUTPUT_DIR / 'gap_reduction_by_progress.png'}`
- `{OUTPUT_DIR / 'gap_reduction_by_phase.png'}`

[Results]
- Raw chunk mean gap: `{summary['raw_chunk_mean_l2_mean']:.6f}`.
- Corrected chunk mean gap: `{summary['hidden_shift_chunk_mean_l2_mean']:.6f}`.
- Aggregate reduction ratio: `{summary['aggregate_reduction_ratio']:.6f}`.
- Improved frames: `{summary['improved_count']}`.
- Worsened frames: `{summary['worsened_count']}`.

[Status]
VERIFIED offline Level-2 failure analysis.

[Problems]
- Offline action-head failure analysis only.
- Does not prove Real robot performance or environment cause.

[Decision]
Use failure structure to guide targeted correction/gating rather than repeating broad distribution alignment.

[Next]
Design targeted correction around the phases/progress/action dimensions where residual gap remains.
"""
    with LOG_PATH.open("a", encoding="utf-8") as handle:
        handle.write(entry)


def main() -> int:
    OUTPUT_DIR.mkdir(parents=True, exist_ok=True)
    phases = phase_map()
    rows: list[dict[str, Any]] = []
    for raw in read_csv(INPUT_CSV):
        row: dict[str, Any] = dict(raw)
        row["planner_phase"] = phases.get(row["frame"], "UNKNOWN")
        for key, value in list(row.items()):
            if key not in {"frame", "episode_id", "planner_phase"}:
                try:
                    row[key] = float(value)
                except ValueError:
                    pass
        row["chunk_gap_reduction"] = float(row["raw_chunk_mean_l2"]) - float(row["hidden_shift_chunk_mean_l2"])
        row["chunk_gap_reduction_ratio"] = (
            row["chunk_gap_reduction"] / float(row["raw_chunk_mean_l2"])
            if abs(float(row["raw_chunk_mean_l2"])) > 1e-18
            else 0.0
        )
        row["first_gripper_reduction"] = float(row["raw_first_gripper_abs"]) - float(row["hidden_shift_first_gripper_abs"])
        rows.append(row)

    by_phase = aggregate(rows, "planner_phase")
    by_episode = aggregate(rows, "episode_id")
    top_worse = sorted(rows, key=lambda r: float(r["chunk_gap_reduction"]))[:25]
    raw_vals = [float(row["raw_chunk_mean_l2"]) for row in rows]
    corrected_vals = [float(row["hidden_shift_chunk_mean_l2"]) for row in rows]
    reductions = [float(row["chunk_gap_reduction"]) for row in rows]
    summary = {
        "experiment": "progress hidden shift failure analysis",
        "policy_scope": "oftplus_h5_vision offline 225-pair dataset only",
        "frames": len(rows),
        "raw_chunk_mean_l2_mean": float(np.mean(raw_vals)),
        "hidden_shift_chunk_mean_l2_mean": float(np.mean(corrected_vals)),
        "chunk_gap_reduction_mean": float(np.mean(reductions)),
        "aggregate_reduction_ratio": float((np.mean(raw_vals) - np.mean(corrected_vals)) / np.mean(raw_vals)),
        "improved_count": int(sum(v > 0 for v in reductions)),
        "worsened_count": int(sum(v < 0 for v in reductions)),
        "outputs": {
            "frame_metrics": str(OUTPUT_DIR / "progress_hidden_shift_failure_frame_metrics.csv"),
            "by_phase": str(OUTPUT_DIR / "progress_hidden_shift_failure_summary_by_phase.csv"),
            "by_episode": str(OUTPUT_DIR / "progress_hidden_shift_failure_summary_by_episode.csv"),
            "report": str(OUTPUT_DIR / "progress_hidden_shift_failure_report.md"),
        },
        "interpretation_limits": [
            "Offline action-head failure analysis only.",
            "No Real robot performance claim.",
            "No environment causality claim.",
        ],
    }
    write_csv(OUTPUT_DIR / "progress_hidden_shift_failure_frame_metrics.csv", rows)
    write_csv(OUTPUT_DIR / "progress_hidden_shift_failure_summary_by_phase.csv", by_phase)
    write_csv(OUTPUT_DIR / "progress_hidden_shift_failure_summary_by_episode.csv", by_episode)
    write_csv(OUTPUT_DIR / "progress_hidden_shift_largest_worsened_frames.csv", top_worse)
    write_json(OUTPUT_DIR / "progress_hidden_shift_failure_summary.json", summary)
    make_plots(rows, by_phase)
    make_report(summary, by_phase, top_worse)
    append_log(summary)
    print(json.dumps(summary, ensure_ascii=False, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
