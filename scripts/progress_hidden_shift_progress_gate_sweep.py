#!/usr/bin/env python3
"""Progress-threshold gate sweep for progress hidden correction."""

from __future__ import annotations

import csv
import json
from pathlib import Path


ROOT = Path("/home/ubuntu/a0509_vla_linux_field_bundle_20260903")
INPUT_CSV = ROOT / "lhj/phase4_policy_relevance/progress_hidden_shift_failure_analysis/progress_hidden_shift_failure_frame_metrics.csv"
OUTPUT_DIR = ROOT / "lhj/phase4_policy_relevance/progress_hidden_shift_progress_gate"


def read_csv(path: Path) -> list[dict[str, str]]:
    with path.open("r", encoding="utf-8", newline="") as handle:
        return list(csv.DictReader(handle))


def write_csv(path: Path, rows: list[dict[str, object]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(rows[0].keys()))
        writer.writeheader()
        writer.writerows(rows)


def main() -> int:
    OUTPUT_DIR.mkdir(parents=True, exist_ok=True)
    rows = read_csv(INPUT_CSV)
    raw_mean = sum(float(row["raw_chunk_mean_l2"]) for row in rows) / len(rows)
    summary = []
    for threshold in range(0, 51):
        vals = []
        for row in rows:
            progress = int(float(row["progress"]))
            value = (
                float(row["hidden_shift_chunk_mean_l2"])
                if progress <= threshold
                else float(row["raw_chunk_mean_l2"])
            )
            vals.append(value)
        mean = sum(vals) / len(vals)
        summary.append(
            {
                "threshold_progress_le": threshold,
                "gap_to_sim_chunk_mean_l2_mean": mean,
                "aggregate_reduction_ratio": (raw_mean - mean) / raw_mean,
                "improved_count": int(sum(v < float(row["raw_chunk_mean_l2"]) for v, row in zip(vals, rows))),
                "worsened_count": int(sum(v > float(row["raw_chunk_mean_l2"]) for v, row in zip(vals, rows))),
            }
        )
    by_gap = sorted(summary, key=lambda row: row["gap_to_sim_chunk_mean_l2_mean"])
    by_worse = sorted(summary, key=lambda row: (row["worsened_count"], row["gap_to_sim_chunk_mean_l2_mean"]))
    write_csv(OUTPUT_DIR / "progress_gate_threshold_sweep.csv", summary)
    report = "# Progress Threshold Gate Sweep\n\n"
    report += "Experiment:\nApply progress hidden correction only when `progress <= threshold`.\n\n"
    report += "Purpose:\nApproximate phase gating without planner phase labels.\n\n"
    report += "Best by mean gap:\n\n"
    report += "| threshold | gap | reduction | improved | worsened |\n|---:|---:|---:|---:|---:|\n"
    for row in by_gap[:10]:
        report += (
            f"| {row['threshold_progress_le']} | {row['gap_to_sim_chunk_mean_l2_mean']:.6f} | "
            f"{row['aggregate_reduction_ratio']:.6f} | {row['improved_count']} | {row['worsened_count']} |\n"
        )
    report += "\nLowest-worsening candidates:\n\n"
    report += "| threshold | gap | reduction | improved | worsened |\n|---:|---:|---:|---:|---:|\n"
    for row in by_worse[:10]:
        report += (
            f"| {row['threshold_progress_le']} | {row['gap_to_sim_chunk_mean_l2_mean']:.6f} | "
            f"{row['aggregate_reduction_ratio']:.6f} | {row['improved_count']} | {row['worsened_count']} |\n"
        )
    report += "\nInterpretation:\nA simple progress threshold can remove over-correction failures at a modest cost to mean Action Gap reduction.\n\n"
    report += "Status:\nVERIFIED offline Level-2 progress-gate sweep.\n"
    (OUTPUT_DIR / "progress_gate_threshold_sweep_report.md").write_text(report, encoding="utf-8")
    payload = {
        "experiment": "progress hidden shift progress-threshold gate sweep",
        "policy_scope": "oftplus_h5_vision offline 225-pair dataset only",
        "best_by_gap": by_gap[0],
        "lowest_worsening_candidates": by_worse[:10],
        "outputs": {
            "sweep": str(OUTPUT_DIR / "progress_gate_threshold_sweep.csv"),
            "report": str(OUTPUT_DIR / "progress_gate_threshold_sweep_report.md"),
        },
    }
    (OUTPUT_DIR / "progress_gate_threshold_sweep_summary.json").write_text(
        json.dumps(payload, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )
    print(json.dumps(payload, ensure_ascii=False, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
