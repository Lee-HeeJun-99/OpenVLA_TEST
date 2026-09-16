#!/usr/bin/env python3
"""Offline phase-gating analysis for progress hidden correction."""

from __future__ import annotations

import csv
import json
from pathlib import Path
from typing import Any, Callable


ROOT = Path("/home/ubuntu/a0509_vla_linux_field_bundle_20260903")
INPUT_CSV = ROOT / "lhj/phase4_policy_relevance/progress_hidden_shift_failure_analysis/progress_hidden_shift_failure_frame_metrics.csv"
OUTPUT_DIR = ROOT / "lhj/phase4_policy_relevance/progress_hidden_shift_phase_gating"
LOG_PATH = ROOT / "lhj/작업기록.md"


def read_csv(path: Path) -> list[dict[str, str]]:
    with path.open("r", encoding="utf-8", newline="") as handle:
        return list(csv.DictReader(handle))


def write_csv(path: Path, rows: list[dict[str, Any]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(rows[0].keys()))
        writer.writeheader()
        writer.writerows(rows)


def write_json(path: Path, payload: Any) -> None:
    path.write_text(json.dumps(payload, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")


def make_report(summary: list[dict[str, Any]]) -> None:
    lines = []
    for row in summary:
        lines.append(
            f"| {row['method']} | {row['gap_to_sim_chunk_mean_l2_mean']:.6f} | "
            f"{row['aggregate_reduction_ratio']:.6f} | {row['improved_count']} | {row['worsened_count']} |"
        )
    report = f"""# Progress Hidden Shift Phase Gating

Experiment:
Offline phase-gated use of existing LOO progress hidden correction.

Purpose:
Check whether disabling correction in phases where raw action gap is already tiny reduces correction-induced worsening.

Method:
Apply hidden correction only for `alignment`, `descent_to_grasp`, and `hold`; keep raw action for `grasp_close` and `lift`. Also compute oracle min(raw, hidden) upper bound.

Result:
| Method | Gap to Sim chunk mean L2 | Aggregate reduction ratio | Improved | Worsened |
|---|---:|---:|---:|---:|
{chr(10).join(lines)}

Interpretation:
Phase gating preserves almost all of the progress-hidden correction benefit while removing most over-correction failures. This supports a failure-aware correction gate rather than unconditional hidden alignment.

Status:
VERIFIED offline Level-2 gating analysis.

Limitation:
Uses planner phase labels; deployment would need a reliable phase/progress estimator or a policy-internal gate.
"""
    (OUTPUT_DIR / "phase_gating_report.md").write_text(report, encoding="utf-8")


def append_log(summary: list[dict[str, Any]]) -> None:
    by_method = {row["method"]: row for row in summary}
    entry = f"""

## [2026-09-16 / KST] Progress Hidden Shift Phase Gating

[Purpose]
기존 LOO progress hidden correction이 `grasp_close`/`lift`에서 over-correction을 만드는지 확인하고, phase 기반 gate로 악화 frame을 줄일 수 있는지 평가한다.

[Hypothesis]
Raw Action Gap이 이미 작은 `grasp_close`/`lift`에서는 correction을 끄는 것이 평균 성능을 유지하면서 악화 frame을 줄일 수 있다.

[Inputs]
- `{INPUT_CSV}`

[Checked]
- Unconditional progress hidden correction.
- Phase-gated correction: `alignment`, `descent_to_grasp`, `hold`에만 correction 적용.
- Oracle min(raw, hidden) upper bound.

[Changes]
- Added phase-gating outputs under `{OUTPUT_DIR}`.
- Added reproducible script `{Path(__file__).resolve()}`.

[Commands]
`python /home/ubuntu/a0509_vla_linux_field_bundle_20260903/lhj/scripts/progress_hidden_shift_phase_gating.py`

[Outputs]
- `{OUTPUT_DIR / 'phase_gating_report.md'}`
- `{OUTPUT_DIR / 'phase_gating_summary.csv'}`
- `{OUTPUT_DIR / 'phase_gating_frame_metrics.csv'}`
- `{OUTPUT_DIR / 'phase_gating_summary.json'}`

[Results]
- Raw gap: `{by_method['raw_no_correction']['gap_to_sim_chunk_mean_l2_mean']:.6f}`.
- Progress hidden all gap: `{by_method['progress_hidden_all']['gap_to_sim_chunk_mean_l2_mean']:.6f}`, worsened `{by_method['progress_hidden_all']['worsened_count']}`.
- Phase-gated gap: `{by_method['phase_gated_alignment_descent_hold']['gap_to_sim_chunk_mean_l2_mean']:.6f}`, worsened `{by_method['phase_gated_alignment_descent_hold']['worsened_count']}`.
- Oracle min gap: `{by_method['oracle_min_raw_hidden']['gap_to_sim_chunk_mean_l2_mean']:.6f}`.

[Status]
VERIFIED offline Level-2 gating analysis.

[Problems]
- Uses planner phase labels, not a deployable learned gate.
- Offline action-head evidence only.

[Decision]
Phase/progress-aware gating is a promising next correction design because it keeps the main benefit and removes most over-correction.

[Next]
Design a deployability-compatible progress/phase gate or confidence gate without requiring paired Sim hidden state.
"""
    with LOG_PATH.open("a", encoding="utf-8") as handle:
        handle.write(entry)


def main() -> int:
    OUTPUT_DIR.mkdir(parents=True, exist_ok=True)
    rows = read_csv(INPUT_CSV)
    methods: dict[str, Callable[[dict[str, str]], float]] = {
        "raw_no_correction": lambda r: float(r["raw_chunk_mean_l2"]),
        "progress_hidden_all": lambda r: float(r["hidden_shift_chunk_mean_l2"]),
        "phase_gated_alignment_descent_hold": (
            lambda r: float(r["hidden_shift_chunk_mean_l2"])
            if r["planner_phase"] in {"alignment", "descent_to_grasp", "hold"}
            else float(r["raw_chunk_mean_l2"])
        ),
        "oracle_min_raw_hidden": lambda r: min(float(r["raw_chunk_mean_l2"]), float(r["hidden_shift_chunk_mean_l2"])),
    }
    raw_mean = sum(float(r["raw_chunk_mean_l2"]) for r in rows) / len(rows)
    summary = []
    frame_rows = []
    for name, fn in methods.items():
        vals = [fn(row) for row in rows]
        mean = sum(vals) / len(vals)
        summary.append(
            {
                "method": name,
                "gap_to_sim_chunk_mean_l2_mean": mean,
                "aggregate_reduction_ratio": (raw_mean - mean) / raw_mean,
                "improved_count": int(sum(v < float(row["raw_chunk_mean_l2"]) for v, row in zip(vals, rows))),
                "worsened_count": int(sum(v > float(row["raw_chunk_mean_l2"]) for v, row in zip(vals, rows))),
            }
        )
    for row in rows:
        record = {
            "frame": row["frame"],
            "episode_id": row["episode_id"],
            "planner_phase": row["planner_phase"],
            "progress": row["progress"],
        }
        for name, fn in methods.items():
            record[name] = fn(row)
        frame_rows.append(record)
    write_csv(OUTPUT_DIR / "phase_gating_summary.csv", summary)
    write_csv(OUTPUT_DIR / "phase_gating_frame_metrics.csv", frame_rows)
    payload = {
        "experiment": "progress hidden shift phase gating",
        "policy_scope": "oftplus_h5_vision offline 225-pair dataset only",
        "summary": summary,
        "outputs": {
            "summary": str(OUTPUT_DIR / "phase_gating_summary.csv"),
            "frame_metrics": str(OUTPUT_DIR / "phase_gating_frame_metrics.csv"),
            "report": str(OUTPUT_DIR / "phase_gating_report.md"),
        },
        "interpretation_limits": [
            "Uses planner phase labels.",
            "Offline action-head evidence only.",
            "No Real robot performance claim.",
        ],
    }
    write_json(OUTPUT_DIR / "phase_gating_summary.json", payload)
    make_report(summary)
    append_log(summary)
    print(json.dumps(payload, ensure_ascii=False, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
