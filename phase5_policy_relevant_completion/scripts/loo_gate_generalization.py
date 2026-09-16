#!/usr/bin/env python3
"""LOO gate generalization for progress hidden correction."""

from __future__ import annotations

import csv
import json
import sys
from pathlib import Path
from typing import Any

import matplotlib.pyplot as plt
import numpy as np


ROOT = Path("/home/ubuntu/a0509_vla_linux_field_bundle_20260903")
LHJ = ROOT / "lhj"
PHASE5 = LHJ / "phase5_policy_relevant_completion"
INPUT_CSV = PHASE5 / "01_correction_robustness/loo_correction_ablation_frame_metrics.csv"
OUTPUT_DIR = PHASE5 / "02_gate_generalization"
LOG_PATH = LHJ / "작업기록.md"
LAMBDA = 0.10


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


def aggregate(rows: list[dict[str, Any]], group_keys: list[str]) -> list[dict[str, Any]]:
    grouped: dict[tuple[Any, ...], list[dict[str, Any]]] = {}
    for row in rows:
        grouped.setdefault(tuple(row[k] for k in group_keys), []).append(row)
    out = []
    for key, items in sorted(grouped.items(), key=lambda kv: kv[0]):
        vals = np.asarray([float(item["selected_gap"]) for item in items], dtype=np.float64)
        raw = np.asarray([float(item["raw_gap"]) for item in items], dtype=np.float64)
        rec = {k: v for k, v in zip(group_keys, key)}
        rec["count"] = len(items)
        rec["gap_to_sim_chunk_mean_l2_mean"] = float(vals.mean())
        rec["gap_to_sim_chunk_mean_l2_p50"] = float(np.percentile(vals, 50))
        rec["gap_to_sim_chunk_mean_l2_p90"] = float(np.percentile(vals, 90))
        rec["aggregate_reduction_ratio"] = float((raw.mean() - vals.mean()) / raw.mean())
        rec["improved_count"] = int(sum(vals < raw))
        rec["worsened_count"] = int(sum(vals > raw))
        out.append(rec)
    return out


def method_rows(rows: list[dict[str, str]], method: str) -> list[dict[str, str]]:
    return [row for row in rows if row["method"] == method]


def base_by_frame(rows: list[dict[str, str]]) -> dict[str, dict[str, Any]]:
    # Use no_correction rows for raw metadata, progress rows for corrected gap.
    raw = {row["frame"]: row for row in method_rows(rows, "no_correction")}
    progress = {row["frame"]: row for row in method_rows(rows, "progress")}
    merged = {}
    for frame, row in raw.items():
        pr = progress[frame]
        merged[frame] = {
            "frame": frame,
            "episode_id": row["episode_id"],
            "planner_phase": row["planner_phase"],
            "progress": int(float(row["progress"])),
            "raw_gap": float(row["gap_to_sim_chunk_mean_l2"]),
            "progress_gap": float(pr["gap_to_sim_chunk_mean_l2"]),
        }
    return merged


def evaluate_threshold(rows: list[dict[str, Any]], threshold: int) -> tuple[float, int, int]:
    vals = [row["progress_gap"] if row["progress"] <= threshold else row["raw_gap"] for row in rows]
    raw = [row["raw_gap"] for row in rows]
    worsened = sum(v > r for v, r in zip(vals, raw))
    improved = sum(v < r for v, r in zip(vals, raw))
    return float(np.mean(vals)), int(improved), int(worsened)


def select_threshold(train: list[dict[str, Any]], objective: str) -> int:
    candidates = []
    for threshold in range(0, 51):
        mean, improved, worsened = evaluate_threshold(train, threshold)
        rate = worsened / max(1, len(train))
        if objective == "min_gap":
            score = (mean, worsened)
        elif objective == "zero_worse":
            score = (0 if worsened == 0 else 1, mean, worsened)
        elif objective == "worse_le_1":
            score = (0 if worsened <= 1 else 1, mean, worsened)
        elif objective == "mean_plus_lambda_worse":
            score = (mean + LAMBDA * rate, mean, worsened)
        else:
            raise ValueError(objective)
        candidates.append((score, threshold))
    return min(candidates, key=lambda item: item[0])[1]


def train_phase_set(train: list[dict[str, Any]]) -> set[str]:
    phases = sorted({row["planner_phase"] for row in train})
    selected = set()
    for phase in phases:
        items = [row for row in train if row["planner_phase"] == phase]
        raw = np.mean([row["raw_gap"] for row in items])
        corrected = np.mean([row["progress_gap"] for row in items])
        if corrected < raw:
            selected.add(phase)
    return selected


def make_plots(overall: list[dict[str, Any]]) -> None:
    methods = [row["gate_method"] for row in overall]
    gaps = [float(row["gap_to_sim_chunk_mean_l2_mean"]) for row in overall]
    reductions = [float(row["aggregate_reduction_ratio"]) for row in overall]
    worsened = [int(row["worsened_count"]) for row in overall]
    x = np.arange(len(methods))
    plt.figure(figsize=(10.5, 4.8))
    plt.bar(x, gaps)
    plt.xticks(x, methods, rotation=25, ha="right")
    plt.ylabel("gap to Sim chunk mean L2")
    plt.title("LOO Gate Generalization")
    for xi, w in zip(x, worsened):
        plt.text(xi, gaps[xi], f"w{w}", ha="center", va="bottom", fontsize=8)
    plt.tight_layout()
    plt.savefig(OUTPUT_DIR / "loo_gate_generalization_gap.png", dpi=160)
    plt.close()

    plt.figure(figsize=(10.5, 4.8))
    plt.bar(x, reductions)
    plt.xticks(x, methods, rotation=25, ha="right")
    plt.ylabel("aggregate reduction ratio")
    plt.title("LOO Gate Generalization Reduction")
    plt.tight_layout()
    plt.savefig(OUTPUT_DIR / "loo_gate_generalization_reduction.png", dpi=160)
    plt.close()


def make_report(overall: list[dict[str, Any]], by_fold: list[dict[str, Any]], fold_config: list[dict[str, Any]]) -> None:
    lines = []
    for row in overall:
        lines.append(
            f"| {row['gate_method']} | {row['gap_to_sim_chunk_mean_l2_mean']:.6f} | "
            f"{row['aggregate_reduction_ratio']:.6f} | {row['improved_count']} | {row['worsened_count']} |"
        )
    fold_lines = []
    for row in by_fold:
        if row["gate_method"] in {"progress_all", "phase_gate_train", "progress_gate_zero_worse", "oracle_min_raw_progress"}:
            fold_lines.append(
                f"| {row['heldout_episode']} | {row['gate_method']} | {row['gap_to_sim_chunk_mean_l2_mean']:.6f} | "
                f"{row['worsened_count']} |"
            )
    report = f"""# LOO Gate Generalization

[Purpose]
Validate whether phase/progress gates selected only on train episodes generalize to held-out episodes.

[Hypothesis]
Train-selected gates should reduce over-correction relative to unconditional progress correction without using held-out episode labels for selection.

[Inputs]
- `{INPUT_CSV}`

[Checked]
- unconditional progress correction
- train-selected phase gate
- progress threshold selected by train min-gap objective
- progress threshold selected by zero-worsening train objective
- progress threshold selected by worsened<=1 train objective
- progress threshold selected by mean + lambda*worsened_rate, lambda={LAMBDA}
- oracle min(raw, progress) upper bound

[Results]
| Gate method | Gap to Sim chunk mean L2 | Aggregate reduction | Improved | Worsened |
|---|---:|---:|---:|---:|
{chr(10).join(lines)}

Fold excerpt:
| Heldout | Gate method | Gap | Worsened |
|---|---|---:|---:|
{chr(10).join(fold_lines)}

Fold-selected gate config:
```json
{json.dumps(fold_config, ensure_ascii=False, indent=2)}
```

[Status]
VERIFIED offline Level-2 gate generalization.

[Problems]
- Gate selection still uses planner-derived progress/phase metadata.
- Episode LOO is not unseen-layout generalization.

[Decision]
Use the best train-selected gate as the deployability-oriented baseline for Phase5.

[Next]
Proceed to low-rank policy-sensitive subspace experiment.
"""
    (OUTPUT_DIR / "loo_gate_generalization_report.md").write_text(report, encoding="utf-8")


def append_log(summary: dict[str, Any]) -> None:
    key = summary["overall_key_results"]
    entry = f"""

## [2026-09-16 / KST] Phase5 STEP 2 — LOO Gate Generalization

[Purpose]
phase/progress gate가 같은 dataset에서 rule을 고른 과적합인지 확인하기 위해, train fold에서만 gate를 선택하고 held-out episode에 적용한다.

[Hypothesis]
Train-selected gate는 unconditional progress correction 대비 over-correction을 줄이면서 Action Gap 감소를 유지해야 한다.

[Inputs]
- `{INPUT_CSV}`

[Checked]
- unconditional progress correction.
- train-selected phase gate.
- train-selected progress threshold gates: min gap, zero-worse, worse<=1, mean+lambda*worse-rate.
- oracle min(raw, progress) upper bound.

[Changes]
- Added Phase5 STEP 2 outputs under `{OUTPUT_DIR}`.

[Commands]
`python /home/ubuntu/a0509_vla_linux_field_bundle_20260903/lhj/phase5_policy_relevant_completion/scripts/loo_gate_generalization.py`

[Outputs]
- `{OUTPUT_DIR / 'loo_gate_generalization_report.md'}`
- `{OUTPUT_DIR / 'loo_gate_generalization_frame_metrics.csv'}`
- `{OUTPUT_DIR / 'loo_gate_generalization_summary_overall.csv'}`
- `{OUTPUT_DIR / 'loo_gate_generalization_summary_by_fold.csv'}`
- `{OUTPUT_DIR / 'loo_gate_generalization_summary.json'}`

[Results]
- raw/no gate gap: `{key['raw_no_correction']['gap_to_sim_chunk_mean_l2_mean']:.6f}`.
- progress_all gap: `{key['progress_all']['gap_to_sim_chunk_mean_l2_mean']:.6f}`, worsened `{key['progress_all']['worsened_count']}`.
- phase_gate_train gap: `{key['phase_gate_train']['gap_to_sim_chunk_mean_l2_mean']:.6f}`, worsened `{key['phase_gate_train']['worsened_count']}`.
- progress_gate_zero_worse gap: `{key['progress_gate_zero_worse']['gap_to_sim_chunk_mean_l2_mean']:.6f}`, worsened `{key['progress_gate_zero_worse']['worsened_count']}`.
- oracle gap: `{key['oracle_min_raw_progress']['gap_to_sim_chunk_mean_l2_mean']:.6f}`.

[Status]
VERIFIED offline Level-2 gate generalization.

[Problems]
- Uses progress/phase metadata.
- 5 episode LOO only.

[Decision]
Use train-selected gate results for deployability-oriented correction comparison.

[Next]
Run policy-sensitive low-rank subspace experiment.
"""
    with (LHJ / "작업기록.md").open("a", encoding="utf-8") as handle:
        handle.write(entry)


def main() -> int:
    OUTPUT_DIR.mkdir(parents=True, exist_ok=True)
    rows = read_csv(INPUT_CSV)
    merged = base_by_frame(rows)
    all_rows = list(merged.values())
    episodes = sorted({row["episode_id"] for row in all_rows})
    frame_rows = []
    fold_config = []
    for heldout in episodes:
        train = [row for row in all_rows if row["episode_id"] != heldout]
        test = [row for row in all_rows if row["episode_id"] == heldout]
        thresholds = {
            "progress_gate_min_gap": select_threshold(train, "min_gap"),
            "progress_gate_zero_worse": select_threshold(train, "zero_worse"),
            "progress_gate_worse_le_1": select_threshold(train, "worse_le_1"),
            "progress_gate_lambda": select_threshold(train, "mean_plus_lambda_worse"),
        }
        phases_on = train_phase_set(train)
        fold_config.append({"heldout_episode": heldout, "thresholds": thresholds, "phases_on": sorted(phases_on)})
        for row in test:
            methods = {
                "raw_no_correction": row["raw_gap"],
                "progress_all": row["progress_gap"],
                "phase_gate_train": row["progress_gap"] if row["planner_phase"] in phases_on else row["raw_gap"],
                "oracle_min_raw_progress": min(row["raw_gap"], row["progress_gap"]),
            }
            for method, threshold in thresholds.items():
                methods[method] = row["progress_gap"] if row["progress"] <= threshold else row["raw_gap"]
            for method, value in methods.items():
                frame_rows.append(
                    {
                        "frame": row["frame"],
                        "episode_id": row["episode_id"],
                        "heldout_episode": heldout,
                        "planner_phase": row["planner_phase"],
                        "progress": row["progress"],
                        "gate_method": method,
                        "raw_gap": row["raw_gap"],
                        "progress_gap": row["progress_gap"],
                        "selected_gap": value,
                        "gap_reduction": row["raw_gap"] - value,
                    }
                )
    overall = aggregate(frame_rows, ["gate_method"])
    by_fold = aggregate(frame_rows, ["heldout_episode", "gate_method"])
    by_phase = aggregate(frame_rows, ["planner_phase", "gate_method"])
    write_csv(OUTPUT_DIR / "loo_gate_generalization_frame_metrics.csv", frame_rows)
    write_csv(OUTPUT_DIR / "loo_gate_generalization_summary_overall.csv", overall)
    write_csv(OUTPUT_DIR / "loo_gate_generalization_summary_by_fold.csv", by_fold)
    write_csv(OUTPUT_DIR / "loo_gate_generalization_summary_by_phase.csv", by_phase)
    write_json(OUTPUT_DIR / "loo_gate_generalization_fold_config.json", fold_config)
    make_plots(overall)
    make_report(overall, by_fold, fold_config)
    key = {
        row["gate_method"]: {
            "gap_to_sim_chunk_mean_l2_mean": float(row["gap_to_sim_chunk_mean_l2_mean"]),
            "aggregate_reduction_ratio": float(row["aggregate_reduction_ratio"]),
            "improved_count": int(row["improved_count"]),
            "worsened_count": int(row["worsened_count"]),
        }
        for row in overall
    }
    summary = {
        "experiment": "Phase5 STEP 2 LOO gate generalization",
        "policy_scope": "oftplus_h5_vision offline 225-pair dataset only",
        "frames": len(all_rows),
        "lambda": LAMBDA,
        "overall_key_results": key,
        "fold_config": fold_config,
        "outputs": {
            "frame_metrics": str(OUTPUT_DIR / "loo_gate_generalization_frame_metrics.csv"),
            "overall": str(OUTPUT_DIR / "loo_gate_generalization_summary_overall.csv"),
            "by_fold": str(OUTPUT_DIR / "loo_gate_generalization_summary_by_fold.csv"),
            "report": str(OUTPUT_DIR / "loo_gate_generalization_report.md"),
        },
        "interpretation_limits": [
            "Uses progress/phase metadata.",
            "Five episode LOO only.",
            "Offline action evidence only.",
        ],
    }
    write_json(OUTPUT_DIR / "loo_gate_generalization_summary.json", summary)
    append_log(summary)
    print(json.dumps(summary, ensure_ascii=False, indent=2)[:10000])
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
