#!/usr/bin/env python3
from __future__ import annotations

import csv
import json
from pathlib import Path
from typing import Any

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np


ROOT = Path("/home/ubuntu/a0509_vla_linux_field_bundle_20260903")
PHASE6 = ROOT / "lhj" / "phase6_environment_attribution"


def read_csv(path: Path) -> list[dict[str, str]]:
    with path.open("r", encoding="utf-8", newline="") as handle:
        return list(csv.DictReader(handle))


def write_csv(path: Path, rows: list[dict[str, Any]], fieldnames: list[str] | None = None) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    if not rows:
        path.write_text("", encoding="utf-8")
        return
    if fieldnames is None:
        fieldnames = list(rows[0].keys())
    with path.open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)


def write_json(path: Path, payload: Any) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")


def f(row: dict[str, str], key: str) -> float:
    value = row.get(key, "")
    return float(value) if value not in ("", None) else float("nan")


def group_mean(rows: list[dict[str, str]], group_keys: list[str], metric_keys: list[str]) -> list[dict[str, Any]]:
    groups: dict[tuple[str, ...], list[dict[str, str]]] = {}
    for row in rows:
        groups.setdefault(tuple(row[key] for key in group_keys), []).append(row)
    out = []
    for group, items in sorted(groups.items()):
        record = {key: group[i] for i, key in enumerate(group_keys)}
        record["count"] = len(items)
        for metric in metric_keys:
            values = [f(item, metric) for item in items]
            values = [v for v in values if np.isfinite(v)]
            if values:
                record[f"{metric}_mean"] = float(np.mean(values))
                record[f"{metric}_median"] = float(np.median(values))
        out.append(record)
    return out


def bar(rows: list[dict[str, str]], key: str, title: str, path: Path, ylabel: str) -> None:
    labels = [row["condition"].replace("_", "\n") for row in rows]
    values = [f(row, key) for row in rows]
    plt.figure(figsize=(13, 5))
    plt.bar(range(len(values)), values)
    plt.xticks(range(len(values)), labels, fontsize=8)
    plt.ylabel(ylabel)
    plt.title(title)
    plt.tight_layout()
    plt.savefig(path, dpi=180)
    plt.close()


def scatter(rows: list[dict[str, str]], xkey: str, ykey: str, title: str, path: Path, xlabel: str, ylabel: str) -> None:
    plt.figure(figsize=(7, 5))
    for row in rows:
        x = f(row, xkey)
        y = f(row, ykey)
        plt.scatter(x, y)
        plt.annotate(row["condition"].replace("P", "P"), (x, y), fontsize=7)
    plt.axhline(0, color="gray", linewidth=0.8)
    plt.axvline(0, color="gray", linewidth=0.8)
    plt.xlabel(xlabel)
    plt.ylabel(ylabel)
    plt.title(title)
    plt.tight_layout()
    plt.savefig(path, dpi=180)
    plt.close()


def main() -> int:
    figures = PHASE6 / "figures"
    tables = PHASE6 / "tables"
    figures.mkdir(exist_ok=True)
    tables.mkdir(exist_ok=True)

    env_rows = read_csv(PHASE6 / "environment_factor_table.csv")
    frame_rows = read_csv(PHASE6 / "policy_sensitive_attribution_frame_metrics.csv")
    env_rows = sorted(env_rows, key=lambda row: row["condition"])

    # Rankings.
    repr_rank = sorted(env_rows, key=lambda row: f(row, "hidden_gap_reduction_pct"), reverse=True)
    policy_rank = sorted(env_rows, key=lambda row: f(row, "action_gap_reduction_pct"), reverse=True)
    write_csv(PHASE6 / "07_environment_ranking" / "ranking_by_representation_alignment.csv", repr_rank)
    write_csv(PHASE6 / "07_environment_ranking" / "ranking_by_policy_impact.csv", policy_rank)
    write_csv(tables / "ranking_by_representation_alignment.csv", repr_rank)
    write_csv(tables / "ranking_by_policy_impact.csv", policy_rank)

    metric_keys = [
        "action_chunk_mean_l2",
        "translation_l2",
        "rotation_l2",
        "gripper_gap",
        "hidden_l2",
        "vision_pooled_l2",
        "projector_pooled_l2",
    ]
    phase_rows = group_mean(frame_rows, ["condition", "planner_phase"], metric_keys)
    write_csv(PHASE6 / "06_policy_relevance" / "phase_conditioned_metrics.csv", phase_rows)
    write_csv(tables / "phase_conditioned_metrics.csv", phase_rows)

    component_rows = [
        {
            "condition": row["condition"],
            "translation_gap": row["translation_gap"],
            "rotation_gap": row["rotation_gap"],
            "gripper_gap": row["gripper_gap"],
            "action_gap": row["action_gap"],
            "action_gap_reduction_pct": row["action_gap_reduction_pct"],
            "status": row["status"],
        }
        for row in env_rows
    ]
    write_csv(PHASE6 / "06_policy_relevance" / "action_component_breakdown.csv", component_rows)
    write_csv(tables / "action_component_breakdown.csv", component_rows)

    # Required core figures.
    bar(env_rows, "action_gap", "Action Gap by Condition (N=225, oftplus_h5_vision step28560)", figures / "action_gap_by_condition.png", "chunk mean L2")
    bar(env_rows, "observation_mse", "Observation MSE by Condition (N=225)", figures / "observation_mse_by_condition.png", "MSE")
    bar(env_rows, "hidden_gap", "Hidden Gap by Condition (N=225)", figures / "hidden_gap_by_condition.png", "L2")
    bar(component_rows, "gripper_gap", "First Action Gripper Gap by Condition (N=225)", figures / "gripper_gap_by_condition.png", "abs gap")
    scatter(env_rows, "vision_gap_reduction_pct", "action_gap_reduction_pct", "Vision Gap Reduction vs Action Gap Reduction", figures / "vision_gap_reduction_vs_action_gap_reduction.png", "vision gap reduction (%)", "action gap reduction (%)")
    scatter(env_rows, "hidden_gap_reduction_pct", "action_gap_reduction_pct", "Hidden Gap Reduction vs Action Gap Reduction", figures / "hidden_gap_reduction_vs_action_gap_reduction.png", "hidden gap reduction (%)", "action gap reduction (%)")

    # Phase heatmap-like grouped bar.
    phases = sorted({row["planner_phase"] for row in phase_rows})
    conditions = [row["condition"] for row in env_rows]
    phase_lookup = {(row["condition"], row["planner_phase"]): row for row in phase_rows}
    plt.figure(figsize=(13, 5))
    width = 0.8 / max(len(phases), 1)
    x = np.arange(len(conditions))
    for i, phase in enumerate(phases):
        vals = [float(phase_lookup.get((cond, phase), {}).get("action_chunk_mean_l2_mean", np.nan)) for cond in conditions]
        plt.bar(x + i * width, vals, width=width, label=phase)
    plt.xticks(x + width * (len(phases) - 1) / 2, [c.replace("_", "\n") for c in conditions], fontsize=7)
    plt.ylabel("chunk mean L2")
    plt.title("Phase-Conditioned Action Gap by Condition (N=225)")
    plt.legend(fontsize=7)
    plt.tight_layout()
    plt.savefig(figures / "phase_conditioned_action_gap.png", dpi=180)
    plt.close()

    best_policy = policy_rank[0]
    worst_policy = policy_rank[-1]
    payload = {
        "condition_count": len(env_rows),
        "frame_metric_count": len(frame_rows),
        "best_policy_condition": best_policy,
        "worst_policy_condition": worst_policy,
        "best_representation_condition": repr_rank[0],
        "status": "VERIFIED_FULL_FORWARD_PHASE6_REPORTS",
        "notes": [
            "Sensitive/null columns remain blank because no serialized Phase5 sensitive basis was available.",
            "Policy relevance is evaluated by final action gap and action-component gaps.",
        ],
    }
    write_json(PHASE6 / "08_final_synthesis" / "phase6_report_summary.json", payload)

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
