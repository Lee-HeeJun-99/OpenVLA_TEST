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
FRAME = PHASE6 / "policy_sensitive_attribution_frame_metrics.csv"
OBS = PHASE6 / "01_preprocessing" / "preprocessing_observation_frame_metrics.csv"


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


def episode(pair_id: str) -> str:
    return "_".join(pair_id.split("_")[:2])


def f(row: dict[str, str], key: str) -> float:
    return float(row[key])


def mean(rows: list[dict[str, Any]], key: str) -> float:
    vals = [float(row[key]) for row in rows]
    return float(np.mean(vals)) if vals else float("nan")


def condition_means(rows: list[dict[str, Any]], conditions: list[str], key: str) -> dict[str, float]:
    return {condition: mean([row for row in rows if row["condition"] == condition], key) for condition in conditions}


def main() -> int:
    frame_rows = read_csv(FRAME)
    obs_rows = read_csv(OBS)
    obs_by_pair_condition = {(row["pair_id"], row["condition"]): row for row in obs_rows}
    for row in frame_rows:
        row["episode_id"] = episode(row["pair_id"])
        obs = obs_by_pair_condition.get((row["pair_id"], row["condition"]), {})
        row["observation_mse"] = obs.get("mse", "")
        row["observation_ssim_global"] = obs.get("ssim_global", "")

    conditions = sorted({row["condition"] for row in frame_rows})
    episodes = sorted({row["episode_id"] for row in frame_rows})
    objectives = [
        ("action_min", "action_chunk_mean_l2", "min"),
        ("hidden_min", "hidden_l2", "min"),
        ("vision_min", "vision_pooled_l2", "min"),
        ("observation_mse_min", "observation_mse", "min"),
        ("observation_ssim_max", "observation_ssim_global", "max"),
    ]

    fold_rows: list[dict[str, Any]] = []
    for heldout in episodes:
        train = [row for row in frame_rows if row["episode_id"] != heldout]
        test = [row for row in frame_rows if row["episode_id"] == heldout]
        baseline_test = [row for row in test if row["condition"] == "P0_current_paired_image"]
        baseline_gap = mean(baseline_test, "action_chunk_mean_l2")
        oracle_means = condition_means(test, conditions, "action_chunk_mean_l2")
        oracle_condition = min(oracle_means, key=oracle_means.get)
        for objective, metric, direction in objectives:
            train_means = condition_means(train, conditions, metric)
            if direction == "min":
                selected = min(train_means, key=train_means.get)
            else:
                selected = max(train_means, key=train_means.get)
            selected_test = [row for row in test if row["condition"] == selected]
            selected_gap = mean(selected_test, "action_chunk_mean_l2")
            fold_rows.append(
                {
                    "fold_episode": heldout,
                    "selection_objective": objective,
                    "selected_condition": selected,
                    "train_selected_metric": train_means[selected],
                    "test_action_gap": selected_gap,
                    "test_baseline_gap": baseline_gap,
                    "test_action_reduction": baseline_gap - selected_gap,
                    "test_action_reduction_pct": (baseline_gap - selected_gap) / baseline_gap * 100.0 if baseline_gap else np.nan,
                    "oracle_condition": oracle_condition,
                    "oracle_action_gap": oracle_means[oracle_condition],
                    "oracle_action_reduction_pct": (baseline_gap - oracle_means[oracle_condition]) / baseline_gap * 100.0 if baseline_gap else np.nan,
                    "matches_oracle": selected == oracle_condition,
                    "status": "VERIFIED_LOO_HELDOUT_SELECTION",
                }
            )

    summary_rows = []
    for objective, _, _ in objectives:
        rows = [row for row in fold_rows if row["selection_objective"] == objective]
        selected_counts = {condition: sum(row["selected_condition"] == condition for row in rows) for condition in conditions}
        summary_rows.append(
            {
                "selection_objective": objective,
                "fold_count": len(rows),
                "mean_test_action_gap": mean(rows, "test_action_gap"),
                "mean_test_baseline_gap": mean(rows, "test_baseline_gap"),
                "mean_test_action_reduction": mean(rows, "test_action_reduction"),
                "mean_test_action_reduction_pct": mean(rows, "test_action_reduction_pct"),
                "oracle_match_count": sum(bool(row["matches_oracle"]) for row in rows),
                "selected_counts_json": json.dumps(selected_counts, sort_keys=True),
                "status": "VERIFIED_LOO_HELDOUT_SELECTION",
            }
        )
    oracle_summary = {
        "selection_objective": "oracle_test_action_min",
        "fold_count": len(episodes),
        "mean_test_action_gap": mean(fold_rows[:: len(objectives)], "oracle_action_gap"),
        "mean_test_baseline_gap": mean(fold_rows[:: len(objectives)], "test_baseline_gap"),
        "mean_test_action_reduction": mean(
            [
                {
                    "v": float(row["test_baseline_gap"]) - float(row["oracle_action_gap"])
                }
                for row in fold_rows[:: len(objectives)]
            ],
            "v",
        ),
        "mean_test_action_reduction_pct": mean(fold_rows[:: len(objectives)], "oracle_action_reduction_pct"),
        "oracle_match_count": len(episodes),
        "selected_counts_json": "{}",
        "status": "ORACLE_UPPER_BOUND_NOT_DEPLOYABLE",
    }
    summary_rows.append(oracle_summary)

    out_dir = PHASE6 / "07_environment_ranking" / "heldout_selection"
    write_csv(out_dir / "heldout_selection_fold_metrics.csv", fold_rows)
    write_csv(out_dir / "heldout_selection_summary.csv", summary_rows)
    write_csv(PHASE6 / "heldout_generalization.csv", summary_rows)

    plt.figure(figsize=(9, 4.8))
    labels = [row["selection_objective"] for row in summary_rows]
    values = [float(row["mean_test_action_gap"]) for row in summary_rows]
    plt.bar(labels, values)
    plt.ylabel("held-out action gap")
    plt.title("LOO Held-Out Condition Selection")
    plt.xticks(rotation=25, ha="right")
    plt.tight_layout()
    plt.savefig(PHASE6 / "figures" / "heldout_selection_action_gap.png", dpi=180)
    plt.close()

    write_json(
        out_dir / "heldout_selection_summary.json",
        {
            "objectives": objectives,
            "conditions": conditions,
            "episodes": episodes,
            "summary": summary_rows,
            "status": "VERIFIED_LOO_HELDOUT_SELECTION",
            "notes": [
                "Parameter/condition selection is performed on train episodes only.",
                "Oracle uses held-out action gap and is upper bound only.",
                "This is episode held-out, not unseen-layout or real-world generalization.",
            ],
        },
    )
    print(json.dumps({"summary": summary_rows}, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
