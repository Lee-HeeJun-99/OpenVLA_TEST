#!/usr/bin/env python3
"""Join Phase 8 observation and representation summaries."""

from __future__ import annotations

import csv
import json
from pathlib import Path
from statistics import mean
from typing import Any


ROOT = Path("/home/ubuntu/a0509_vla_linux_field_bundle_20260903")
PHASE8 = ROOT / "lhj" / "phase8_shadow_mode_distribution_analysis"


def read_csv_map(path: Path, key: str = "comparison_id") -> dict[str, dict[str, str]]:
    with path.open("r", encoding="utf-8", newline="") as handle:
        return {row[key]: row for row in csv.DictReader(handle)}


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


def f(row: dict[str, str], key: str) -> float | None:
    try:
        return float(row[key])
    except Exception:
        return None


def rank(rows: list[dict[str, Any]], metric: str, descending: bool = True) -> list[str]:
    return [
        row["comparison_id"]
        for row in sorted(rows, key=lambda r: float(r[metric]), reverse=descending)
        if row.get(metric) is not None
    ]


def pearson(xs: list[float], ys: list[float]) -> float | None:
    if len(xs) < 3:
        return None
    mx = mean(xs)
    my = mean(ys)
    x = [v - mx for v in xs]
    y = [v - my for v in ys]
    denom = (sum(v * v for v in x) * sum(v * v for v in y)) ** 0.5
    if denom <= 1e-12:
        return None
    return sum(a * b for a, b in zip(x, y)) / denom


def main() -> int:
    obs = read_csv_map(PHASE8 / "05_observation_gap" / "comparison_summary.csv")
    rep = read_csv_map(PHASE8 / "06_representation_gap" / "comparison_summary.csv")
    mmd_rows = []
    with (PHASE8 / "06_representation_gap" / "distribution_mmd.csv").open("r", encoding="utf-8", newline="") as handle:
        for row in csv.DictReader(handle):
            mmd_rows.append(row)
    mmd = {(row["comparison_id"], row["feature"]): row for row in mmd_rows}

    comparisons = [
        "real4_vs_real8",
        "real8_vs_real9",
        "real9_vs_real10",
        "sim4_vs_real4",
        "sim4_vs_real8",
        "sim4_vs_real9",
        "sim4_vs_real10",
    ]
    joined = []
    for cid in comparisons:
        o = obs[cid]
        r = rep[cid]
        joined.append({
            "comparison_id": cid,
            "comparison_type": "real_sequential" if cid.startswith("real") else "fixed_sim_reference",
            "obs_pixel_l1": f(o, "pixel_l1_mean_mean"),
            "obs_rmse": f(o, "pixel_l2_rmse_mean"),
            "obs_ssim": f(o, "ssim_gray_mean"),
            "obs_brightness_diff": f(o, "brightness_diff_mean"),
            "obs_edge_l1": f(o, "edge_l1_mean_mean"),
            "vision_cosine": f(r, "vision_backbone_output_pooled_cosine_distance_mean"),
            "vision_l2": f(r, "vision_backbone_output_pooled_l2_mean"),
            "projector_cosine": f(r, "projector_output_pooled_cosine_distance_mean"),
            "projector_l2": f(r, "projector_output_pooled_l2_mean"),
            "hidden_cosine": f(r, "action_hidden_states_input_pooled_cosine_distance_mean"),
            "hidden_l2": f(r, "action_hidden_states_input_pooled_l2_mean"),
            "action_output_l2": f(r, "action_head_output_full_l2_mean"),
            "action_output_cosine": f(r, "action_head_output_pooled_cosine_distance_mean"),
            "vision_mmd": f(mmd[(cid, "vision_backbone.output")], "pooled_mmd_rbf"),
            "projector_mmd": f(mmd[(cid, "projector.output")], "pooled_mmd_rbf"),
            "hidden_mmd": f(mmd[(cid, "action_hidden_states.input")], "pooled_mmd_rbf"),
            "action_output_mmd": f(mmd[(cid, "action_head.output")], "pooled_mmd_rbf"),
        })

    out_dir = PHASE8 / "08_distribution_analysis"
    out_dir.mkdir(parents=True, exist_ok=True)
    write_csv(out_dir / "observation_representation_joined_summary.csv", joined)

    real_rows = [row for row in joined if row["comparison_type"] == "real_sequential"]
    sim_rows = [row for row in joined if row["comparison_type"] == "fixed_sim_reference"]
    corr_metrics = ["vision_cosine", "projector_cosine", "hidden_cosine", "action_output_l2", "action_output_cosine"]
    correlations = []
    for subset_name, subset in [("real_sequential", real_rows), ("fixed_sim_reference", sim_rows), ("all", joined)]:
        for metric in corr_metrics:
            xs = [float(row["obs_pixel_l1"]) for row in subset if row["obs_pixel_l1"] is not None and row[metric] is not None]
            ys = [float(row[metric]) for row in subset if row["obs_pixel_l1"] is not None and row[metric] is not None]
            correlations.append({
                "subset": subset_name,
                "x": "obs_pixel_l1",
                "y": metric,
                "count": len(xs),
                "pearson": pearson(xs, ys),
                "note": "Small N; descriptive only.",
            })
    write_csv(out_dir / "observation_representation_correlations.csv", correlations)

    summary = {
        "status": "COMPLETED",
        "joined_rows": len(joined),
        "real_sequential_rank_by_observation_l1": rank(real_rows, "obs_pixel_l1"),
        "real_sequential_rank_by_hidden_cosine": rank(real_rows, "hidden_cosine"),
        "real_sequential_rank_by_action_output_l2": rank(real_rows, "action_output_l2"),
        "fixed_sim_rank_by_hidden_cosine": rank(sim_rows, "hidden_cosine"),
        "fixed_sim_rank_by_action_output_cosine": rank(sim_rows, "action_output_cosine"),
        "important_guardrail": "This joins observation and representation/action-output gaps, but it is still offline model output, not real rollout performance.",
    }
    (out_dir / "summary.json").write_text(json.dumps(summary, indent=2), encoding="utf-8")

    report = [
        "# Phase 8 Distribution Analysis Summary",
        "",
        "## Real Sequential Condition Chain",
        "",
        "| Comparison | Obs L1 | Vision Cos | Projector Cos | Hidden Cos | Action Output L2 |",
        "|---|---:|---:|---:|---:|---:|",
    ]
    for row in real_rows:
        report.append(
            f"| {row['comparison_id']} | {row['obs_pixel_l1']:.6f} | {row['vision_cosine']:.6f} | "
            f"{row['projector_cosine']:.6f} | {row['hidden_cosine']:.6f} | {row['action_output_l2']:.6f} |"
        )
    report.extend([
        "",
        "## Fixed Sim Reference",
        "",
        "| Comparison | Obs L1 | Vision Cos | Projector Cos | Hidden Cos | Action Output Cos |",
        "|---|---:|---:|---:|---:|---:|",
    ])
    for row in sim_rows:
        report.append(
            f"| {row['comparison_id']} | {row['obs_pixel_l1']:.6f} | {row['vision_cosine']:.6f} | "
            f"{row['projector_cosine']:.6f} | {row['hidden_cosine']:.6f} | {row['action_output_cosine']:.6f} |"
        )
    report.extend([
        "",
        "## Interpretation",
        "",
        "- In the real sequential chain, lighting change (`real4_vs_real8`) is largest across observation, visual representation, hidden representation, and action output metrics.",
        "- Non-target cube relocation (`real8_vs_real9`) is smallest across these metrics.",
        "- Extra object insertion (`real9_vs_real10`) is intermediate, but closer to lighting than cube relocation in some representation metrics.",
        "- Fixed sim reference comparisons show the changed real conditions increasingly move away from sim4 in representation, especially `sim4_vs_real10`.",
        "- These are offline representation/action-output gaps; they are not closed-loop rollout success metrics.",
    ])
    (out_dir / "distribution_summary.md").write_text("\n".join(report) + "\n", encoding="utf-8")
    print(json.dumps(summary, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
