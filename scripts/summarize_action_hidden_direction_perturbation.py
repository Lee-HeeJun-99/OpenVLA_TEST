#!/usr/bin/env python3
"""Summarize action-hidden Real->Sim direction perturbation outputs."""

from __future__ import annotations

import csv
import json
from collections import defaultdict
from pathlib import Path

import matplotlib.pyplot as plt


BUNDLE_ROOT = Path("/home/ubuntu/a0509_vla_linux_field_bundle_20260903")
RESULT_ROOT = BUNDLE_ROOT / "lhj/phase4_policy_relevance/action_hidden_direction_perturbation"
LOG_PATH = BUNDLE_ROOT / "lhj/작업기록.md"


def read_csv(path: Path) -> list[dict[str, str]]:
    with path.open("r", encoding="utf-8", newline="") as handle:
        return list(csv.DictReader(handle))


def f(row: dict[str, str], key: str) -> float:
    return float(row[key])


def fmt(value: float) -> str:
    return f"{value:.6f}"


def row_for(rows: list[dict[str, str]], direction: str, alpha: float) -> dict[str, str]:
    for row in rows:
        if row["direction"] == direction and abs(f(row, "alpha") - alpha) < 1e-9:
            return row
    raise KeyError((direction, alpha))


def plot_alpha_curves(overall: list[dict[str, str]]) -> None:
    directions = ["real_to_sim", "random", "orthogonal"]
    metrics = [
        (
            "gap_to_sim_chunk_mean_l2_mean",
            "Action gap to Sim action",
            "gap_to_sim_chunk_mean_vs_alpha.png",
        ),
        (
            "change_from_real_chunk_mean_l2_mean",
            "Action change from Real action",
            "change_from_real_chunk_mean_vs_alpha.png",
        ),
        (
            "gap_reduction_ratio_chunk_mean_l2_mean",
            "Raw-to-Sim gap reduction ratio",
            "gap_reduction_ratio_vs_alpha.png",
        ),
    ]
    by_dir: dict[str, list[dict[str, str]]] = defaultdict(list)
    for row in overall:
        by_dir[row["direction"]].append(row)

    for metric, title, filename in metrics:
        plt.figure(figsize=(7.2, 4.4))
        for direction in directions:
            rows = sorted(by_dir[direction], key=lambda r: f(r, "alpha"))
            plt.plot(
                [f(r, "alpha") for r in rows],
                [f(r, metric) for r in rows],
                marker="o",
                label=direction,
            )
        plt.axvline(0.0, color="0.75", linewidth=1)
        plt.axvline(1.0, color="0.75", linewidth=1, linestyle="--")
        plt.xlabel("alpha")
        plt.ylabel(metric.replace("_mean", ""))
        plt.title(title)
        plt.legend()
        plt.grid(True, alpha=0.25)
        plt.tight_layout()
        plt.savefig(RESULT_ROOT / filename, dpi=160)
        plt.close()


def plot_action_components(overall: list[dict[str, str]]) -> None:
    metrics = [
        ("gap_to_sim_first_translation_l2_mean", "translation"),
        ("gap_to_sim_first_rotation_l2_mean", "rotation"),
        ("gap_to_sim_first_gripper_abs_mean", "gripper"),
    ]
    rows = sorted(
        [r for r in overall if r["direction"] == "real_to_sim"],
        key=lambda r: f(r, "alpha"),
    )
    plt.figure(figsize=(7.2, 4.4))
    for metric, label in metrics:
        plt.plot([f(r, "alpha") for r in rows], [f(r, metric) for r in rows], marker="o", label=label)
    plt.axvline(1.0, color="0.75", linewidth=1, linestyle="--")
    plt.xlabel("alpha")
    plt.ylabel("gap to Sim first action")
    plt.title("Real->Sim Direction: Action Component Gap")
    plt.legend()
    plt.grid(True, alpha=0.25)
    plt.tight_layout()
    plt.savefig(RESULT_ROOT / "real_to_sim_action_components_vs_alpha.png", dpi=160)
    plt.close()


def plot_phase_alpha1(phase_rows: list[dict[str, str]]) -> None:
    selected = [r for r in phase_rows if abs(f(r, "alpha") - 1.0) < 1e-9]
    phases = []
    for row in selected:
        phase = row["planner_phase"]
        if phase not in phases:
            phases.append(phase)
    directions = ["real_to_sim", "random", "orthogonal"]
    width = 0.24
    x = list(range(len(phases)))
    plt.figure(figsize=(8.4, 4.8))
    for i, direction in enumerate(directions):
        values = []
        for phase in phases:
            match = next(
                r for r in selected if r["planner_phase"] == phase and r["direction"] == direction
            )
            values.append(f(match, "gap_to_sim_chunk_mean_l2_mean"))
        xs = [v + (i - 1) * width for v in x]
        plt.bar(xs, values, width=width, label=direction)
    plt.xticks(x, phases, rotation=25, ha="right")
    plt.ylabel("gap to Sim chunk mean L2")
    plt.title("Alpha=1 Gap by Planner Phase")
    plt.legend()
    plt.tight_layout()
    plt.savefig(RESULT_ROOT / "alpha1_gap_by_phase.png", dpi=160)
    plt.close()


def make_report(overall: list[dict[str, str]], phase_rows: list[dict[str, str]]) -> None:
    raw = row_for(overall, "real_to_sim", 0.0)
    real_alpha1 = row_for(overall, "real_to_sim", 1.0)
    rand_alpha1 = row_for(overall, "random", 1.0)
    orth_alpha1 = row_for(overall, "orthogonal", 1.0)
    real_alpha05 = row_for(overall, "real_to_sim", 0.5)
    real_alpha15 = row_for(overall, "real_to_sim", 1.5)

    phase_lines = []
    for row in phase_rows:
        if row["direction"] != "real_to_sim" or abs(f(row, "alpha") - 1.0) > 1e-9:
            continue
        phase_lines.append(
            f"| {row['planner_phase']} | {int(float(row['count']))} | "
            f"{fmt(f(row, 'gap_to_sim_chunk_mean_l2_mean'))} | "
            f"{fmt(f(row, 'gap_reduction_ratio_chunk_mean_l2_mean'))} |"
        )

    report = f"""# Action-Hidden Real→Sim Direction Perturbation

Experiment:
action_hidden_states.input Real→Sim direction perturbation.

Purpose:
Test whether the Real→Sim hidden-state difference is merely a large latent displacement or an action-sensitive direction for the `oftplus_h5_vision` action head.

Hypothesis:
If Δh_RS = h_sim - h_real contains policy-relevant components, perturbing h_real along Δh_RS should move the predicted action toward the Sim action more strongly than same-norm random or orthogonal controls.

Input:
- 225 Real/Sim paired frames from the verified 5-episode offline dataset.
- Feature: `action_hidden_states.input`, shape `(1, 35, 4096)`.
- Policy scope: `oftplus_h5_vision`, checkpoint step 28560.
- Action head checkpoint: `runtime_state/oft_mixed480_step28560_merged/action_head--28560_checkpoint.pt`.

Method:
For each pair, compute `Δh_RS = h_sim - h_real`, then evaluate `h(α) = h_real + αδ` through the action head for `α = [-0.5, 0, 0.25, 0.5, 0.75, 1.0, 1.5]`.

Control:
Two same-norm controls were used for each pair: a random direction and an orthogonal direction relative to Δh_RS.

Metrics:
- Gap to Sim action: chunk mean L2, chunk max L2.
- Change from Real action: chunk mean L2.
- First-action translation L2, rotation L2, gripper absolute gap.
- Overall, episode-wise, phase-wise, and progress-compatible frame outputs.

Result:
| Direction / α | Gap to Sim chunk mean L2 | Change from Real chunk mean L2 | Mean gap reduction ratio |
|---|---:|---:|---:|
| raw Real action, α=0 | {fmt(f(raw, 'gap_to_sim_chunk_mean_l2_mean'))} | {fmt(f(raw, 'change_from_real_chunk_mean_l2_mean'))} | {fmt(f(raw, 'gap_reduction_ratio_chunk_mean_l2_mean'))} |
| Real→Sim, α=0.5 | {fmt(f(real_alpha05, 'gap_to_sim_chunk_mean_l2_mean'))} | {fmt(f(real_alpha05, 'change_from_real_chunk_mean_l2_mean'))} | {fmt(f(real_alpha05, 'gap_reduction_ratio_chunk_mean_l2_mean'))} |
| Real→Sim, α=1.0 | {fmt(f(real_alpha1, 'gap_to_sim_chunk_mean_l2_mean'))} | {fmt(f(real_alpha1, 'change_from_real_chunk_mean_l2_mean'))} | {fmt(f(real_alpha1, 'gap_reduction_ratio_chunk_mean_l2_mean'))} |
| Real→Sim, α=1.5 | {fmt(f(real_alpha15, 'gap_to_sim_chunk_mean_l2_mean'))} | {fmt(f(real_alpha15, 'change_from_real_chunk_mean_l2_mean'))} | {fmt(f(real_alpha15, 'gap_reduction_ratio_chunk_mean_l2_mean'))} |
| random, α=1.0 | {fmt(f(rand_alpha1, 'gap_to_sim_chunk_mean_l2_mean'))} | {fmt(f(rand_alpha1, 'change_from_real_chunk_mean_l2_mean'))} | {fmt(f(rand_alpha1, 'gap_reduction_ratio_chunk_mean_l2_mean'))} |
| orthogonal, α=1.0 | {fmt(f(orth_alpha1, 'gap_to_sim_chunk_mean_l2_mean'))} | {fmt(f(orth_alpha1, 'change_from_real_chunk_mean_l2_mean'))} | {fmt(f(orth_alpha1, 'gap_reduction_ratio_chunk_mean_l2_mean'))} |

At α=1.0, Real→Sim perturbation reduced the action gap to nearly zero because it reconstructs the Sim hidden-state input to the same action head. The important control result is that same-norm random and orthogonal directions changed the action only weakly and did not reduce the gap.

Phase-wise Real→Sim α=1:
| Planner phase | Frames | Gap to Sim chunk mean L2 | Mean gap reduction ratio |
|---|---:|---:|---:|
{chr(10).join(phase_lines)}

Interpretation:
This is direct offline Level-2 evidence that the Real→Sim difference at `action_hidden_states.input` includes an action-sensitive direction. The result is not explained by perturbation norm alone, because same-norm random and orthogonal directions produced much smaller action changes.

Status:
VERIFIED offline Level-2 evidence for the existing `oftplus_h5_vision` 225-pair dataset.

Limitation:
This does not identify which subcomponents/tokens are sensitive yet. It also does not prove environment causality or Real robot performance improvement. The α=1 Real→Sim result is a sanity check of action-head consistency, not deployable correction by itself because paired Sim hidden states are unavailable at deployment time.

Next decision:
Proceed to correction decomposition: estimate action-sensitive vs action-null components and compare full shift, sensitive-only, null-only, random, and no-correction conditions under the same action-dimension and phase breakdown.
"""
    (RESULT_ROOT / "action_hidden_direction_perturbation_report.md").write_text(report, encoding="utf-8")


def append_log() -> None:
    entry = f"""

## 2026-09-16 — action_hidden_states.input direction perturbation summary

Experiment:
Real→Sim direction perturbation at `action_hidden_states.input`.

Purpose:
Identify whether the existing Real/Sim hidden-state gap contains policy-relevant action-sensitive directions.

Hypothesis:
If `Δh_RS = h_sim - h_real` is policy-relevant, perturbing `h_real` along it should move action predictions toward Sim actions more than same-norm random/orthogonal directions.

Input:
- 225 verified Real/Sim pairs, 5 episodes.
- `oftplus_h5_vision`, checkpoint step 28560.
- Feature: `action_hidden_states.input`.

Method:
Ran action head on `h_real + αδ` for `α=[-0.5,0,0.25,0.5,0.75,1.0,1.5]`.

Control:
Same-norm random direction and same-norm orthogonal direction per frame.

Metrics:
Chunk mean/max L2, first-action translation/rotation/gripper gaps, overall/episode/phase summaries.

Result:
- Raw α=0 gap to Sim chunk mean L2: `0.370245`.
- Real→Sim α=1 gap to Sim chunk mean L2: `0.000664`.
- Random α=1 gap to Sim chunk mean L2: `0.365349`.
- Orthogonal α=1 gap to Sim chunk mean L2: `0.364505`.
- Real→Sim α=1 change from Real action: `0.370302`.
- Random/orthogonal α=1 change from Real action: about `0.0085`.

Interpretation:
The Real→Sim hidden direction contains policy-sensitive action components. The effect is not explained by latent norm alone because same-norm controls barely changed action predictions.

Status:
VERIFIED offline Level-2 evidence. No Real robot performance claim.

Limitation:
This does not yet decompose sensitive/null components, identify tokens, prove environment cause, or provide deployable correction.

Next decision:
Run correction decomposition and sensitivity/null-direction analysis.

Outputs:
- `{RESULT_ROOT}/action_hidden_direction_perturbation_report.md`
- `{RESULT_ROOT}/gap_to_sim_chunk_mean_vs_alpha.png`
- `{RESULT_ROOT}/change_from_real_chunk_mean_vs_alpha.png`
- `{RESULT_ROOT}/gap_reduction_ratio_vs_alpha.png`
- `{RESULT_ROOT}/real_to_sim_action_components_vs_alpha.png`
- `{RESULT_ROOT}/alpha1_gap_by_phase.png`
"""
    with LOG_PATH.open("a", encoding="utf-8") as handle:
        handle.write(entry)


def main() -> None:
    overall = read_csv(RESULT_ROOT / "direction_perturbation_summary_overall.csv")
    phase_rows = read_csv(RESULT_ROOT / "direction_perturbation_summary_by_phase.csv")
    plot_alpha_curves(overall)
    plot_action_components(overall)
    plot_phase_alpha1(phase_rows)
    make_report(overall, phase_rows)
    append_log()
    summary = {
        "report": str(RESULT_ROOT / "action_hidden_direction_perturbation_report.md"),
        "plots": [
            str(RESULT_ROOT / "gap_to_sim_chunk_mean_vs_alpha.png"),
            str(RESULT_ROOT / "change_from_real_chunk_mean_vs_alpha.png"),
            str(RESULT_ROOT / "gap_reduction_ratio_vs_alpha.png"),
            str(RESULT_ROOT / "real_to_sim_action_components_vs_alpha.png"),
            str(RESULT_ROOT / "alpha1_gap_by_phase.png"),
        ],
    }
    print(json.dumps(summary, indent=2, ensure_ascii=False))


if __name__ == "__main__":
    main()
