#!/usr/bin/env python3
"""Build Phase 8 interim tables, lightweight SVG figures, and report.

This script intentionally uses only the Python standard library so it can run
in the field bundle environment without installing plotting dependencies.
"""

from __future__ import annotations

import csv
import datetime as dt
import html
import json
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / "09_interim_report"
FIG = OUT / "figures"


REAL_CHAIN = ["real4_vs_real8", "real8_vs_real9", "real9_vs_real10"]
SIM_CHAIN = ["sim4_vs_real4", "sim4_vs_real8", "sim4_vs_real9", "sim4_vs_real10"]
LABELS = {
    "real4_vs_real8": "real4 vs real8\nlighting",
    "real8_vs_real9": "real8 vs real9\nblue/yellow moved",
    "real9_vs_real10": "real9 vs real10\nextra object",
    "sim4_vs_real4": "sim4 vs real4\nbaseline",
    "sim4_vs_real8": "sim4 vs real8\nlighting",
    "sim4_vs_real9": "sim4 vs real9\ncube distractors",
    "sim4_vs_real10": "sim4 vs real10\nextra object",
}


def read_csv_dict(path: Path) -> list[dict[str, str]]:
    with path.open(newline="", encoding="utf-8") as f:
        return list(csv.DictReader(f))


def f(row: dict[str, str], key: str, default: float = 0.0) -> float:
    value = row.get(key, "")
    if value == "" or value is None:
        return default
    return float(value)


def by_id(rows: list[dict[str, str]], key: str = "comparison_id") -> dict[str, dict[str, str]]:
    return {row[key]: row for row in rows}


def write_csv(path: Path, rows: list[dict[str, object]], fieldnames: list[str]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="", encoding="utf-8") as fp:
        writer = csv.DictWriter(fp, fieldnames=fieldnames)
        writer.writeheader()
        for row in rows:
            writer.writerow(row)


def fmt(x: float, digits: int = 6) -> str:
    return f"{x:.{digits}f}"


def svg_bar_chart(
    path: Path,
    title: str,
    data: list[tuple[str, float]],
    ylabel: str,
    width: int = 980,
    height: int = 420,
    color: str = "#4b7bec",
) -> None:
    margin_left, margin_right, margin_top, margin_bottom = 90, 35, 58, 105
    plot_w = width - margin_left - margin_right
    plot_h = height - margin_top - margin_bottom
    max_v = max([v for _, v in data] + [1e-9])
    n = len(data)
    gap = 18
    bar_w = (plot_w - gap * (n - 1)) / max(n, 1)
    lines = [
        f'<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}" viewBox="0 0 {width} {height}">',
        '<rect width="100%" height="100%" fill="white"/>',
        f'<text x="{width/2}" y="30" text-anchor="middle" font-family="Arial" font-size="20" font-weight="700">{html.escape(title)}</text>',
        f'<text x="18" y="{margin_top + plot_h/2}" transform="rotate(-90 18 {margin_top + plot_h/2})" text-anchor="middle" font-family="Arial" font-size="13">{html.escape(ylabel)}</text>',
        f'<line x1="{margin_left}" y1="{margin_top + plot_h}" x2="{width - margin_right}" y2="{margin_top + plot_h}" stroke="#333"/>',
        f'<line x1="{margin_left}" y1="{margin_top}" x2="{margin_left}" y2="{margin_top + plot_h}" stroke="#333"/>',
    ]
    for i in range(5):
        val = max_v * i / 4
        y = margin_top + plot_h - (val / max_v) * plot_h
        lines.append(f'<line x1="{margin_left-5}" y1="{y:.2f}" x2="{width-margin_right}" y2="{y:.2f}" stroke="#ddd"/>')
        lines.append(f'<text x="{margin_left-10}" y="{y+4:.2f}" text-anchor="end" font-family="Arial" font-size="11">{fmt(val, 3)}</text>')
    for idx, (label, value) in enumerate(data):
        x = margin_left + idx * (bar_w + gap)
        h = (value / max_v) * plot_h if max_v else 0
        y = margin_top + plot_h - h
        lines.append(f'<rect x="{x:.2f}" y="{y:.2f}" width="{bar_w:.2f}" height="{h:.2f}" fill="{color}"/>')
        lines.append(f'<text x="{x + bar_w/2:.2f}" y="{y - 7:.2f}" text-anchor="middle" font-family="Arial" font-size="11">{fmt(value, 4)}</text>')
        label_lines = label.split("\n")
        for j, piece in enumerate(label_lines):
            lines.append(
                f'<text x="{x + bar_w/2:.2f}" y="{margin_top + plot_h + 22 + j*14}" '
                f'text-anchor="middle" font-family="Arial" font-size="11">{html.escape(piece)}</text>'
            )
    lines.append("</svg>")
    path.write_text("\n".join(lines), encoding="utf-8")


def svg_grouped_bar(
    path: Path,
    title: str,
    groups: list[str],
    series: list[tuple[str, str, list[float]]],
    ylabel: str,
    width: int = 1080,
    height: int = 460,
) -> None:
    margin_left, margin_right, margin_top, margin_bottom = 85, 40, 70, 110
    plot_w = width - margin_left - margin_right
    plot_h = height - margin_top - margin_bottom
    max_v = max([v for _, _, vals in series for v in vals] + [1e-9])
    group_gap = 34
    group_w = (plot_w - group_gap * (len(groups) - 1)) / len(groups)
    bar_gap = 5
    bar_w = (group_w - bar_gap * (len(series) - 1)) / len(series)
    lines = [
        f'<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}" viewBox="0 0 {width} {height}">',
        '<rect width="100%" height="100%" fill="white"/>',
        f'<text x="{width/2}" y="30" text-anchor="middle" font-family="Arial" font-size="20" font-weight="700">{html.escape(title)}</text>',
        f'<text x="18" y="{margin_top + plot_h/2}" transform="rotate(-90 18 {margin_top + plot_h/2})" text-anchor="middle" font-family="Arial" font-size="13">{html.escape(ylabel)}</text>',
        f'<line x1="{margin_left}" y1="{margin_top + plot_h}" x2="{width - margin_right}" y2="{margin_top + plot_h}" stroke="#333"/>',
        f'<line x1="{margin_left}" y1="{margin_top}" x2="{margin_left}" y2="{margin_top + plot_h}" stroke="#333"/>',
    ]
    for i in range(5):
        val = max_v * i / 4
        y = margin_top + plot_h - (val / max_v) * plot_h
        lines.append(f'<line x1="{margin_left-5}" y1="{y:.2f}" x2="{width-margin_right}" y2="{y:.2f}" stroke="#ddd"/>')
        lines.append(f'<text x="{margin_left-10}" y="{y+4:.2f}" text-anchor="end" font-family="Arial" font-size="11">{fmt(val, 3)}</text>')
    legend_x = margin_left
    for idx, (name, color, _) in enumerate(series):
        x = legend_x + idx * 170
        lines.append(f'<rect x="{x}" y="48" width="13" height="13" fill="{color}"/>')
        lines.append(f'<text x="{x+18}" y="60" font-family="Arial" font-size="12">{html.escape(name)}</text>')
    for gi, group in enumerate(groups):
        gx = margin_left + gi * (group_w + group_gap)
        for si, (_, color, vals) in enumerate(series):
            value = vals[gi]
            x = gx + si * (bar_w + bar_gap)
            h = (value / max_v) * plot_h if max_v else 0
            y = margin_top + plot_h - h
            lines.append(f'<rect x="{x:.2f}" y="{y:.2f}" width="{bar_w:.2f}" height="{h:.2f}" fill="{color}"/>')
        for j, piece in enumerate(LABELS[group].split("\n")):
            lines.append(
                f'<text x="{gx + group_w/2:.2f}" y="{margin_top + plot_h + 24 + j*14}" '
                f'text-anchor="middle" font-family="Arial" font-size="11">{html.escape(piece)}</text>'
            )
    lines.append("</svg>")
    path.write_text("\n".join(lines), encoding="utf-8")


def main() -> int:
    OUT.mkdir(parents=True, exist_ok=True)
    FIG.mkdir(parents=True, exist_ok=True)

    obs = by_id(read_csv_dict(ROOT / "05_observation_gap" / "comparison_summary.csv"))
    rep = by_id(read_csv_dict(ROOT / "06_representation_gap" / "comparison_summary.csv"))
    action = by_id(read_csv_dict(ROOT / "07_shadow_action_analysis" / "action_comparison_summary.csv"))
    phase_rows = read_csv_dict(ROOT / "07_shadow_action_analysis" / "action_phase_summary.csv")

    rows: list[dict[str, object]] = []
    for cid in REAL_CHAIN + SIM_CHAIN:
        orow, rrow, arow = obs[cid], rep[cid], action[cid]
        rows.append(
            {
                "comparison_id": cid,
                "comparison_type": "real_sequential" if cid in REAL_CHAIN else "fixed_sim_reference",
                "factor": {
                    "real4_vs_real8": "lighting_change",
                    "real8_vs_real9": "blue_yellow_cube_position_change",
                    "real9_vs_real10": "extra_object_added",
                    "sim4_vs_real4": "baseline_sim_real",
                    "sim4_vs_real8": "lighting_change_vs_fixed_sim",
                    "sim4_vs_real9": "distractor_cube_change_vs_fixed_sim",
                    "sim4_vs_real10": "extra_object_vs_fixed_sim",
                }[cid],
                "n_pairs": int(float(orow["count"])),
                "observation_l1": f(orow, "pixel_l1_mean_mean"),
                "observation_ssim": f(orow, "ssim_gray_mean"),
                "brightness_diff": f(orow, "brightness_diff_mean"),
                "vision_cosine": f(rrow, "vision_backbone_output_pooled_cosine_distance_mean"),
                "projector_cosine": f(rrow, "projector_output_pooled_cosine_distance_mean"),
                "hidden_cosine": f(rrow, "action_hidden_states_input_pooled_cosine_distance_mean"),
                "action_head_output_l2": f(rrow, "action_head_output_full_l2_mean"),
                "response_action_chunk_mean_l2": f(arow, "chunk_mean_l2_mean"),
                "response_translation_l2": f(arow, "chunk_mean_translation_l2_mean"),
                "response_rotation_l2": f(arow, "chunk_mean_rotation_l2_mean"),
                "response_gripper_abs": f(arow, "chunk_mean_gripper_abs_mean"),
                "gripper_disagreement_rate": f(arow, "gripper_disagreement_rate_mean"),
            }
        )

    fields = list(rows[0].keys())
    write_csv(OUT / "phase8_evidence_matrix.csv", rows, fields)

    real_action_data = [(LABELS[c], f(action[c], "chunk_mean_l2_mean")) for c in REAL_CHAIN]
    svg_bar_chart(FIG / "real_chain_action_gap.svg", "Real-only condition chain: response action gap", real_action_data, "chunk mean L2", color="#e15f41")

    real_obs_data = [(LABELS[c], f(obs[c], "pixel_l1_mean_mean")) for c in REAL_CHAIN]
    svg_bar_chart(FIG / "real_chain_observation_l1.svg", "Real-only condition chain: observation gap", real_obs_data, "pixel L1 mean", color="#2bcbba")

    groups = REAL_CHAIN
    svg_grouped_bar(
        FIG / "real_chain_representation_metrics.svg",
        "Real-only condition chain: representation/action-facing metrics",
        groups,
        [
            ("Vision cosine", "#45aaf2", [f(rep[c], "vision_backbone_output_pooled_cosine_distance_mean") for c in groups]),
            ("Projector cosine", "#26de81", [f(rep[c], "projector_output_pooled_cosine_distance_mean") for c in groups]),
            ("Hidden cosine", "#8854d0", [f(rep[c], "action_hidden_states_input_pooled_cosine_distance_mean") for c in groups]),
            ("Action chunk L2", "#eb3b5a", [f(action[c], "chunk_mean_l2_mean") for c in groups]),
        ],
        "distance / gap",
    )

    svg_grouped_bar(
        FIG / "real_chain_action_components.svg",
        "Real-only condition chain: action component breakdown",
        groups,
        [
            ("Translation", "#45aaf2", [f(action[c], "chunk_mean_translation_l2_mean") for c in groups]),
            ("Rotation", "#fed330", [f(action[c], "chunk_mean_rotation_l2_mean") for c in groups]),
            ("Gripper", "#eb3b5a", [f(action[c], "chunk_mean_gripper_abs_mean") for c in groups]),
        ],
        "component gap",
    )

    phase_for_real = [row for row in phase_rows if row["comparison_id"] in REAL_CHAIN]
    write_csv(
        OUT / "phase8_real_chain_phase_action_summary.csv",
        phase_for_real,
        list(phase_for_real[0].keys()) if phase_for_real else [],
    )

    real_by_action = sorted(
        [row for row in rows if row["comparison_type"] == "real_sequential"],
        key=lambda r: float(r["response_action_chunk_mean_l2"]),
        reverse=True,
    )
    real_by_obs = sorted(
        [row for row in rows if row["comparison_type"] == "real_sequential"],
        key=lambda r: float(r["observation_l1"]),
        reverse=True,
    )
    real_by_hidden = sorted(
        [row for row in rows if row["comparison_type"] == "real_sequential"],
        key=lambda r: float(r["hidden_cosine"]),
        reverse=True,
    )

    summary = {
        "generated_at_kst": dt.datetime.now().isoformat(timespec="seconds"),
        "policy": "oftplus_h5_vision, checkpoint step 28560",
        "num_comparisons": len(rows),
        "real_chain_action_gap_ranking": [r["comparison_id"] for r in real_by_action],
        "real_chain_observation_gap_ranking": [r["comparison_id"] for r in real_by_obs],
        "real_chain_hidden_gap_ranking": [r["comparison_id"] for r in real_by_hidden],
        "primary_interpretation": [
            "Lighting change is the dominant source of action disagreement in the real-only chain.",
            "The response action gap is dominated by gripper output rather than translation/rotation.",
            "Blue/yellow cube relocation and extra object insertion produce smaller but measurable representation/action changes.",
            "All conclusions are offline policy-output disagreement, not rollout success or physical correctness.",
        ],
        "outputs": {
            "evidence_matrix": str(OUT / "phase8_evidence_matrix.csv"),
            "phase_summary": str(OUT / "phase8_real_chain_phase_action_summary.csv"),
            "figures": str(FIG),
        },
    }
    (OUT / "summary.json").write_text(json.dumps(summary, indent=2, ensure_ascii=False), encoding="utf-8")

    report = build_report(rows, phase_for_real)
    (OUT / "phase8_interim_report.md").write_text(report, encoding="utf-8")
    print(f"Wrote Phase 8 interim report to {OUT}")
    return 0


def md_table(rows: list[dict[str, object]], cols: list[tuple[str, str]]) -> str:
    header = "| " + " | ".join(label for _, label in cols) + " |"
    sep = "| " + " | ".join("---" for _ in cols) + " |"
    body = []
    for row in rows:
        vals = []
        for key, _ in cols:
            value = row[key]
            if isinstance(value, float):
                vals.append(fmt(value, 6))
            else:
                vals.append(str(value))
        body.append("| " + " | ".join(vals) + " |")
    return "\n".join([header, sep] + body)


def build_report(rows: list[dict[str, object]], phase_rows: list[dict[str, str]]) -> str:
    real_rows = [r for r in rows if r["comparison_type"] == "real_sequential"]
    sim_rows = [r for r in rows if r["comparison_type"] == "fixed_sim_reference"]
    cols = [
        ("comparison_id", "Comparison"),
        ("factor", "Factor"),
        ("observation_l1", "Obs L1"),
        ("vision_cosine", "Vision cos"),
        ("projector_cosine", "Projector cos"),
        ("hidden_cosine", "Hidden cos"),
        ("response_action_chunk_mean_l2", "Action L2"),
        ("response_gripper_abs", "Gripper"),
        ("gripper_disagreement_rate", "Grip disagree"),
    ]
    phase_focus = []
    for row in phase_rows:
        if row["phase"] in {"hold", "alignment", "descent_to_grasp", "grasp_close", "lift"}:
            phase_focus.append(
                {
                    "comparison_id": row["comparison_id"],
                    "phase": row["phase"],
                    "count": int(float(row["count"])),
                    "chunk_mean_l2": f(row, "chunk_mean_l2_mean"),
                    "translation": f(row, "chunk_mean_translation_l2_mean"),
                    "rotation": f(row, "chunk_mean_rotation_l2_mean"),
                    "gripper": f(row, "chunk_mean_gripper_abs_mean"),
                    "grip_disagree": f(row, "gripper_disagreement_rate_mean"),
                }
            )
    phase_focus.sort(key=lambda r: (str(r["comparison_id"]), -float(r["chunk_mean_l2"])))
    phase_cols = [
        ("comparison_id", "Comparison"),
        ("phase", "Phase"),
        ("count", "N"),
        ("chunk_mean_l2", "Action L2"),
        ("translation", "Trans"),
        ("rotation", "Rot"),
        ("gripper", "Grip"),
        ("grip_disagree", "Grip disagree"),
    ]
    return f"""# Phase 8 Interim Report — Shadow-Mode Distribution Analysis

Generated: {dt.datetime.now().isoformat(timespec="seconds")}

## Scope

This report summarizes the newly collected real-only environment perturbation episodes:

- `episode_000004`: baseline reference condition.
- `episode_000008`: same cube layout as episode 4, lighting changed.
- `episode_000009`: episode 8 condition with blue/yellow cube positions changed.
- `episode_000010`: episode 9 condition with an additional non-cube object.

The requested real-only comparison chain is:

`real4_vs_real8` → `real8_vs_real9` → `real9_vs_real10`.

Fixed sim-reference comparisons use `sim episode_000004` as the shared sim reference. Policy context is `oftplus_h5_vision`, checkpoint step 28560.

## Key Finding

The strongest real-only change is the lighting condition (`real4_vs_real8`). It is largest not only at the image level, but also at the vision/projector/action-hidden representation levels and in final response action disagreement.

The final response action difference is dominated by the gripper output. Translation and rotation changes are small in all three real-only comparisons.

## Real-Only Evidence Matrix

{md_table(real_rows, cols)}

## Fixed Sim Reference Matrix

{md_table(sim_rows, cols)}

## Phase / Action Component Summary

{md_table(phase_focus, phase_cols)}

## Figures

- [Real chain observation gap](figures/real_chain_observation_l1.svg)
- [Real chain representation metrics](figures/real_chain_representation_metrics.svg)
- [Real chain action gap](figures/real_chain_action_gap.svg)
- [Real chain action components](figures/real_chain_action_components.svg)

## Interpretation

1. `real4_vs_real8` is the dominant condition shift. Its action gap is mainly a gripper prediction disagreement in `hold` and `alignment`.
2. `real8_vs_real9` shows that moving non-target blue/yellow cubes changes visual and latent distributions, but the action impact is much smaller than the lighting shift.
3. `real9_vs_real10` shows that adding an extra non-cube object creates a measurable representation shift and a small action shift, again dominated by gripper.
4. The fixed sim-reference comparisons indicate that `real10` is farthest from the fixed sim reference in action-hidden/action-output metrics, but this is descriptive because sim was not independently regenerated for each real condition.

## Limitations

- This is offline policy-output disagreement, not closed-loop robot performance.
- `response.actions` are policy outputs; the analysis does not establish which condition is physically correct.
- Gripper binary disagreement uses threshold `0.5` as an analysis convention.
- Each condition has 45 paired frames, so statistical claims should remain conservative.
- `real9_vs_real10` has a larger planned EEF translation difference than the other real-only comparisons, so small action differences there should be interpreted with that correspondence caveat.

## Next Decision

The next scientific step is to connect these condition shifts to the policy-sensitive subspace:

- Project each condition's `action_hidden_states.input` difference onto the existing policy-sensitive low-rank basis from Phase 5/6, if compatible.
- Compare sensitive energy vs null energy for `real4_vs_real8`, `real8_vs_real9`, and `real9_vs_real10`.
- Check whether the large lighting-induced gripper disagreement is concentrated in the same sensitive directions that previously explained Action Gap reduction.

If that compatibility is not available, the fallback is to build a Phase 8 local sensitive-direction analysis at `action_hidden_states.input`, focused on gripper-sensitive output dimensions.
"""


if __name__ == "__main__":
    raise SystemExit(main())
