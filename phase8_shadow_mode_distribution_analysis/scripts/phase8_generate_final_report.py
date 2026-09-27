#!/usr/bin/env python3
from __future__ import annotations

import csv
import datetime as dt
import json
from pathlib import Path
from typing import Any


ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / "final_report"


REAL_CHAIN = ["real4_vs_real8", "real8_vs_real9", "real9_vs_real10"]
SIM_CHAIN = ["sim4_vs_real4", "sim4_vs_real8", "sim4_vs_real9", "sim4_vs_real10"]

FACTOR_LABEL = {
    "real4_vs_real8": "Lighting change with same cube layout",
    "real8_vs_real9": "Blue/yellow distractor cube position change",
    "real9_vs_real10": "Additional non-cube object",
    "sim4_vs_real4": "Fixed sim4 vs real4 baseline",
    "sim4_vs_real8": "Fixed sim4 vs lighting-changed real8",
    "sim4_vs_real9": "Fixed sim4 vs distractor-cube-changed real9",
    "sim4_vs_real10": "Fixed sim4 vs extra-object real10",
}


def read_csv(path: Path) -> list[dict[str, str]]:
    with path.open(newline="", encoding="utf-8") as fp:
        return list(csv.DictReader(fp))


def by_id(rows: list[dict[str, str]], key: str = "comparison_id") -> dict[str, dict[str, str]]:
    return {row[key]: row for row in rows}


def f(row: dict[str, str] | None, key: str, default: float = 0.0) -> float:
    if row is None:
        return default
    value = row.get(key, "")
    if value == "" or value is None:
        return default
    return float(value)


def write_csv(path: Path, rows: list[dict[str, Any]], fields: list[str] | None = None) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    if fields is None:
        fields = list(rows[0].keys()) if rows else []
    with path.open("w", newline="", encoding="utf-8") as fp:
        writer = csv.DictWriter(fp, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)


def write_json(path: Path, payload: Any) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, indent=2, ensure_ascii=False), encoding="utf-8")


def fmt(value: Any, digits: int = 6) -> str:
    if isinstance(value, float):
        return f"{value:.{digits}f}"
    return str(value)


def md_table(rows: list[dict[str, Any]], fields: list[tuple[str, str]]) -> str:
    lines = [
        "| " + " | ".join(label for _, label in fields) + " |",
        "| " + " | ".join("---" for _ in fields) + " |",
    ]
    for row in rows:
        lines.append("| " + " | ".join(fmt(row[key]) for key, _ in fields) + " |")
    return "\n".join(lines)


def rank(rows: list[dict[str, Any]], key: str) -> list[str]:
    return [str(row["comparison_id"]) for row in sorted(rows, key=lambda r: float(r[key]), reverse=True)]


def main() -> int:
    OUT.mkdir(parents=True, exist_ok=True)

    interim = by_id(read_csv(ROOT / "09_interim_report" / "phase8_evidence_matrix.csv"))
    lowrank = by_id(read_csv(ROOT / "10_policy_sensitive_projection" / "sensitive_projection_summary_overall.csv"))
    grip = by_id(read_csv(ROOT / "11_gripper_sensitive_direction" / "gripper_sensitive_summary_overall.csv"))
    align = by_id(read_csv(ROOT / "03_pair_alignment" / "alignment_statistics.csv"))

    rows: list[dict[str, Any]] = []
    for cid in REAL_CHAIN + SIM_CHAIN:
        base = interim[cid]
        lr = lowrank.get(cid)
        gr = grip.get(cid)
        ar = align.get(cid)
        rows.append(
            {
                "comparison_id": cid,
                "comparison_type": base["comparison_type"],
                "factor": FACTOR_LABEL[cid],
                "n_pairs": int(float(base["n_pairs"])),
                "max_planned_ee_translation_error_m": f(ar, "ee_translation_error_m_max", 0.0),
                "observation_l1": f(base, "observation_l1"),
                "observation_ssim": f(base, "observation_ssim"),
                "brightness_diff": f(base, "brightness_diff"),
                "vision_cosine": f(base, "vision_cosine"),
                "projector_cosine": f(base, "projector_cosine"),
                "hidden_cosine": f(base, "hidden_cosine"),
                "phase6_sensitive_energy": f(lr, "hidden_sensitive_energy_mean"),
                "phase6_sensitive_ratio": f(lr, "hidden_sensitive_ratio_mean"),
                "gripper_sensitive_energy": f(gr, "gripper_sensitive_energy_mean"),
                "gripper_sensitive_ratio": f(gr, "gripper_sensitive_ratio_mean"),
                "gripper_delta_cosine_negative_grad": f(gr, "delta_cosine_with_negative_grad_mean"),
                "action_l2": f(base, "response_action_chunk_mean_l2"),
                "translation_l2": f(base, "response_translation_l2"),
                "rotation_l2": f(base, "response_rotation_l2"),
                "gripper_gap": f(base, "response_gripper_abs"),
                "gripper_disagreement_rate": f(base, "gripper_disagreement_rate"),
            }
        )

    fields = list(rows[0].keys())
    write_csv(OUT / "phase8_final_evidence_matrix.csv", rows, fields)

    real_rows = [row for row in rows if row["comparison_id"] in REAL_CHAIN]
    sim_rows = [row for row in rows if row["comparison_id"] in SIM_CHAIN]

    summary = {
        "generated_at": dt.datetime.now().isoformat(timespec="seconds"),
        "policy": "oftplus_h5_vision, checkpoint step 28560",
        "dataset": {
            "real_episodes": ["episode_000004", "episode_000008", "episode_000009", "episode_000010"],
            "sim_reference": "episode_000004",
            "pairs_per_comparison": 45,
        },
        "real_chain_rankings": {
            "observation_l1": rank(real_rows, "observation_l1"),
            "hidden_cosine": rank(real_rows, "hidden_cosine"),
            "phase6_sensitive_energy": rank(real_rows, "phase6_sensitive_energy"),
            "gripper_sensitive_energy": rank(real_rows, "gripper_sensitive_energy"),
            "action_l2": rank(real_rows, "action_l2"),
            "gripper_gap": rank(real_rows, "gripper_gap"),
        },
        "status": "COMPLETED_PHASE8_OFFLINE_SYNTHESIS",
        "rollout_status": "NOT_EXECUTED",
    }
    write_json(OUT / "phase8_final_summary.json", summary)

    (OUT / "phase8_final_report.md").write_text(build_report(real_rows, sim_rows, summary), encoding="utf-8")
    (OUT / "limitations.md").write_text(build_limitations(), encoding="utf-8")
    (OUT / "recommended_next_actions.md").write_text(build_next_actions(), encoding="utf-8")
    print(json.dumps({"out": str(OUT), "rows": len(rows)}, indent=2))
    return 0


def build_report(real_rows: list[dict[str, Any]], sim_rows: list[dict[str, Any]], summary: dict[str, Any]) -> str:
    main_fields = [
        ("comparison_id", "Comparison"),
        ("factor", "Factor"),
        ("observation_l1", "Obs L1"),
        ("hidden_cosine", "Hidden cos"),
        ("phase6_sensitive_energy", "Phase6 sens"),
        ("gripper_sensitive_energy", "Grip sens"),
        ("action_l2", "Action L2"),
        ("gripper_gap", "Grip gap"),
    ]
    component_fields = [
        ("comparison_id", "Comparison"),
        ("translation_l2", "Translation"),
        ("rotation_l2", "Rotation"),
        ("gripper_gap", "Gripper"),
        ("gripper_disagreement_rate", "Grip disagree"),
        ("max_planned_ee_translation_error_m", "Max planned EEF diff m"),
    ]
    return f"""# Phase 8 Final Report — Shadow-Mode Distribution Analysis

Generated: {summary["generated_at"]}

## 1. Research Question

Phase 8 asks whether real-world visual perturbations around the same orange-cube grasp task change the VLA policy in a policy-relevant way.

The key question is not simply:

`Does the image distribution change?`

but:

`Does the changed observation move the internal representation in action-sensitive directions and change the final policy action?`

## 2. Dataset And Conditions

Policy context:

- Variant: `oftplus_h5_vision`
- Checkpoint: step 28560
- Instruction: `Pick up the orange cube.`
- Proprio: not used

Real conditions:

- `episode_000004`: baseline real condition.
- `episode_000008`: same cube layout as episode 4, lighting changed.
- `episode_000009`: episode 8 condition, blue/yellow distractor cubes moved.
- `episode_000010`: episode 9 condition, additional non-cube object added.

The requested real-only chain is:

`real4_vs_real8` → `real8_vs_real9` → `real9_vs_real10`

Fixed sim-reference comparisons use `sim episode_000004` as the common sim image/reference condition.

## 3. Analysis Pipeline

The Phase 8 pipeline was:

1. Validate episode files, frame count, image count, timing, and pairability.
2. Build aligned pairs for real-only and fixed-sim comparisons.
3. Measure observation gap.
4. Extract full-forward VLA features and policy actions.
5. Measure representation gaps at vision, projector, action-hidden, and action-head output.
6. Measure response action gap by translation, rotation, and gripper.
7. Project hidden differences onto the Phase 6 policy-sensitive low-rank basis.
8. Compute gripper-specific local-gradient sensitivity because Phase 8 action gap is gripper-dominated.

## 4. Main Evidence Matrix

{md_table(real_rows, main_fields)}

## 5. Action Component Breakdown

{md_table(real_rows, component_fields)}

## 6. Fixed Sim Reference Summary

{md_table(sim_rows, main_fields)}

## 7. Main Findings

### Finding 1 — Lighting Change Is The Dominant Policy-Relevant Perturbation

`real4_vs_real8` is largest across:

- Observation gap
- Vision/projector/action-hidden representation gap
- Phase 6 policy-sensitive low-rank projection
- Gripper-specific local-gradient projection
- Final response action gap
- Gripper output disagreement

This supports the interpretation that the lighting change is not merely a visual distribution shift. In this dataset, it also enters hidden directions that matter for the policy's gripper output.

### Finding 2 — Action Gap Is Dominated By Gripper, Not Translation/Rotation

For `real4_vs_real8`, action L2 is `0.291046`, while gripper gap is `0.290410`. Translation and rotation gaps are about `0.004`.

Therefore, the total action L2 is mostly a gripper prediction disagreement. It should not be interpreted as a large Cartesian motion change.

### Finding 3 — Distractor Cube Movement Changes Distribution But Has Much Smaller Action Impact

`real8_vs_real9` changes the blue/yellow cube positions. It does produce measurable observation and representation shifts, but final action impact is small:

- Action L2: `0.027105`
- Gripper gap: `0.025872`

This supports the broader project hypothesis that visual/representation gap magnitude alone is not sufficient. The policy-relevant component matters.

### Finding 4 — Extra Object Produces More Hidden Shift Than Distractor Movement But Similar Action Gap

`real9_vs_real10` has larger observation and hidden gaps than `real8_vs_real9`, but its final action gap is nearly the same:

- `real8_vs_real9` action L2: `0.027105`
- `real9_vs_real10` action L2: `0.027705`

This is another example where a larger distribution or hidden shift does not automatically imply a larger policy impact.

### Finding 5 — Gripper-Specific Sensitivity Sharpens The Interpretation

Using a local gripper-only gradient:

- `real4_vs_real8` gripper-sensitive energy: `21.025672`
- `real8_vs_real9` gripper-sensitive energy: `4.314261`
- `real9_vs_real10` gripper-sensitive energy: `5.041613`

This directly matches the gripper/action gap pattern and is more diagnostic than global representation distance.

## 8. Claims Supported By Current Evidence

Supported:

- The lighting change in episode 8 is the strongest policy-relevant perturbation among the tested Phase 8 real-only changes.
- Phase 8 final action disagreement is gripper-dominated.
- Non-target cube movement and extra object insertion produce measurable distribution/representation shifts but much smaller action effects.
- Policy-sensitive and gripper-sensitive projections explain the action-gap pattern better than observation gap alone.

Not supported yet:

- That lighting causes real robot task failure.
- That extra objects are always harmless.
- That offline policy-output disagreement predicts rollout success.
- That the same result generalizes to unseen layouts, other cameras, or other tasks.

## 9. Status

Status: `COMPLETED_PHASE8_OFFLINE_SYNTHESIS`

Real rollout: `NOT_EXECUTED`

Shadow Mode: not executed in this phase.
"""


def build_limitations() -> str:
    return """# Phase 8 Limitations

1. This is offline policy-output analysis, not closed-loop robot performance.

2. The gripper/action gap is disagreement between two policy outputs under different observations. It is not ground-truth action error.

3. Episode count is small: 45 frames per comparison and only one trajectory family.

4. Phase 8 real-only changes are sequential, not fully factorial:
   - real4→real8 changes lighting.
   - real8→real9 changes blue/yellow cube placement.
   - real9→real10 adds an object on top of the previous condition.

5. Sim images were fixed to episode 4 for reference. Sim was not regenerated to exactly match each new real perturbation.

6. `real9_vs_real10` has a larger planned EEF translation difference than the other real-only comparisons, so small action changes should be interpreted with correspondence caution.

7. The Phase 6 low-rank sensitive basis was learned from the previous 5-episode P0 Real-Sim setting, not from Phase 8 data.

8. The gripper-specific direction uses local first-order gradients and is not a causal intervention by itself.

9. Results apply to `oftplus_h5_vision` checkpoint step 28560 only. They are not ROS proprio-policy results.
"""


def build_next_actions() -> str:
    return """# Recommended Next Actions

## Immediate

1. Use this Phase 8 report as the current offline evidence summary.

2. Add frame-level correlation plots:
   - gripper-sensitive energy vs gripper gap
   - Phase 6 sensitive energy vs action gap
   - hidden cosine vs action gap

3. Inspect high-gap frames from `real4_vs_real8`, especially hold/alignment, to confirm the visual trigger of gripper disagreement.

## Next Data Collection

1. Collect controlled lighting sweep:
   - normal
   - mildly dark
   - strongly dark
   - directional shadow

2. Keep cube layout and trajectory fixed for lighting sweep.

3. For object insertion, collect multiple object types and positions to avoid one-off conclusions.

## Sim Extension

1. Generate matching sim variants for:
   - darker lighting
   - extra object near cube
   - distractor cube position changes

2. Compare fixed-sim reference vs matched-sim condition to separate:
   - real-only sensitivity
   - Real-Sim mismatch reduction

## Deployment Track

1. Do not start closed-loop rollout from this result alone.

2. Next practical validation should be Shadow Mode:
   - real camera input
   - VLA action logging only
   - planner still controls robot
   - compare gripper timing and gripper output confidence across lighting conditions
"""


if __name__ == "__main__":
    raise SystemExit(main())
