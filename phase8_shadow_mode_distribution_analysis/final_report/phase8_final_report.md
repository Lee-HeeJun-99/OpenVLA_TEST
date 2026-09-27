# Phase 8 Final Report — Shadow-Mode Distribution Analysis

Generated: 2026-09-22T21:38:48

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

| Comparison | Factor | Obs L1 | Hidden cos | Phase6 sens | Grip sens | Action L2 | Grip gap |
| --- | --- | --- | --- | --- | --- | --- | --- |
| real4_vs_real8 | Lighting change with same cube layout | 0.086176 | 0.214049 | 52.419522 | 21.025672 | 0.291046 | 0.290410 |
| real8_vs_real9 | Blue/yellow distractor cube position change | 0.016755 | 0.099563 | 23.036442 | 4.314261 | 0.027105 | 0.025872 |
| real9_vs_real10 | Additional non-cube object | 0.026796 | 0.145395 | 33.178598 | 5.041613 | 0.027705 | 0.025659 |

## 5. Action Component Breakdown

| Comparison | Translation | Rotation | Gripper | Grip disagree | Max planned EEF diff m |
| --- | --- | --- | --- | --- | --- |
| real4_vs_real8 | 0.004264 | 0.004268 | 0.290410 | 0.346667 | 0.000767 |
| real8_vs_real9 | 0.001265 | 0.002717 | 0.025872 | 0.008889 | 0.000611 |
| real9_vs_real10 | 0.001721 | 0.003520 | 0.025659 | 0.008889 | 0.007999 |

## 6. Fixed Sim Reference Summary

| Comparison | Factor | Obs L1 | Hidden cos | Phase6 sens | Grip sens | Action L2 | Grip gap |
| --- | --- | --- | --- | --- | --- | --- | --- |
| sim4_vs_real4 | Fixed sim4 vs real4 baseline | 0.261196 | 0.225261 | 58.572548 | 29.558661 | 0.334297 | 0.333699 |
| sim4_vs_real8 | Fixed sim4 vs lighting-changed real8 | 0.244911 | 0.232378 | 39.413381 | 10.785413 | 0.056566 | 0.054921 |
| sim4_vs_real9 | Fixed sim4 vs distractor-cube-changed real9 | 0.242501 | 0.237687 | 40.273347 | 10.254546 | 0.050926 | 0.047851 |
| sim4_vs_real10 | Fixed sim4 vs extra-object real10 | 0.261253 | 0.257501 | 46.422643 | 7.322363 | 0.035899 | 0.030737 |

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
