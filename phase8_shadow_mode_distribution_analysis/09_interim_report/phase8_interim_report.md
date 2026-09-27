# Phase 8 Interim Report — Shadow-Mode Distribution Analysis

Generated: 2026-09-22T21:23:57

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

| Comparison | Factor | Obs L1 | Vision cos | Projector cos | Hidden cos | Action L2 | Gripper | Grip disagree |
| --- | --- | --- | --- | --- | --- | --- | --- | --- |
| real4_vs_real8 | lighting_change | 0.086176 | 0.091892 | 0.067887 | 0.214049 | 0.291046 | 0.290410 | 0.346667 |
| real8_vs_real9 | blue_yellow_cube_position_change | 0.016755 | 0.022556 | 0.020605 | 0.099563 | 0.027105 | 0.025872 | 0.008889 |
| real9_vs_real10 | extra_object_added | 0.026796 | 0.058087 | 0.043349 | 0.145395 | 0.027705 | 0.025659 | 0.008889 |

## Fixed Sim Reference Matrix

| Comparison | Factor | Obs L1 | Vision cos | Projector cos | Hidden cos | Action L2 | Gripper | Grip disagree |
| --- | --- | --- | --- | --- | --- | --- | --- | --- |
| sim4_vs_real4 | baseline_sim_real | 0.261196 | 0.151481 | 0.109822 | 0.225261 | 0.334297 | 0.333699 | 0.360000 |
| sim4_vs_real8 | lighting_change_vs_fixed_sim | 0.244911 | 0.188875 | 0.144493 | 0.232378 | 0.056566 | 0.054921 | 0.013333 |
| sim4_vs_real9 | distractor_cube_change_vs_fixed_sim | 0.242501 | 0.201537 | 0.160912 | 0.237687 | 0.050926 | 0.047851 | 0.022222 |
| sim4_vs_real10 | extra_object_vs_fixed_sim | 0.261253 | 0.245028 | 0.198877 | 0.257501 | 0.035899 | 0.030737 | 0.013333 |

## Phase / Action Component Summary

| Comparison | Phase | N | Action L2 | Trans | Rot | Grip | Grip disagree |
| --- | --- | --- | --- | --- | --- | --- | --- |
| real4_vs_real8 | hold | 2 | 0.604313 | 0.004648 | 0.005569 | 0.604260 | 0.800000 |
| real4_vs_real8 | alignment | 17 | 0.587041 | 0.006363 | 0.008548 | 0.586867 | 0.729412 |
| real4_vs_real8 | descent_to_grasp | 8 | 0.223785 | 0.004399 | 0.003846 | 0.223286 | 0.200000 |
| real4_vs_real8 | grasp_close | 10 | 0.008843 | 0.001878 | 0.000292 | 0.008203 | 0.000000 |
| real4_vs_real8 | lift | 8 | 0.003752 | 0.002556 | 0.000243 | 0.001855 | 0.000000 |
| real8_vs_real9 | hold | 2 | 0.059507 | 0.000456 | 0.001301 | 0.059448 | 0.000000 |
| real8_vs_real9 | descent_to_grasp | 8 | 0.047927 | 0.001211 | 0.001217 | 0.047125 | 0.050000 |
| real8_vs_real9 | alignment | 17 | 0.038261 | 0.001815 | 0.006344 | 0.036051 | 0.000000 |
| real8_vs_real9 | grasp_close | 10 | 0.005045 | 0.000477 | 0.000132 | 0.004844 | 0.000000 |
| real8_vs_real9 | lift | 8 | 0.002051 | 0.001338 | 0.000093 | 0.000879 | 0.000000 |
| real9_vs_real10 | descent_to_grasp | 8 | 0.065496 | 0.001883 | 0.002535 | 0.064177 | 0.050000 |
| real9_vs_real10 | hold | 2 | 0.037614 | 0.000652 | 0.002258 | 0.037158 | 0.000000 |
| real9_vs_real10 | alignment | 17 | 0.033528 | 0.002053 | 0.007749 | 0.029765 | 0.000000 |
| real9_vs_real10 | grasp_close | 10 | 0.005015 | 0.000979 | 0.000096 | 0.004687 | 0.000000 |
| real9_vs_real10 | lift | 8 | 0.003426 | 0.002048 | 0.000117 | 0.001758 | 0.000000 |

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
