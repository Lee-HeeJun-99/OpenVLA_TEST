# Phase 6 Environment Attribution Summary

## 1. Research Question

Which controllable Real-Sim observation/environment factors produce policy-relevant representation differences that affect action prediction?

## 2. Existing Verified Evidence

Phase 5 showed that reducing the whole representation gap is not sufficient; action-hidden policy-sensitive structure is what matters for Action Gap. Phase 6 therefore evaluates environment/preprocessing factors through final `oftplus_h5_vision` action output, not through visual similarity alone.

## 3. Full-Forward Feasibility Audit

Status: `VERIFIED_FULL_FORWARD` after local GPU execution.

Feature/action extraction was completed for 7 conditions x 2 domains x 225 paired frames using fixed policy:

```text
oftplus_h5_vision
checkpoint step 28560
instruction: Pick up the orange cube.
```

## 4. Preprocessing Attribution

Baseline `P0_current_paired_image` Action Gap: `0.370203`.

Best preprocessing condition:

```text
P4_letterbox_224
Action Gap: 0.101993
Action Gap reduction: 72.45%
Hidden Gap reduction: 32.08%
Vision Gap reduction: 23.42%
```

Center-crop conditions also reduced action gap substantially:

- `P2_center_crop_square_resize_224`: action reduction `59.70%`
- `P3_center_crop_0p875_resize_224`: action reduction `51.24%`

Direct resize `P1` barely changed Action Gap.

## 5. Photometric Attribution

Photometric image-stat matching did not improve policy output.

- `P5_brightness_match_sim_to_real`: observation MSE improved to `0.026690`, but Action Gap worsened by `1.78%`.
- `P6_contrast_match_sim_to_real`: luma std alignment improved, but Action Gap worsened by `0.36%`.

This is direct evidence that reducing observation-level photometric distance does not necessarily improve action consistency.

## 6. Camera Attribution

Not executed in this Phase6 run. Camera attribution remains `PLANNED` and requires controlled render/camera perturbation.

## 7. Appearance / Geometry Sensitivity

Not executed. No Real physical geometry alignment claim is made.

## 8. Policy-Relevant Representation Analysis

The best policy-impact condition `P4_letterbox_224` also reduced hidden gap by `32.08%`, but the strongest evidence remains final Action Gap.

Sensitive/null energy is not populated because the Phase5 sensitive basis was not serialized into this Phase6 output. Therefore this Phase uses hidden gap plus final action gap as the verified policy-relevance evidence.

## 9. Representation Alignment vs Policy Alignment

Important contrast:

```text
P5 brightness:
  Observation MSE best: 0.026690
  Action Gap: 0.376782
  Action change: -1.78%

P4 letterbox:
  Observation MSE: 0.048943
  Action Gap: 0.101993
  Action reduction: 72.45%
```

Thus, the best observation alignment is not the best policy alignment.

## 10. Action-Dimension Analysis

Action Gap is dominated by gripper gap in this dataset. `P4_letterbox_224` reduced gripper gap from `0.315337` to `0.120110`. Translation/rotation gaps are small in all conditions.

## 11. Phase Analysis

Phase-conditioned metrics were generated in:

```text
06_policy_relevance/phase_conditioned_metrics.csv
figures/phase_conditioned_action_gap.png
```

## 12. Combined Environment Alignment

Not executed. The next valid combined test is `P4` plus the best future camera/appearance factor, if such a factor is verified.

## 13. Environment Alignment x Hidden Correction

Not executed. Candidate next 2x2 test:

```text
Original vs P4 letterbox
No hidden correction vs progress/phase hidden correction
```

## 14. Held-Out Evaluation

Not executed. Current Phase6 conditions are deterministic factor interventions evaluated on all 225 pairs. No parameter was selected by held-out folds in this run.

## 15. Failure Analysis

Frame-level metrics are saved in `policy_sensitive_attribution_frame_metrics.csv`; worsened/improved frame analysis can be performed from this file.

## 16. Final Evidence Chain

Verified chain for evaluated preprocessing/photometric factors:

```text
Preprocessing / photometric intervention
    -> Observation metric change
    -> Vision / projector / hidden feature change
    -> Final action gap change
```

The central Phase6 finding is that observation-level photometric improvement alone did not improve policy action consistency, while image geometry/preprocessing changes, especially letterbox/crop, strongly affected hidden/action gap.

## 17. Limitations

- No real robot performance claim.
- No ROS proprio policy claim.
- Camera, appearance, geometry, combined, and hidden-correction 2x2 experiments remain planned.
- Sensitive/null energy requires serializing or reconstructing the Phase5 sensitive basis.
- P4 is a controlled preprocessing sensitivity result, not automatically a deployment recommendation.

## 18. Next Deployment Experiment

Run the 2x2 comparison:

```text
Original Sim vs P4 letterbox Sim
No hidden correction vs progress/phase hidden correction
```

and then evaluate whether P4 and hidden correction are complementary.
