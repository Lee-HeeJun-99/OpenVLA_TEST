# Phase 6 Environment Attribution Summary

## 1. Research Question

Which controllable Real-Sim observation/environment factors produce policy-relevant representation differences that affect action prediction?

## 2. Existing Verified Evidence

Phase 5 showed that reducing the whole representation gap is not sufficient; action-hidden policy-sensitive structure is what matters for Action Gap. Phase 6 therefore evaluates environment/preprocessing factors through final `oftplus_h5_vision` action output, not through visual similarity alone.

## 3. Full-Forward Feasibility Audit

Status: `VERIFIED_FULL_FORWARD` after local GPU execution.

Feature/action extraction was completed for preprocessing/photometric conditions and image-space camera sensitivity conditions using fixed policy:

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

Executed image-space camera sensitivity ablation. This is not calibrated Real camera alignment; it is controlled camera-like perturbation of Sim images.

Best camera condition by Action Gap:

```text
C2_sim_shift_right_24px
Action Gap: 0.335682
Action Gap reduction: 9.33%
Hidden Gap reduction: 2.81%
Vision Gap reduction: -1.06%
```

Important directional contrast:

```text
C1 shift left:
  Observation MSE: 0.085792
  Action Gap: 0.382553
  Action reduction: -3.34%

C2 shift right:
  Observation MSE: 0.095760
  Action Gap: 0.335682
  Action reduction: 9.33%
```

The left shift slightly improves observation MSE but worsens Action Gap. The right shift worsens observation MSE but improves Action Gap. This again supports the Phase6 claim that pixel/statistical alignment is not sufficient to predict policy impact.

Policy-sensitive projection using the Phase6 P0 sensitive basis adds a stronger policy-relevance proxy:

```text
C0 sensitive energy: 67.585359
C2 sensitive energy: 62.328616
C2 sensitive energy reduction: 7.78%
C2 action reduction: 9.33%
```

The best camera condition by Action Gap is also the best by policy-sensitive energy reduction. Conditions that worsen Action Gap generally increase the sensitive-energy proxy. This is still an image-space camera sensitivity result, not calibrated Real camera alignment.

## 7. Appearance / Geometry Sensitivity

Not executed. No Real physical geometry alignment claim is made.

## 8. Policy-Relevant Representation Analysis

The best policy-impact condition `P4_letterbox_224` also reduced hidden gap by `32.08%`, but the strongest evidence remains final Action Gap.

Sensitive/null energy is populated for P0/P4 correction variants and camera sensitivity variants using the Phase6 P0-derived sensitive basis. It is a policy-relevance proxy, not a causal neuron identity claim.

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

Executed controlled combined image-space condition `P4_letterbox_224 + C2_sim_shift_right_24px`.

```text
K0_P4_letterbox:
  Action Gap: 0.101274
  Hidden Gap: 187.747536
  Sensitive Energy: 29.077619

K1_P4_letterbox_C2_shift_right_24px:
  Action Gap: 0.130148
  Hidden Gap: 200.902644
  Sensitive Energy: 37.908585
```

Adding the C2 right-shift on top of P4 worsened Action Gap by `28.51%` relative to K0, increased hidden gap by `7.01%`, and increased sensitive energy by `30.37%`. Therefore the single-factor C2 improvement on the original images is not additive with P4. This supports condition-dependent environment effects: a factor can improve policy consistency in one preprocessing context and hurt it in another.

This remains a controlled image-space sensitivity combination, not calibrated physical camera alignment.

## 13. Environment Alignment x Hidden Correction

Executed offline 2x2 using action-head hidden injection.

```text
Original P0 raw Action Gap: 0.370203
Original P0 + condition-specific progress/phase hidden correction: 0.061464

P4 letterbox raw Action Gap: 0.101993
P4 letterbox + condition-specific progress/phase hidden correction: 0.072575
P4 letterbox + P0-trained correction transfer: 0.090531
```

Interpretation:

- Hidden correction is strongly effective on Original P0.
- P4 letterbox already removes much of the Action Gap; condition-specific hidden correction further improves it from `0.101993` to `0.072575`.
- P0-trained correction transfers weakly to P4: Action Gap improves to `0.090531`, but representation gap increases (`repr_gap_reduction_ratio = -0.387`).
- This supports complementarity, but also shows correction calibration is environment-condition dependent.

## 13b. Policy-Sensitive Energy Analysis

A P0-derived k=128 action-sensitive subspace was reconstructed and saved:

```text
06_policy_relevance/sensitive_energy/phase6_p0_sensitive_basis_k128.npz
```

Mean sensitive energy:

```text
P0 raw: 67.585359
P0 + correction: 36.784414

P4 raw: 29.082477
P4 + P4 correction: 21.531851
P4 + P0 correction transfer: 56.653547
```

Interpretation:

- P4 raw reduces P0-derived sensitive energy substantially: `67.585 -> 29.082`.
- P4-specific hidden correction further reduces sensitive energy: `29.082 -> 21.532`.
- P0-trained correction transferred to P4 increases sensitive energy to `56.654`, matching the weaker action improvement and the negative representation-reduction result.

This strengthens the Phase6 interpretation: the useful environment/correction changes are those that reduce action-sensitive residual structure, not merely global observation or representation distance.

## 14. Held-Out Evaluation

Executed leave-one-episode-out condition selection. Each fold selects a condition using train episodes only and evaluates on the held-out episode.

```text
Train action-min selection:
  selected P4 in 5/5 folds
  held-out action gap: 0.102457
  held-out action reduction: 72.65%

Train hidden-min selection:
  selected P4 in 5/5 folds
  held-out action gap: 0.102457

Train observation-MSE-min selection:
  selected P5 brightness in 5/5 folds
  held-out action gap: 0.378114
  held-out action change: -1.85%
```

This strengthens the conclusion: selecting by observation MSE chooses the photometric condition that worsens held-out policy action, while selecting by policy/hidden metrics consistently chooses P4.

## 15. Failure Analysis

Frame-level metrics are saved in `policy_sensitive_attribution_frame_metrics.csv`, `camera_frame_metrics.csv`, and `03_camera/sensitive_energy/camera_sensitive_energy_frame_metrics.csv`. Current key failure pattern: observation-MSE-selected `P5_brightness_match_sim_to_real` worsens Action Gap despite improving image statistics, and several camera perturbations increase policy-sensitive energy while worsening Action Gap.

## 16. Final Evidence Chain

Verified chain for evaluated preprocessing, photometric, and image-space camera sensitivity factors:

```text
Preprocessing / photometric intervention
    -> Observation metric change
    -> Vision / projector / hidden feature change
    -> Final action gap change
```

The central Phase6 finding is that observation-level photometric or pixel-statistical improvement alone did not improve policy action consistency. Image geometry/preprocessing changes, especially letterbox/crop, strongly affected hidden/action gap. For camera-like perturbations, the best action condition also reduced P0-derived policy-sensitive energy, while an observation-MSE-improving left shift worsened Action Gap.

## 17. Limitations

- No real robot performance claim.
- No ROS proprio policy claim.
- Camera analysis is image-space sensitivity, not calibrated camera extrinsic/intrinsic alignment.
- Appearance/geometry physical attribution was not executed because verified Real physical geometry/calibration is not available in this offline phase.
- Combined `P4 + C2` was executed and worsened Action Gap relative to P4 alone; this indicates non-additive, condition-dependent effects.
- Sensitive/null energy uses a P0-derived k=128 basis and should be interpreted as a policy-relevance proxy, not a causal neuron identity.
- P4 is a controlled preprocessing sensitivity result, not automatically a deployment recommendation.

## 18. Next Deployment Experiment

Next valid experiment:

```text
1. Do not combine P4 and C2 by default; combined full-forward worsened Action Gap relative to P4 alone.
2. If deployment relevance is needed, audit actual ROS preprocessing and run a separate proprio-policy track with synchronized proprio.
3. For physical environment attribution, first obtain calibrated Real camera/geometry measurements; otherwise report camera/geometry only as sensitivity experiments.
4. Before real robot use, validate whether the chosen preprocessing/correction can be integrated into the runtime path without changing policy semantics unexpectedly.
```
