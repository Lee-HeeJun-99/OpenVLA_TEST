# Phase 11 paired-condition policy-sensitive analysis

## Executive result

- Status: `COMPLETED_PHASE11_POLICY_SENSITIVE_ANALYSIS`.
- Valid pairs: 15/15 across five layout/trajectory IDs and three conditions.
- Aligned samples: 684 condition–baseline pairs.
- Actual inference: OpenVLA 912 frames; OFT 192 K=5 requests (expanded to aligned target steps). Errors/fixtures: 0.
- Full feature records: 1,824; paired layer rows: 3,420.
- Robot/ROS/trajectory/Home/gripper commands: 0.

This is an exploratory five-pair offline study. Stored actions are Scripted Reference Commands, not measured actions or Ground Truth Actions. Results do not establish closed-loop success.

## Models and hooks

- OpenVLA: `vanilla_s1_balanced_step8130`, `openvla_token`, K=1, crop-bottom 0.0. Hooks: vision output and projector output. An action-hidden hook was not available; projector output is explicitly treated as an upstream proxy.
- OFT: `oft_mixed480_step28560_merged`, `oftplus_h5_vision`, K=5, center crop, no proprio. Hooks: vision output, projector output, action-hidden input and action-head output.

## Main findings

Mean training-stat standardized condition-induced action shift:

| Model | Distractor swap | Extra object | Lighting low |
|---|---:|---:|---:|
| OpenVLA | 0.791 | **1.143** | 0.950 |
| OFT | 0.336 | **0.767** | 0.748 |

Episode-level paired-bootstrap 95% intervals were respectively OpenVLA: [0.578,0.994], [0.807,1.385], [0.784,1.121]; OFT: [0.232,0.468], [0.636,0.938], [0.600,0.924]. With only five episode IDs these intervals describe repeatability in this sample, not population guarantees.

`extra_object` has the largest normalized total action shift for both models. `distractor_swap` has the smallest. `lighting_low` is the clearest gripper-specific case: OFT produced 116 false-close aligned frames, including 77/84 alignment frames and 30 descent frames. OpenVLA lighting produced 22 false-close frames, 21 during descent. No false-open event occurred in these paired samples.

Upstream visual representation shift is largest or near-largest for lighting. OFT action-hidden relative shift is slightly larger for extra-object (0.564) than lighting (0.544), matching the distinction between visual change and downstream policy impact.

## Policy-sensitive low-rank result

The train-selected basis used Episodes 1–4 only; Episode 5 was held out. OFT action-hidden train-selected validation association reached Spearman 0.470 at k=1. OpenVLA projector-proxy association reached 0.301 with effective rank 7. A random projection was better in the sampled best run for both models, so there is no evidence that the current train-selected subspace robustly dominates random projection.

The covariance-selected action-associated space has rank at most seven because the paired action target is 7-D. Requested k values above seven therefore saturate; they are retained in the sweep only to document this limit. Oracle-validation rows are labeled as upper-bound diagnostics and are not used for selection.

## Phase interpretation

Large shifts concentrate in hold/alignment/descent rather than only after grasp. For OpenVLA, extra-object is strongest in hold and alignment. For OFT, extra-object is strongest in alignment, while lighting produces large hold/alignment/descent changes and the dominant early false-close pattern.

## Rollout hypotheses—not outcomes

- LOW_POLICY_IMPACT: `distractor_swap`.
- HIGH_POLICY_IMPACT: `extra_object`.
- PHASE_SPECIFIC_IMPACT: `lighting_low`, testing approach/descent gripper timing.

Use paired layouts and record reach/alignment/grasp/lift failure, close timing, minimum cube–gripper distance, timeout and hold. No robot rollout was executed here.

## Limitations

- Only five paired episode IDs; frames are not treated as independent trials.
- OpenVLA action-hidden representation is unavailable.
- Phase-progress alignment interpolates pairs when raw frame counts differ.
- The low-rank result is not robustly superior to random projection.
- Offline agreement and sensitivity cannot be equated with real closed-loop success.
