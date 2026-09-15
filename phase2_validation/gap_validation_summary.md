# Gap Validation Summary

Date: 2026-09-15 KST

This document summarizes what is currently verified about Real/Sim gap and what remains unresolved. The scope is the existing 5-episode paired offline bundle dataset unless explicitly stated otherwise.

## Dataset / Scope

- Analysis root: `/home/ubuntu/a0509_vla_linux_field_bundle_20260903/outputs/token_distribution_analysis/5_episodes`
- Paired frames: `225`
- Episodes:
  - `episode_000001`: 46 frames
  - `episode_000002`: 47 frames
  - `episode_000003`: 44 frames
  - `episode_000004`: 45 frames
  - `episode_000005`: 43 frames
- Policy/checkpoint scope:
  - Bundle offline policy: `oftplus_h5_vision`
  - Checkpoint: `runtime_state/oft_mixed480_step28560_merged`
- Current ROS deployment equivalence:
  - `INVALID / RE-RUN REQUIRED`
  - Reason: current ROS runtime config uses a different policy/preprocessing path from the bundle offline policy.

## Verified Gap Measurements

### Observation / Token Gap

Evidence:

- `lhj/phase1_observation_token_gap/observation_token_gap_summary.json`

Status:

- `VERIFIED` for offline 5-episode paired image/token calculations.
- `UNVERIFIED` as a pure photometric gap because geometry/state mismatch can contribute to image metrics.

Key results:

- Brightness absolute difference mean: `61.019`
- RGB mean L2 mean: `106.193`
- Resized image MSE mean: `5687.354`
- Resized PSNR mean: `10.612`
- Resized luma SSIM global mean: `0.588806`
- Vision token L2 mean: `201.239`
- Vision token cosine distance mean: `0.340484`
- Projector token L2 mean: `27.4038`
- Projector token cosine distance mean: `0.354466`

Interpretation:

- Real/Sim observation and token distributions are clearly different in this dataset.
- The cause of the difference is not isolated yet.

### Representation Gap

Evidence:

- `lhj/phase1_action_gap/action_facing_metrics/summary.json`
- `lhj/phase1_audit/loo_real_to_sim_progress_shift/loo_summary.json`

Status:

- `VERIFIED` for offline feature extraction and metrics.
- `EXISTING/CONDITIONAL` as a final research baseline because camera/geometry equivalence is still not fully verified.

Key uncorrected representation gaps:

- `vision_backbone.output`
  - paired pooled cosine mean: `0.141749`
  - paired pooled L2 mean: `85.1580`
  - MMD: `0.273608`
- `projector.output`
  - paired pooled cosine mean: `0.092312`
  - paired pooled L2 mean: `10.2502`
  - MMD: `0.339874`
- `action_hidden_states.input`
  - paired pooled cosine mean: `0.233869`
  - paired pooled L2 mean: `27.0532`
  - MMD: `0.117994`

Leave-One-Episode-Out Real-to-Sim progress shift:

- `vision_backbone.output`
  - cosine: `0.141749 -> 0.040102`
  - L2: `85.1580 -> 46.4359`
  - MMD: `0.273608 -> 0.002791`
- `projector.output`
  - cosine: `0.092312 -> 0.018920`
  - L2: `10.2502 -> 4.6653`
  - MMD: `0.339874 -> 0.002052`

Interpretation:

- Episode-held-out progress-conditioned correction reduces representation discrepancy.
- This is representation-level evidence only unless action/policy output also improves.

### Representation Gap to Action Gap

Evidence:

- `lhj/phase1_action_gap/layerwise_policy_relevance_summary.json`
- `lhj/phase1_action_gap/repr_action_gap_summary.json`
- `lhj/phase1_action_gap/policy_relevant_progress_shift/policy_relevant_progress_shift_summary.json`

Status:

- `VERIFIED` for offline policy action outputs generated from saved features.
- `UNVERIFIED` for online runtime behavior and real-world success.

Action gap baseline:

- Raw first action L2 mean: `0.316387`
- Raw first translation L2 mean: `0.004150`
- Raw first rotation L2 mean: `0.005911`
- Raw first gripper abs mean: `0.315673`
- Raw chunk mean L2: `0.370262`

Layer-wise policy relevance:

- Strongest overall correlation found:
  - `action_head_output_pooled_l2` vs `action_chunk_mean_l2`
  - Spearman: `0.960294`
  - Pearson: `0.960058`
- Strong action-facing hidden correlation:
  - `action_hidden_states_input_pooled_cosine_distance` vs `action_chunk_mean_l2`
  - Spearman: `0.958045`
  - Pearson: `0.950559`

Interpretation:

- In this offline dataset, action-facing representation gap is strongly related to action gap.
- This supports the policy-relevant representation gap hypothesis at the offline action level.
- It does not prove causal improvement or real-world task success.

### Policy-Relevant Hidden Shift

Evidence:

- `lhj/phase1_action_gap/policy_relevant_progress_shift/policy_relevant_progress_shift_summary.json`
- `lhj/phase1_action_gap/policy_relevant_progress_shift/policy_relevant_progress_shift_frame_metrics.csv`

Status:

- `VERIFIED` for Leave-One-Episode-Out offline action-gap reduction.
- `UNVERIFIED` for unseen layout, online runtime, and real robot performance.

Key result:

- Raw final action chunk mean L2:
  - `0.370262`
- Action-hidden progress shift, action head rerun, final action unnormalized:
  - `0.072325`
- Relative reduction:
  - `80.47%`
- Raw first action L2:
  - `0.316387`
- Hidden-shift first action L2:
  - `0.051591`
- Relative reduction:
  - `83.69%`

Interpretation:

- The current evidence reaches Level 2:
  - latent correction reduces offline final action discrepancy.
- It does not reach Level 3:
  - real-world performance improvement is not verified.

## Additional Validation: No-Collapse / Over-Correction Check

Evidence:

- Script: `lhj/scripts/verify_hidden_shift_no_collapse.py`
- Output: `lhj/phase2_validation/hidden_shift_no_collapse/hidden_shift_no_collapse_summary.json`

Status:

- `VERIFIED` for hidden-state variance and pairwise structure checks on the 225-frame offline dataset.

Key paired hidden-state result:

- `action_hidden_states.input` full-tensor L2:
  - before: `277.423`
  - after: `225.517`
- full-tensor cosine distance:
  - before: `0.393221`
  - after: `0.271807`
- L2 worsened frames:
  - `8 / 225`

Variance check:

- Real feature variance mean: `0.346409`
- Sim feature variance mean: `0.360631`
- Corrected feature variance mean: `0.415532`

Pairwise structure:

- Corrected vs Real pairwise distance correlation: `0.957173`
- Corrected vs Sim pairwise distance correlation: `0.829088`
- Real vs Sim pairwise distance correlation: `0.838765`

Action result by episode:

- episode_000001 chunk L2: `0.315780 -> 0.067559`
- episode_000002 chunk L2: `0.344004 -> 0.053680`
- episode_000003 chunk L2: `0.424920 -> 0.094340`
- episode_000004 chunk L2: `0.334297 -> 0.035089`
- episode_000005 chunk L2: `0.438956 -> 0.114245`
- Worsened action frames:
  - `35 / 225`

Interpretation:

- No feature collapse was observed.
- Corrected representations preserve strong frame-to-frame structure relative to Real.
- Correction is not uniformly beneficial per frame; episode 5 has the largest number of action-worsened frames.

## Current Answer

The gap difference is partially verified, but not fully finished.

Finished:

- Observation/token gap exists in the 5-episode paired offline dataset.
- Representation gap exists at multiple layers.
- Action gap exists.
- Action-facing representation gap is strongly related to action gap.
- LOO progress-conditioned hidden correction reduces offline action gap.
- No obvious collapse from hidden correction was observed.

Not finished:

- Causal attribution of the gap to camera, geometry, lighting, control, or preprocessing.
- Verification against the current ROS deployment policy/config.
- Online runtime integration of correction.
- Real robot shadow-mode or closed-loop performance validation.
- Unseen layout/trajectory generalization beyond the existing 5 episodes.

## Decision

Use the current results as offline Level-2 evidence for the bundle policy:

`Representation Gap -> Policy-Relevant Representation Gap -> Offline Action Gap`

Do not claim:

- real-world performance improvement,
- deployment readiness,
- or that the exact cause of the gap is known.

