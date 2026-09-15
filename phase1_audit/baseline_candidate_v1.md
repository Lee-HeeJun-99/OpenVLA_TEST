# Baseline Candidate V1 - Phase 1 Audit

Date: 2026-09-15 KST

This file summarizes what can currently be reused as a baseline and what still requires verification.

## Scope

Baseline candidate:

- Dataset: existing 5-episode Real/Sim paired dataset
- Frame count: 225 paired frames
- Policy/encoder used for current analysis: bundle OFT `oftplus_h5_vision`
- Checkpoint:
  - `/home/ubuntu/a0509_vla_linux_field_bundle_20260903/runtime_state/oft_mixed480_step28560_merged`
- Instruction:
  - `Pick up the orange cube.`
- Output root:
  - `/home/ubuntu/a0509_vla_linux_field_bundle_20260903/lhj`

## Status Matrix

| Item | Status | Evidence | Impact | Decision |
|---|---|---|---|---|
| 5-episode Real image/step counts | VERIFIED | `episode_consistency_summary.json` | High | Use 225 frames as current exploratory baseline. |
| 5-episode visual environment consistency | VERIFIED visually | `real_sim_episode_sample_contact_sheet.jpg` | High | Treat as one visual condition; do not claim measured geometry identity. |
| 5-episode Real commanded metadata consistency | VERIFIED | `real_episode_metadata_summary.json` | High | File-level instruction/camera/workspace/start/velocity conditions are consistent. |
| Real physical geometry/table history | UNVERIFIED | No definitive measurement timeline found | High | Do not use old Z/table offsets as ground truth. |
| Real measured feedback trajectory | UNVERIFIED | metadata has `feedback_pose_samples=0` | High | Do not claim measured TCP tracking accuracy from planned pose metadata alone. |
| Sim scene actually used by replay | VERIFIED | replay logs, `sim_replay_runtime_evidence.json` | High | Use listed USD/config as current Sim condition. |
| Sim replay condition consistency | VERIFIED | `sim_episode_replay_summary.json` | High | Replay method is consistent across five episodes. |
| Sim authored table/camera values | VERIFIED for authored USD values | `sim_usd_geometry_extract.json` | Medium/High | Direct authored values are usable; composed world transforms remain unchecked. |
| Real/Sim same-index pairing | VERIFIED as replay source-step correspondence | `pair_correspondence_summary.json` | High | Valid for current paired analysis; physical TCP equivalence remains unverified. |
| Real TCP vs Sim EEF direct comparison | INVALID until frame audit | naive L2 around `0.813 m` | High | Do not interpret direct EEF/TCP numbers as mismatch without frame transform audit. |
| Real effective dataset Hz | VERIFIED | episode timestamps around 5 Hz | Medium | Use 5 Hz as dataset sampling condition. |
| Sim replay rate | VERIFIED in logs/config | `joint_replay_rate_hz=5.0` | Medium | Use as replay condition. |
| Feature extraction tensor shapes | VERIFIED | feature manifests / saved NPZ | Medium | Use for layer-wise analysis. |
| Observation/token gap metrics | VERIFIED for existing pairs | `phase1_observation_token_gap` | Medium | Interpret as mixed geometry/photometric/state gap, not pure photometric gap. |
| Representation gap metrics | VERIFIED for existing pairs | `outputs/token_distribution_analysis/5_episodes/metrics` | Medium | Valid conditional on pairing/preprocessing. |
| In-sample progress correction | VERIFIED implementation; not generalization | previous outputs | Medium | Existing result only; not final evidence. |
| LOO representation correction | VERIFIED | `loo_real_to_sim_progress_shift` | Medium/High | Level 1 evidence: held-out episode latent gap decreases. |
| LOO offline action correction | VERIFIED | `policy_relevant_progress_shift` | High | Level 2 evidence: held-out episode final action gap decreases offline. |
| Real robot performance improvement | UNVERIFIED | No closed-loop corrected rollout | High | No performance claim allowed. |
| Bundle policy identity | VERIFIED | `runtime_preflight.json`, `a0509_training_config.json` | High | Current analysis is for bundle `oftplus_h5_vision`. |
| Current `/robot_ws` real runtime policy identity | VERIFIED at config level | `/home/ubuntu/robot_ws/src/openvla_doosan_runtime/config/runtime_oft.yaml` | High | Current ROS config points to `oftplus_h5_proprio` and different checkpoint. Not equivalent to bundle analysis. |
| Offline analysis vs current ROS deployment equivalence | INVALID / RE-RUN REQUIRED | variant/checkpoint/preprocess mismatch | High | Do not claim current ROS deployment behavior from current 5-episode analysis. |

## Policy Identity Finding

Bundle analysis policy:

- variant: `oftplus_h5_vision`
- use_film: `true`
- use_proprio: `false`
- action chunk K: `5`
- bounded gripper: `true`
- center crop in bundle offline runtime: `true`

Evidence:

- `runtime_state/oft_mixed480_step28560_merged/a0509_training_config.json`
- `validation/evidence/oft/runtime_preflight.json`
- feature manifest responses report:
  - `variant: oftplus_h5_vision`

Current ROS real runtime config:

- file:
  - `/home/ubuntu/robot_ws/src/openvla_doosan_runtime/config/runtime_oft.yaml`
- variant:
  - `oftplus_h5_proprio`
- model path:
  - `/home/ubuntu/robot_ws/src/openvla/runs/oftplus_h5_proprio_bounded_oft200_9000--6000_chkpt`
- center crop enabled:
  - `false`
- requires proprio:
  - `true`

Decision:

- The existing 5-episode LHJ analysis is a valid bundle-policy offline study.
- It is not yet a verified baseline for the current ROS real-runtime policy.
- Before real deployment claims, either:
  - run the same bundle `oftplus_h5_vision` policy in the real runtime, or
  - rerun Real/Sim feature/action analysis using the current `oftplus_h5_proprio` policy and identical preprocessing.

## Current Evidence Level

LEVEL 1 - Representation:

- LOO progress shift reduces vision/projector latent discrepancy.
- Status: VERIFIED for existing 5 episodes.

LEVEL 2 - Offline Action:

- LOO hidden-state progress shift reduces final unnormalized action discrepancy.
- Raw action chunk mean L2:
  - `0.370262`
- Hidden-shift action chunk mean L2:
  - `0.072325`
- Relative reduction:
  - `80.47%`
- Status: VERIFIED for existing 5 episodes.

LEVEL 3 - Real Performance:

- Not tested.
- Status: UNVERIFIED.

## Next Required Work

1. Decide which policy is the real target for deployment:
   - bundle `oftplus_h5_vision`, or
   - current ROS `oftplus_h5_proprio`.
2. If using current ROS policy, regenerate feature/action metrics using that policy and its proprio/preprocessing path.
3. Continue frame/camera/geometry audit before interpreting observation gap causes.
4. Keep the current 5-episode result as policy-relevance proof-of-method, not final deployment evidence.
