# Progress Hidden Shift Failure Analysis

Experiment:
Failure analysis of existing leave-one-episode-out progress-conditioned hidden correction.

Purpose:
Identify where the strongest existing offline correction still fails or worsens Action Gap.

Hypothesis:
Remaining/worsened frames will be structured by phase/progress/action dimension rather than uniformly random.

Input:
- `/home/ubuntu/a0509_vla_linux_field_bundle_20260903/lhj/phase1_action_gap/policy_relevant_progress_shift/policy_relevant_progress_shift_frame_metrics.csv`
- Phase labels from `/home/ubuntu/a0509_vla_linux_field_bundle_20260903/outputs/token_distribution_analysis/5_episodes/episode_phase_summary/frame_metrics_enriched.csv`

Method:
Compute per-frame `raw_chunk_mean_l2 - hidden_shift_chunk_mean_l2`, then summarize by phase, progress, episode, and action dimensions.

Result:
- Raw chunk mean gap: `0.370262`
- Corrected chunk mean gap: `0.072325`
- Mean reduction: `0.297937`
- Aggregate reduction ratio: `0.804665`
- Improved frames: `190`
- Worsened frames: `35`

Phase summary:
| Phase | Frames | Mean reduction | Corrected gap | Improved | Worsened |
|---|---:|---:|---:|---:|---:|
| alignment | 85 | 0.626689 | 0.086198 | 85 | 0 |
| descent_to_grasp | 40 | 0.180635 | 0.207231 | 39 | 1 |
| grasp_close | 50 | -0.001142 | 0.005604 | 31 | 19 |
| hold | 10 | 0.659556 | 0.025754 | 10 | 0 |
| lift | 40 | 0.000085 | 0.002984 | 25 | 15 |

Largest worsened frames:
| Frame | Phase | Raw gap | Corrected gap | Reduction | Corrected gripper gap |
|---|---|---:|---:|---:|---:|
| episode_000002_000022 | descent_to_grasp | 0.128084 | 0.202190 | -0.074106 | 0.058040 |
| episode_000004_000027 | grasp_close | 0.023375 | 0.051724 | -0.028349 | 0.189992 |
| episode_000003_000026 | grasp_close | 0.015729 | 0.029384 | -0.013655 | 0.122486 |
| episode_000005_000025 | grasp_close | 0.007025 | 0.019999 | -0.012974 | 0.056552 |
| episode_000005_000026 | grasp_close | 0.004224 | 0.012971 | -0.008747 | 0.025601 |
| episode_000004_000028 | grasp_close | 0.010147 | 0.017501 | -0.007354 | 0.054039 |
| episode_000003_000027 | grasp_close | 0.007512 | 0.012567 | -0.005055 | 0.039003 |
| episode_000005_000027 | grasp_close | 0.003077 | 0.007952 | -0.004875 | 0.002370 |
| episode_000001_000028 | grasp_close | 0.008952 | 0.013015 | -0.004064 | 0.039111 |
| episode_000005_000028 | grasp_close | 0.001966 | 0.005385 | -0.003418 | 0.002316 |
| episode_000005_000029 | grasp_close | 0.002181 | 0.004756 | -0.002575 | 0.003567 |
| episode_000005_000037 | lift | 0.001790 | 0.004185 | -0.002394 | 0.004800 |
| episode_000005_000038 | lift | 0.001930 | 0.004306 | -0.002376 | 0.005752 |
| episode_000005_000039 | lift | 0.001735 | 0.003823 | -0.002087 | 0.005912 |
| episode_000004_000029 | grasp_close | 0.005064 | 0.006900 | -0.001835 | 0.013971 |

Interpretation:
The progress-conditioned hidden correction is strong on average, but remaining failure frames should guide the next correction rather than blindly reducing representation distance.

Status:
VERIFIED offline Level-2 failure analysis.

Limitation:
This diagnoses action-head outputs only. It does not prove Real robot performance or environment causality.

Next decision:
Use phase/progress/action-dimension failure structure to design a targeted correction or failure-aware gating rule.
