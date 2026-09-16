# LOO Correction Robustness Ablation

[Purpose]
Test whether progress-conditioned hidden correction is genuinely stronger than simple/global/random/shuffled/wrong-direction controls under leave-one-episode-out estimation.

[Hypothesis]
Progress-conditioned correction should reduce held-out Action Gap more than global mean, random matched-norm, shuffled progress, or wrong-direction controls.

[Inputs]
- 225 verified Real/Sim pairs.
- Feature: `action_hidden_states.input`.
- Policy: `oftplus_h5_vision`, checkpoint step 28560.

[Checked]
- C0 no correction
- C1 global mean hidden shift
- C2 progress-conditioned hidden shift
- C3 phase-conditioned shift
- C4 progress+phase conditioned shift
- C5 random matched-norm shift
- C6 shuffled progress shift
- C7 wrong-direction/sign-flipped shift

[Results]
| Method | Gap to Sim chunk mean L2 | Aggregate action reduction | Repr reduction ratio | Improved | Worsened |
|---|---:|---:|---:|---:|---:|
| no_correction | 0.370245 | 0.000000 | 0.000000 | 101 | 124 |
| global_mean | 0.125657 | 0.660611 | 0.083778 | 136 | 89 |
| progress | 0.072325 | 0.804656 | 0.184831 | 190 | 35 |
| phase | 0.066682 | 0.819898 | 0.168540 | 200 | 25 |
| progress_phase | 0.061829 | 0.833006 | 0.171036 | 198 | 27 |
| random_matched_norm | 0.367535 | 0.007319 | -0.207271 | 129 | 96 |
| shuffled_progress | 0.130975 | 0.646246 | 0.074655 | 136 | 89 |
| wrong_direction | 0.485313 | -0.310788 | -0.496956 | 7 | 218 |

Phase summary for structured methods:
| Phase | Method | Gap | Improved | Worsened |
|---|---|---:|---:|---:|
| alignment | phase | 0.095436 | 85 | 0 |
| alignment | progress | 0.086198 | 85 | 0 |
| alignment | progress_phase | 0.077256 | 85 | 0 |
| descent_to_grasp | phase | 0.158635 | 39 | 1 |
| descent_to_grasp | progress | 0.207231 | 39 | 1 |
| descent_to_grasp | progress_phase | 0.170335 | 38 | 2 |
| grasp_close | phase | 0.002666 | 41 | 9 |
| grasp_close | progress | 0.005604 | 31 | 19 |
| grasp_close | progress_phase | 0.002370 | 42 | 8 |
| hold | phase | 0.029545 | 10 | 0 |
| hold | progress | 0.025754 | 10 | 0 |
| hold | progress_phase | 0.029545 | 10 | 0 |
| lift | phase | 0.002931 | 25 | 15 |
| lift | progress | 0.002984 | 25 | 15 |
| lift | progress_phase | 0.002933 | 23 | 17 |

[Status]
VERIFIED offline Level-2 correction robustness analysis.

[Problems]
- Only 5 episode folds.
- LOO episode split is not unseen-layout generalization.
- Correction still uses train Real/Sim paired calibration data.

[Decision]
Use the best robust method as the reference for Phase5 gating and deployability comparisons.

[Next]
Proceed to held-out gate generalization.
