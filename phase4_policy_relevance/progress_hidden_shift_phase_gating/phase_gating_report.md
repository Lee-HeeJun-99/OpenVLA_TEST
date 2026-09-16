# Progress Hidden Shift Phase Gating

Experiment:
Offline phase-gated use of existing LOO progress hidden correction.

Purpose:
Check whether disabling correction in phases where raw action gap is already tiny reduces correction-induced worsening.

Method:
Apply hidden correction only for `alignment`, `descent_to_grasp`, and `hold`; keep raw action for `grasp_close` and `lift`. Also compute oracle min(raw, hidden) upper bound.

Result:
| Method | Gap to Sim chunk mean L2 | Aggregate reduction ratio | Improved | Worsened |
|---|---:|---:|---:|---:|
| raw_no_correction | 0.370262 | 0.000000 | 0 | 0 |
| progress_hidden_all | 0.072325 | 0.804665 | 190 | 35 |
| phase_gated_alignment_descent_hold | 0.072086 | 0.805310 | 134 | 1 |
| oracle_min_raw_hidden | 0.071470 | 0.806974 | 190 | 0 |

Interpretation:
Phase gating preserves almost all of the progress-hidden correction benefit while removing most over-correction failures. This supports a failure-aware correction gate rather than unconditional hidden alignment.

Status:
VERIFIED offline Level-2 gating analysis.

Limitation:
Uses planner phase labels; deployment would need a reliable phase/progress estimator or a policy-internal gate.
