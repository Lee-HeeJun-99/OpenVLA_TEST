# Action-Hidden Sensitive / Null-Like Decomposition

Experiment:
Gradient-based decomposition of Real→Sim hidden-state difference at `action_hidden_states.input`.

Purpose:
Check whether Action Gap reduction is tied to action-sensitive components rather than total representation-distance reduction.

Hypothesis:
The full Real→Sim Δh should reduce Action Gap. If only part of Δh is policy-sensitive, a gradient-aligned component should reduce Action Gap more efficiently than null-like or random components, even when representation-gap reduction is smaller.

Input:
- 225 verified Real/Sim pairs.
- Policy: `oftplus_h5_vision`, checkpoint step 28560.
- Layer: `action_hidden_states.input`.

Method:
For each frame, compute the gradient of `0.5 * mean((Action(h_real) - Action_sim)^2)` with respect to `h_real`. Then evaluate several hidden corrections through the same action head.

Control:
- `random_same_norm`: random direction with same norm as full Δh.
- `orthogonal_to_gradient_same_norm`: random direction orthogonal to the local action-gap gradient.
- `null_residual`: component of Δh after removing its projection onto the negative gradient direction.

Metrics:
Action gap to Sim action, hidden shift norm, representation gap reduction ratio, aggregate action gap reduction ratio, phase summaries, and action-dimension breakdown.

Result:
The aggregate action reduction ratio below uses `(raw mean gap - method mean gap) / raw mean gap`. This is preferred for interpretation because per-frame ratio means can be distorted by frames whose raw action gap is very small.

| Method | Gap to Sim chunk mean L2 | Aggregate action reduction | Repr reduction ratio | Improved | Worsened |
|---|---:|---:|---:|---:|---:|
| full_delta | 0.000664 | 0.998206 | 1.000000 | 225 | 0 |
| neg_gradient_same_norm | 0.099080 | 0.732395 | -0.329098 | 132 | 93 |
| no_correction | 0.370245 | 0.000000 | 0.000000 | 101 | 124 |
| null_residual | 0.274003 | 0.259942 | 0.884700 | 149 | 76 |
| orthogonal_to_gradient_same_norm | 0.364942 | 0.014323 | -0.414297 | 143 | 82 |
| random_same_norm | 0.365349 | 0.013223 | -0.414196 | 139 | 86 |
| sensitive_projection | 0.136015 | 0.632635 | 0.009250 | 218 | 7 |

Key contrast:
- `sensitive_projection` reduced representation gap by only `0.009250` on average but reduced aggregate Action Gap by `0.632635`.
- `null_residual` reduced representation gap by `0.884700` on average but reduced aggregate Action Gap by only `0.259942`.
- Same-norm random and gradient-orthogonal controls barely reduced Action Gap.

Interpretation:
The contrast between `sensitive_projection` and `null_residual` supports the policy-relevance hypothesis: reducing the whole representation gap is not the same as reducing the action-relevant part of the gap. A small gradient-aligned component can have much larger action impact than a much larger null-like residual component.

Status:
VERIFIED offline Level-2 analysis for the existing `oftplus_h5_vision` 225-pair dataset.

Limitation:
The gradient direction is local to the action head at `h_real`; it is not a full causal decomposition of upstream vision tokens or environment factors. Null-like residual is defined relative to a first-order gradient direction, not the exact nonlinear nullspace.

Next decision:
Use this result to decide whether to expand to token/component-level sensitivity or correction ablation with progress-conditioned learned shifts.

Summary JSON:
`/home/ubuntu/a0509_vla_linux_field_bundle_20260903/lhj/phase4_policy_relevance/action_hidden_sensitive_null_decomposition/sensitive_null_decomposition_summary.json`
