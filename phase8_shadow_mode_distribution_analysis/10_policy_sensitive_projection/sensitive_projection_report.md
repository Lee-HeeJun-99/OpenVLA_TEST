# Phase 8 Policy-Sensitive Projection

## Purpose

This analysis projects Phase 8 `action_hidden_states.input` differences onto the Phase 6 P0-derived policy-sensitive low-rank basis (`k=128`).

It tests whether each environment change moves hidden states mostly in the previously identified policy-sensitive subspace or mostly in the residual/null component.

## Important Scope

- Basis source: Phase 6 `phase6_p0_sensitive_basis_k128.npz`.
- Dataset: Phase 8 episodes 4, 8, 9, 10 plus fixed sim episode 4 reference.
- Policy: `oftplus_h5_vision`, checkpoint step 28560.
- This is an offline projection analysis. It is not a new causal intervention or rollout result.

## Real-Only Overall Summary

| Comparison | N | Total | Sensitive | Null | Ratio | Action L2 | Gripper |
| --- | --- | --- | --- | --- | --- | --- | --- |
| real4_vs_real8 | 45.000000 | 278.651235 | 52.419522 | 273.490833 | 0.188988 | 0.291046 | 0.290410 |
| real8_vs_real9 | 45.000000 | 179.167126 | 23.036442 | 177.579253 | 0.126637 | 0.027105 | 0.025872 |
| real9_vs_real10 | 45.000000 | 232.338600 | 33.178598 | 229.744587 | 0.138497 | 0.027705 | 0.025659 |

## Real-Only Phase Summary

| Comparison | Phase | N | Ratio | Sensitive | Action L2 | Gripper |
| --- | --- | --- | --- | --- | --- | --- |
| real4_vs_real8 | alignment | 17.000000 | 0.199466 | 54.343131 | 0.587041 | 0.586867 |
| real4_vs_real8 | descent_to_grasp | 8.000000 | 0.187229 | 57.160937 | 0.223785 | 0.223286 |
| real4_vs_real8 | grasp_close | 10.000000 | 0.169993 | 49.111044 | 0.008843 | 0.008203 |
| real4_vs_real8 | hold | 2.000000 | 0.196971 | 45.914704 | 0.604313 | 0.604260 |
| real4_vs_real8 | lift | 8.000000 | 0.190231 | 49.352237 | 0.003752 | 0.001855 |
| real8_vs_real9 | alignment | 17.000000 | 0.110296 | 24.151717 | 0.038261 | 0.036051 |
| real8_vs_real9 | descent_to_grasp | 8.000000 | 0.123145 | 21.717847 | 0.047927 | 0.047125 |
| real8_vs_real9 | grasp_close | 10.000000 | 0.143800 | 22.253338 | 0.005045 | 0.004844 |
| real8_vs_real9 | hold | 2.000000 | 0.089596 | 12.511462 | 0.059507 | 0.059448 |
| real8_vs_real9 | lift | 8.000000 | 0.152660 | 25.595202 | 0.002051 | 0.000879 |
| real9_vs_real10 | alignment | 17.000000 | 0.133058 | 40.428010 | 0.033528 | 0.029765 |
| real9_vs_real10 | descent_to_grasp | 8.000000 | 0.131882 | 30.117889 | 0.065496 | 0.064177 |
| real9_vs_real10 | grasp_close | 10.000000 | 0.134424 | 26.231511 | 0.005015 | 0.004687 |
| real9_vs_real10 | hold | 2.000000 | 0.088747 | 15.273960 | 0.037614 | 0.037158 |
| real9_vs_real10 | lift | 8.000000 | 0.174200 | 33.994323 | 0.003426 | 0.001758 |

## Interpretation

The key quantity is `hidden_sensitive_ratio`, the fraction of hidden delta norm captured by the Phase 6 policy-sensitive subspace.

If a condition has high total hidden energy but low sensitive ratio, it may be visually/representationally different while not strongly aligned with the previously identified action-sensitive structure.

If a condition has high sensitive energy or ratio and also high action/gripper gap, it is a stronger candidate for a policy-relevant environment perturbation.

## Limitations

- The basis was built from the previous 5-episode P0 Real-Sim setting, not from Phase 8 real-only perturbations.
- Projection onto this basis is compatibility evidence, not proof that the same hidden directions causally drive the new action gaps.
- Because Phase 8 action gap is gripper-dominated, a future gripper-specific sensitive basis may be more diagnostic.
