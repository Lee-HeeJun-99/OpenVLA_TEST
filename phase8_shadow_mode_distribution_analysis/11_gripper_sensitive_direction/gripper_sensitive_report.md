# Phase 8 Gripper-Specific Sensitive Direction

## Purpose

This analysis computes a local hidden-space gradient using only the gripper component of the action chunk. It then measures how much of each Phase 8 hidden delta lies along that local gripper-sensitive direction.

## Method

For each aligned pair:

1. Use the left hidden state as the gradient point.
2. Use the right policy action chunk as the target.
3. Compute loss only on action dimension 6, the gripper output.
4. Compute the gradient of gripper loss with respect to `action_hidden_states.input`.
5. Project the observed hidden delta onto the gripper gradient direction.

Positive `cos(delta,-grad)` means the observed hidden delta points in the local direction that should reduce the gripper mismatch to the right-side action.

## Runtime

- Action head checkpoint: `/home/ubuntu/a0509_vla_linux_field_bundle_20260903/runtime_state/oft_mixed480_step28560_merged/action_head--28560_checkpoint.pt`
- Device: `cpu`
- Policy context: `oftplus_h5_vision`, checkpoint step 28560.

## Real-Only Overall Summary

| Comparison | N | Grip-sensitive | Ratio | cos(delta,-grad) | Grip loss | Action L2 | Grip gap |
| --- | --- | --- | --- | --- | --- | --- | --- |
| real4_vs_real8 | 45.000000 | 21.025672 | 0.076338 | 0.076338 | 0.089201 | 0.291046 | 0.290410 |
| real8_vs_real9 | 45.000000 | 4.314261 | 0.024398 | 0.024001 | 0.001279 | 0.027105 | 0.025872 |
| real9_vs_real10 | 45.000000 | 5.041613 | 0.021472 | 0.021472 | 0.001769 | 0.027705 | 0.025659 |

## Real-Only Phase Summary

| Comparison | Phase | N | Ratio | cos(delta,-grad) | Action L2 | Grip gap |
| --- | --- | --- | --- | --- | --- | --- |
| real4_vs_real8 | alignment | 17.000000 | 0.137391 | 0.137391 | 0.587041 | 0.586867 |
| real4_vs_real8 | descent_to_grasp | 8.000000 | 0.063411 | 0.063411 | 0.223785 | 0.223286 |
| real4_vs_real8 | grasp_close | 10.000000 | 0.021906 | 0.021906 | 0.008843 | 0.008203 |
| real4_vs_real8 | hold | 2.000000 | 0.137167 | 0.137167 | 0.604313 | 0.604260 |
| real4_vs_real8 | lift | 8.000000 | 0.012364 | 0.012364 | 0.003752 | 0.001855 |
| real8_vs_real9 | alignment | 17.000000 | 0.019715 | 0.019715 | 0.038261 | 0.036051 |
| real8_vs_real9 | descent_to_grasp | 8.000000 | 0.037244 | 0.037244 | 0.047927 | 0.047125 |
| real8_vs_real9 | grasp_close | 10.000000 | 0.028386 | 0.028386 | 0.005045 | 0.004844 |
| real8_vs_real9 | hold | 2.000000 | 0.022865 | 0.022865 | 0.059507 | 0.059448 |
| real8_vs_real9 | lift | 8.000000 | 0.016901 | 0.014672 | 0.002051 | 0.000879 |
| real9_vs_real10 | alignment | 17.000000 | 0.017296 | 0.017296 | 0.033528 | 0.029765 |
| real9_vs_real10 | descent_to_grasp | 8.000000 | 0.025787 | 0.025787 | 0.065496 | 0.064177 |
| real9_vs_real10 | grasp_close | 10.000000 | 0.032855 | 0.032855 | 0.005015 | 0.004687 |
| real9_vs_real10 | hold | 2.000000 | 0.013197 | 0.013197 | 0.037614 | 0.037158 |
| real9_vs_real10 | lift | 8.000000 | 0.013870 | 0.013870 | 0.003426 | 0.001758 |

## Interpretation

The comparison with the largest gripper-specific sensitive energy is the strongest candidate for a perturbation that changes the policy through gripper-sensitive hidden components.

This is more specific than the Phase 6 low-rank projection because it targets only the gripper output, which dominates Phase 8 action gap.

## Limitations

- This remains an offline local-gradient analysis, not a closed-loop rollout result.
- It uses right-side policy output as a disagreement target, not ground-truth correctness.
- A local gradient is a first-order approximation around the left hidden state.
