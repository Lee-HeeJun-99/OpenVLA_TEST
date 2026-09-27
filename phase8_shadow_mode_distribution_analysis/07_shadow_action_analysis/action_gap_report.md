# Phase 8 Action Gap Analysis

## Method

- Primary action source: `response.actions` from `oftplus_h5_vision` full-forward feature manifests.
- Action shape: `K=5`, `dim=7`.
- Dimensions: translation xyz, rotation rx/ry/rz, gripper.
- Binary gripper disagreement uses threshold `0.5` and should be treated as an analysis convention.

## Comparison Summary

| Comparison | Chunk mean L2 | Translation | Rotation | Gripper abs | First L2 | Gripper disagreement |
|---|---:|---:|---:|---:|---:|---:|
| real4_vs_real8 | 0.291046 | 0.004264 | 0.004268 | 0.290410 | 0.195383 | 0.346667 |
| real8_vs_real9 | 0.027105 | 0.001265 | 0.002717 | 0.025872 | 0.021042 | 0.008889 |
| real9_vs_real10 | 0.027705 | 0.001721 | 0.003520 | 0.025659 | 0.018651 | 0.008889 |
| sim4_vs_real10 | 0.035899 | 0.003060 | 0.006084 | 0.030737 | 0.049596 | 0.013333 |
| sim4_vs_real4 | 0.334297 | 0.003876 | 0.003886 | 0.333699 | 0.237745 | 0.360000 |
| sim4_vs_real8 | 0.056566 | 0.002466 | 0.003010 | 0.054921 | 0.056115 | 0.013333 |
| sim4_vs_real9 | 0.050926 | 0.002757 | 0.004325 | 0.047851 | 0.062351 | 0.022222 |

## Interpretation Guardrails

- This measures policy output disagreement between two image conditions.
- It does not say which output is correct unless planner or rollout ground truth is introduced.
- Component-level differences should be interpreted with representation and observation gaps, not alone.
