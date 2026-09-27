# Phase 8 Pair Alignment Quality Report

## Method

Pairs are aligned by identical frame/source step index because the checked episodes have identical 45-frame planned trajectories.
The file still records timing, progress, phase, gripper, and available pose/joint differences for downstream filtering.

Real-vs-real EEF and joint differences use the recorded planned-commanded convention. Sim-vs-real EEF frames are not treated as physically comparable here.

## Summary

| Comparison | Pairs | Valid | Invalid | Phase mismatches | Gripper mismatches | Mean time diff | Max real EE error m |
|---|---:|---:|---:|---:|---:|---:|---:|
| real4_vs_real8 | 45 | 45 | 0 | 0 | 0 | 0.0013621749317583938 | 0.0007674057769203046 |
| real8_vs_real9 | 45 | 45 | 0 | 0 | 0 | 0.00140660862541861 | 0.000610926137651044 |
| real9_vs_real10 | 45 | 45 | 0 | 0 | 0 | 0.001894924523205393 | 0.007999244001867675 |
| sim4_vs_real10 | 45 | 45 | 0 | 0 | 0 | 0.002016692866002106 | None |
| sim4_vs_real4 | 45 | 45 | 0 | 0 | 0 | 0.0 | None |
| sim4_vs_real8 | 45 | 45 | 0 | 0 | 0 | 0.0013621749317583938 | None |
| sim4_vs_real9 | 45 | 45 | 0 | 0 | 0 | 0.001520009151297725 | None |

## Interpretation

- These pairs are suitable for the next observation-distribution step when `pair_valid=true`.
- For sim-vs-real comparisons, image comparisons are valid as fixed-reference visual comparisons, but numeric EEF pose distance is intentionally not interpreted.
- For real-only sequential comparisons, planned pose differences are expected to be near zero if the commanded trajectory was preserved.
