# Phase 8 Observation Gap Report

## Method

Metrics were computed on `pair_valid=true` rows from `03_pair_alignment/aligned_pairs.csv`.
Images were compared in RGB after resizing the right image only if dimensions differed.

## Comparison Summary

| Comparison | N | L1 | RMSE | SSIM | Brightness diff | Contrast diff | Saturation diff | Edge L1 | Shift x | Shift y |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| real4_vs_real8 | 45 | 0.086176 | 0.136291 | 0.807108 | -0.031188 | -0.015306 | -0.004158 | 0.013262 | 0.329376 | 0.362626 |
| real8_vs_real9 | 45 | 0.016755 | 0.037598 | 0.927505 | -0.005639 | -0.001902 | -0.005132 | 0.006688 | 0.061439 | -0.049738 |
| real9_vs_real10 | 45 | 0.026796 | 0.065851 | 0.890415 | 0.021359 | -0.013425 | -0.009696 | 0.010119 | 0.128476 | -0.085489 |
| sim4_vs_real10 | 45 | 0.261253 | 0.317681 | 0.690216 | 0.252372 | -0.002867 | 0.128625 | 0.018745 | -5.759631 | -6.750994 |
| sim4_vs_real4 | 45 | 0.261196 | 0.307319 | 0.726507 | 0.267840 | 0.027766 | 0.147610 | 0.013719 | -9.238376 | -1.035555 |
| sim4_vs_real8 | 45 | 0.244911 | 0.294727 | 0.724735 | 0.236653 | 0.012460 | 0.143452 | 0.015951 | 4.892630 | -9.861212 |
| sim4_vs_real9 | 45 | 0.242501 | 0.292551 | 0.724733 | 0.231013 | 0.010558 | 0.138320 | 0.016151 | 7.543328 | -8.984864 |

## Interpretation Guardrails

- These are observation-level image metrics only.
- A larger image gap is not automatically a larger policy-relevant gap.
- The next step must compare representation/action-facing features before making policy-impact claims.

## Phase Summary

See `phase_summary.csv` for phase-conditioned metrics.
