# Phase 8 Distribution Analysis Summary

## Real Sequential Condition Chain

| Comparison | Obs L1 | Vision Cos | Projector Cos | Hidden Cos | Action Output L2 |
|---|---:|---:|---:|---:|---:|
| real4_vs_real8 | 0.086176 | 0.091892 | 0.067887 | 0.214049 | 1.067581 |
| real8_vs_real9 | 0.016755 | 0.022556 | 0.020605 | 0.099563 | 0.422997 |
| real9_vs_real10 | 0.026796 | 0.058087 | 0.043349 | 0.145395 | 0.521422 |

## Fixed Sim Reference

| Comparison | Obs L1 | Vision Cos | Projector Cos | Hidden Cos | Action Output Cos |
|---|---:|---:|---:|---:|---:|
| sim4_vs_real4 | 0.261196 | 0.151481 | 0.109822 | 0.225261 | 0.066402 |
| sim4_vs_real8 | 0.244911 | 0.188875 | 0.144493 | 0.232378 | 0.014986 |
| sim4_vs_real9 | 0.242501 | 0.201537 | 0.160912 | 0.237687 | 0.044616 |
| sim4_vs_real10 | 0.261253 | 0.245028 | 0.198877 | 0.257501 | 0.136367 |

## Interpretation

- In the real sequential chain, lighting change (`real4_vs_real8`) is largest across observation, visual representation, hidden representation, and action output metrics.
- Non-target cube relocation (`real8_vs_real9`) is smallest across these metrics.
- Extra object insertion (`real9_vs_real10`) is intermediate, but closer to lighting than cube relocation in some representation metrics.
- Fixed sim reference comparisons show the changed real conditions increasingly move away from sim4 in representation, especially `sim4_vs_real10`.
- These are offline representation/action-output gaps; they are not closed-loop rollout success metrics.
