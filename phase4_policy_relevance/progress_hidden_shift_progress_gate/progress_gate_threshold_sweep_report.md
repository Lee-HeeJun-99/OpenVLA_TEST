# Progress Threshold Gate Sweep

Experiment:
Apply progress hidden correction only when `progress <= threshold`.

Purpose:
Approximate phase gating without planner phase labels.

Best by mean gap:

| threshold | gap | reduction | improved | worsened |
|---:|---:|---:|---:|---:|
| 46 | 0.072325 | 0.804665 | 190 | 35 |
| 47 | 0.072325 | 0.804665 | 190 | 35 |
| 48 | 0.072325 | 0.804665 | 190 | 35 |
| 49 | 0.072325 | 0.804665 | 190 | 35 |
| 50 | 0.072325 | 0.804665 | 190 | 35 |
| 45 | 0.072329 | 0.804654 | 189 | 35 |
| 26 | 0.072331 | 0.804649 | 131 | 4 |
| 44 | 0.072337 | 0.804632 | 187 | 35 |
| 43 | 0.072343 | 0.804617 | 184 | 35 |
| 39 | 0.072348 | 0.804603 | 173 | 27 |

Lowest-worsening candidates:

| threshold | gap | reduction | improved | worsened |
|---:|---:|---:|---:|---:|
| 21 | 0.083165 | 0.775389 | 110 | 0 |
| 20 | 0.089067 | 0.759448 | 105 | 0 |
| 19 | 0.094511 | 0.744745 | 100 | 0 |
| 18 | 0.100433 | 0.728751 | 95 | 0 |
| 17 | 0.106037 | 0.713617 | 90 | 0 |
| 16 | 0.113608 | 0.693167 | 85 | 0 |
| 15 | 0.124353 | 0.664149 | 80 | 0 |
| 14 | 0.136383 | 0.631660 | 75 | 0 |
| 13 | 0.152027 | 0.589407 | 70 | 0 |
| 12 | 0.168252 | 0.545587 | 65 | 0 |

Interpretation:
A simple progress threshold can remove over-correction failures at a modest cost to mean Action Gap reduction.

Status:
VERIFIED offline Level-2 progress-gate sweep.
