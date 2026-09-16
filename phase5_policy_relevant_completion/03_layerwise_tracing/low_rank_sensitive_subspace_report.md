# LOO Low-Rank Policy-Sensitive Subspace

[Purpose]
Evaluate whether a small train-estimated policy-sensitive subspace can approximate the Action Gap reduction of full hidden correction.

[Hypothesis]
A low-rank basis built from train local sensitive directions should reduce held-out Action Gap more efficiently than random low-rank bases and possibly more efficiently than generic delta PCA.

[Inputs]
- 225 verified Real/Sim pairs.
- `action_hidden_states.input`
- `oftplus_h5_vision`, checkpoint step 28560.

[Checked]
- train-fold sensitive-projection PCA basis
- train-fold full-delta PCA basis
- random basis control
- rank sweep: `[1, 2, 4, 8, 16, 32, 64, 128]`

[Results]
| Subspace | k | Gap to Sim chunk mean L2 | Repr reduction ratio | Improved | Worsened |
|---|---:|---:|---:|---:|---:|
| delta_basis | 1 | 0.279946 | 0.063986 | 152 | 73 |
| delta_basis | 2 | 0.282293 | 0.087424 | 155 | 70 |
| delta_basis | 4 | 0.227003 | 0.120557 | 170 | 55 |
| delta_basis | 8 | 0.171879 | 0.161085 | 193 | 32 |
| delta_basis | 16 | 0.141591 | 0.187604 | 202 | 23 |
| delta_basis | 32 | 0.124229 | 0.208585 | 205 | 20 |
| delta_basis | 64 | 0.109817 | 0.226655 | 209 | 16 |
| delta_basis | 128 | 0.088365 | 0.244976 | 217 | 8 |
| random_basis | 1 | 0.370247 | 0.000004 | 101 | 124 |
| random_basis | 2 | 0.370236 | 0.000006 | 102 | 123 |
| random_basis | 4 | 0.370239 | 0.000015 | 103 | 122 |
| random_basis | 8 | 0.370233 | 0.000022 | 104 | 121 |
| random_basis | 16 | 0.370218 | 0.000053 | 106 | 119 |
| random_basis | 32 | 0.370195 | 0.000112 | 111 | 114 |
| random_basis | 64 | 0.370100 | 0.000214 | 123 | 102 |
| random_basis | 128 | 0.369996 | 0.000443 | 129 | 96 |
| sensitive_basis | 1 | 0.151052 | 0.010212 | 158 | 67 |
| sensitive_basis | 2 | 0.058514 | 0.014090 | 154 | 71 |
| sensitive_basis | 4 | 0.053546 | 0.015490 | 163 | 62 |
| sensitive_basis | 8 | 0.049779 | 0.020216 | 169 | 56 |
| sensitive_basis | 16 | 0.014663 | 0.024465 | 223 | 2 |
| sensitive_basis | 32 | 0.009351 | 0.026433 | 225 | 0 |
| sensitive_basis | 64 | 0.008616 | 0.028187 | 225 | 0 |
| sensitive_basis | 128 | 0.006676 | 0.030669 | 225 | 0 |

[Status]
VERIFIED offline Level-2 low-rank subspace analysis.

[Problems]
- Test correction projects held-out Real→Sim Δh, so it is not deployable as-is.
- Local sensitive vectors depend on action-head gradient and paired Sim action during analysis.
- Five episode LOO only.

[Decision]
Use this result to judge whether policy-sensitive correction can be compressed into a low-rank basis.

[Next]
Proceed to projector/vision upstream tracing.
