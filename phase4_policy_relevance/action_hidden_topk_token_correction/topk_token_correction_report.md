# Top-k Action-Hidden Token Correction

Experiment:
Apply Real→Sim Δh only to the most action-effective `action_hidden_states.input` tokens.

Purpose:
Test whether targeted token correction can reduce Action Gap without requiring whole-representation alignment.

Hypothesis:
Top-k action-sensitive token correction should reduce Action Gap more efficiently than bottom-k token correction and should expose a tradeoff between representation reduction and action reduction.

Input:
- 225 verified Real/Sim pairs.
- Token ranking from `/home/ubuntu/a0509_vla_linux_field_bundle_20260903/lhj/phase4_policy_relevance/action_hidden_token_sensitivity/token_sensitivity_summary_by_token.csv`.
- Ranked tokens: `[4, 3, 1, 9, 10, 5, 0, 11, 12, 2, 13, 8, 17, 16, 18, 15, 14, 6, 19, 20, 25, 21, 27, 7, 22, 24, 23, 26, 32, 33, 28, 34, 31, 29, 30]`.

Method:
For each frame, apply Δh only to selected token sets: top1/top2/top3/top5/top10/top15/top20, bottom5/bottom10, and all35 full Δh.

Metrics:
Remaining Action Gap, aggregate action reduction, representation reduction, improved/worsened frame count.

Result:
| Method | Tokens | Gap to Sim chunk mean L2 | Aggregate action reduction | Repr reduction ratio | Improved | Worsened |
|---|---:|---:|---:|---:|---:|---:|
| no_correction | 0 | 0.370245 | 0.000000 | 0.000000 | 101 | 124 |
| top1 | 1 | 0.351506 | 0.050613 | 0.030759 | 181 | 44 |
| top2 | 2 | 0.335666 | 0.093395 | 0.059612 | 191 | 34 |
| top3 | 3 | 0.324355 | 0.123946 | 0.104182 | 208 | 17 |
| top5 | 5 | 0.295366 | 0.202242 | 0.160663 | 218 | 7 |
| top10 | 10 | 0.253949 | 0.314106 | 0.349017 | 222 | 3 |
| top15 | 15 | 0.203339 | 0.450798 | 0.436359 | 224 | 1 |
| top20 | 20 | 0.156502 | 0.577300 | 0.515104 | 223 | 2 |
| bottom5 | 5 | 0.317262 | 0.143104 | 0.042929 | 167 | 58 |
| bottom10 | 10 | 0.269851 | 0.271156 | 0.088156 | 184 | 41 |
| all35_full_delta | 35 | 0.000664 | 0.998206 | 1.000000 | 225 | 0 |

Interpretation:
Top-k token correction tests deployability-relevant targeting more directly than single-token perturbation. If top-k improves Action Gap more efficiently than bottom-k at comparable or lower representation reduction, this supports selective policy-relevant correction.

Status:
VERIFIED offline Level-2 targeted-token correction.

Limitation:
Ranking is estimated and evaluated on the same 225-pair dataset, so this is not held-out generalization. Token index still has no physical-region interpretation.

Next decision:
Run held-out or fold-wise token ranking/correction if this targeted effect is strong enough; otherwise prioritize gradient/progress-conditioned correction.
