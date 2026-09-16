# Action-Hidden Token-Level Sensitivity

Experiment:
Apply the Real→Sim Δh of one `action_hidden_states.input` token at a time and measure the action-head effect.

Purpose:
Identify whether policy-relevant Real/Sim hidden differences are concentrated in specific action-hidden tokens/components.

Hypothesis:
Some tokens will reduce Action Gap more than others, and token norm or representation-distance reduction alone will not fully explain policy impact.

Input:
- 225 verified Real/Sim pairs.
- Policy: `oftplus_h5_vision`, checkpoint step 28560.
- Layer: `action_hidden_states.input`, token count 35.

Method:
For each frame and token `t`, evaluate `h_real` with only token `t` replaced by `h_real[t] + (h_sim[t] - h_real[t])`. All other tokens remain Real.

Control:
The control is the full no-correction raw action gap and comparison across all same-layer tokens. This does not assume token index maps directly to a physical image region.

Metrics:
Action gap reduction, remaining chunk mean Action Gap, first-action translation/rotation/gripper gaps, representation gap reduction, token norm fraction.

Result:
Top 10 tokens by mean Action Gap reduction:
| Token | Mean action gap reduction | Remaining chunk mean gap | Repr reduction ratio | Token norm fraction | Improved | Worsened |
|---:|---:|---:|---:|---:|---:|---:|
| 4 | 0.018756 | 0.351506 | 0.030759 | 0.236746 | 181 | 44 |
| 3 | 0.016227 | 0.354035 | 0.027732 | 0.227874 | 186 | 39 |
| 1 | 0.015362 | 0.354900 | 0.041584 | 0.283266 | 188 | 37 |
| 9 | 0.013345 | 0.356917 | 0.019132 | 0.193710 | 186 | 39 |
| 10 | 0.012712 | 0.357550 | 0.030414 | 0.242860 | 200 | 25 |
| 5 | 0.011473 | 0.358789 | 0.015046 | 0.171628 | 170 | 55 |
| 0 | 0.011363 | 0.358899 | 0.059609 | 0.332852 | 214 | 11 |
| 11 | 0.011091 | 0.359171 | 0.026336 | 0.225437 | 199 | 26 |
| 12 | 0.010859 | 0.359403 | 0.018921 | 0.190170 | 196 | 29 |
| 2 | 0.010855 | 0.359407 | 0.022948 | 0.210126 | 184 | 41 |

Interpretation:
This is token-level policy-relevance evidence at the action-facing hidden layer. A token with high action effect but modest representation reduction is more policy-relevant than a token that only explains large latent distance.

Status:
VERIFIED offline Level-2 token/component analysis for the existing `oftplus_h5_vision` 225-pair dataset.

Limitation:
Token index semantics are not mapped to physical image regions. This is action-hidden token relevance, not direct visual patch attribution.

Next decision:
Use top tokens/components for targeted correction ablation and compare against whole-hidden progress-conditioned correction.
