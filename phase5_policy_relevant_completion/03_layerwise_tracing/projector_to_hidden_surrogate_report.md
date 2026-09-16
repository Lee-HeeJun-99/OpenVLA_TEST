# Projector→Hidden Surrogate Tracing

[Purpose]
Trace whether upstream `projector.output` Real/Sim differences can predict policy-sensitive corrections at `action_hidden_states.input`.

[Hypothesis]
If policy-relevant hidden differences are already encoded in projector deltas, a train-fold linear map from pooled projector delta to sensitive hidden delta should reduce held-out Action Gap.

[Inputs]
- 225 verified Real/Sim pairs.
- `projector.output`
- `action_hidden_states.input`
- `oftplus_h5_vision`, checkpoint step 28560.

[Checked]
- LOO ridge map: pooled projector delta -> sensitive hidden delta.
- Ridge values: `[0.001, 0.01, 0.1, 1.0, 10.0]`.
- Evaluation by action-head output after applying predicted hidden correction.

[Results]
| Ridge | Gap to Sim chunk mean L2 | Cosine to sensitive hidden | Improved | Worsened |
|---:|---:|---:|---:|---:|
| 0.001 | 0.145573 | 0.710075 | 171 | 54 |
| 0.01 | 0.145573 | 0.710157 | 171 | 54 |
| 0.1 | 0.145569 | 0.710962 | 171 | 54 |
| 1.0 | 0.145546 | 0.717666 | 171 | 54 |
| 10.0 | 0.145533 | 0.742800 | 178 | 47 |

[Status]
VERIFIED offline surrogate tracing.

[Problems]
- This is not exact full-forward projector injection.
- Projector delta is pooled over 256 tokens, so token-level upstream attribution is not resolved.
- Uses train paired data to learn mapping and held-out delta for projection/evaluation.

[Decision]
Use as evidence for whether projector differences carry action-sensitive hidden structure; do not claim causal full-forward injection.

[Next]
If surrogate is positive, expand to token/component mapping; otherwise prioritize hidden-level correction and deployable gating.
