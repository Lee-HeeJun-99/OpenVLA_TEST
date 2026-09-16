# Vision→Hidden Surrogate Tracing

[Purpose]
Trace whether upstream `vision_backbone.output` Real/Sim differences can predict policy-sensitive corrections at `action_hidden_states.input`.

[Hypothesis]
If policy-relevant hidden differences are already encoded in vision deltas, a train-fold linear map from pooled vision delta to sensitive hidden delta should reduce held-out Action Gap.

[Inputs]
- 225 verified Real/Sim pairs.
- `vision_backbone.output`
- `action_hidden_states.input`
- `oftplus_h5_vision`, checkpoint step 28560.

[Checked]
- LOO ridge map: pooled vision delta -> sensitive hidden delta.
- Ridge values: `[0.001, 0.01, 0.1, 1.0, 10.0]`.
- Evaluation by action-head output after applying predicted hidden correction.

[Results]
| Ridge | Gap to Sim chunk mean L2 | Cosine to sensitive hidden | Improved | Worsened |
|---:|---:|---:|---:|---:|
| 0.001 | 0.138379 | 0.754362 | 178 | 47 |
| 0.01 | 0.138378 | 0.754460 | 178 | 47 |
| 0.1 | 0.138377 | 0.755408 | 178 | 47 |
| 1.0 | 0.138359 | 0.762453 | 179 | 46 |
| 10.0 | 0.138268 | 0.781250 | 181 | 44 |

[Status]
VERIFIED offline surrogate tracing.

[Problems]
- This is not exact full-forward vision-backbone injection.
- Vision-backbone delta is pooled over 256 tokens, so token-level upstream attribution is not resolved.
- Uses train paired data to learn mapping and held-out delta for projection/evaluation.

[Decision]
Use as evidence for whether vision-backbone differences carry action-sensitive hidden structure; do not claim causal full-forward injection.

[Next]
If surrogate is positive, expand to token/component mapping; otherwise prioritize hidden-level correction and deployable gating.
