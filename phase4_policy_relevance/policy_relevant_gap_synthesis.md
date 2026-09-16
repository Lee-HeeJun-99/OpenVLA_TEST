# Policy-Relevant Representation Gap Synthesis

Experiment:
Synthesis of policy-relevant representation analyses at `action_hidden_states.input`.

Purpose:
Summarize which Real/Sim representation differences affected the `oftplus_h5_vision` action head, and decide the next research direction.

Hypothesis:
Action Gap is driven by selected action-sensitive directions/components, not by total representation distance alone.

Input:
- 225 verified Real/Sim pairs, 5 episodes.
- Policy: `oftplus_h5_vision`, checkpoint step 28560.
- Main layer: `action_hidden_states.input`.
- Existing LOO progress hidden correction result.
- New direction perturbation, sensitive/null decomposition, token sensitivity, top-k token correction, and LOO top-k token correction results.

Method:
Compare correction/perturbation methods by remaining Action Gap, Action Gap reduction, and representation reduction where available.

Result:
| Method | Held-out? | Gap to Sim chunk mean L2 | Aggregate action reduction | Representation reduction | Interpretation |
|---|---|---:|---:|---:|---|
| no correction | yes | 0.370245 | 0.000000 | 0.000000 | baseline |
| full Δh Real→Sim | no | 0.000664 | 0.998206 | 1.000000 | sanity check, not deployable |
| LOO progress hidden shift | yes | 0.072325 | 0.804663 | not summarized here | strongest existing deployability-like offline correction |
| sensitive projection | no | 0.136015 | 0.632635 | 0.009250 | strong evidence that small policy-sensitive component matters |
| null residual | no | 0.274003 | 0.259942 | 0.884700 | large representation reduction with weaker action effect |
| neg-gradient same-norm | no | 0.099080 | 0.732395 | -0.329098 | local action-sensitive direction, not a stable correction |
| LOO top10 token correction | yes | 0.253949 | 0.314106 | 0.349017 | token targeting helps but is weak/diffuse |
| LOO top20 token correction | yes | 0.157898 | 0.573532 | 0.515030 | many tokens needed for strong effect |
| LOO bottom10 token correction | yes | 0.271100 | 0.267783 | 0.087323 | top ranking only modestly better than bottom10 |

Interpretation:
The clearest policy-relevance evidence is the contrast between `sensitive_projection` and `null_residual`:

- `sensitive_projection` reduced representation gap by only `0.009250` but reduced Action Gap by `0.632635`.
- `null_residual` reduced representation gap by `0.884700` but reduced Action Gap by only `0.259942`.

This directly supports:

`Representation Gap magnitude != Policy relevance`

Token-level results show that action effect is not concentrated in a tiny number of token indices. Top-k token correction improves Action Gap, and the ranking is stable under episode holdout, but top10 is only modestly better than bottom10. This suggests token-index targeting is less precise than direction-based or progress-conditioned hidden correction.

Status:
VERIFIED offline Level-2 synthesis for the existing `oftplus_h5_vision` 225-pair dataset.

Limitation:
- All results are offline action-level evidence.
- No Real robot performance claim.
- Full Δh and sensitive projection use paired Sim hidden states, so they are not deployable as-is.
- Token index is not physical-region attribution.
- Environment causality remains UNVERIFIED.

Decision:
Prioritize gradient/progress-conditioned hidden correction over token-index-only correction.

Next:
1. Analyze where LOO progress hidden correction still fails or worsens frames.
2. Compare failure frames by phase/progress/action dimension.
3. Use failure analysis to design a more targeted policy-sensitive correction, rather than aligning the entire hidden distribution.
