# Leave-One-Episode-Out Top-k Token Correction

Experiment:
Rank action-hidden tokens on 4 training episodes, then evaluate top-k token correction on the held-out episode.

Purpose:
Check whether token-level policy relevance survives episode-level holdout rather than only fitting the same 225-pair dataset.

Hypothesis:
LOO top-k token correction should outperform LOO bottom-k correction on held-out episodes if token ranking has generalizable policy relevance.

Input:
- Token sensitivity frame metrics: `/home/ubuntu/a0509_vla_linux_field_bundle_20260903/lhj/phase4_policy_relevance/action_hidden_token_sensitivity/token_sensitivity_frame_metrics.csv`.
- 225 verified Real/Sim pairs.
- Policy: `oftplus_h5_vision`, checkpoint step 28560.

Method:
For each held-out episode, rank tokens by mean action-gap reduction on the other four episodes. Apply top5/top10/top15/top20 and bottom10 correction on the held-out episode only.

Result:
| Method | Tokens | Gap to Sim chunk mean L2 | Aggregate action reduction | Repr reduction ratio | Improved | Worsened |
|---|---:|---:|---:|---:|---:|---:|
| no_correction | 0 | 0.370245 | 0.000000 | 0.000000 | 101 | 124 |
| loo_top5 | 5 | 0.295366 | 0.202242 | 0.160663 | 218 | 7 |
| loo_top10 | 10 | 0.253949 | 0.314106 | 0.349017 | 222 | 3 |
| loo_top15 | 15 | 0.203516 | 0.450319 | 0.434959 | 224 | 1 |
| loo_top20 | 20 | 0.157898 | 0.573532 | 0.515030 | 223 | 2 |
| loo_bottom10 | 10 | 0.271100 | 0.267783 | 0.087323 | 188 | 37 |
| all35_full_delta | 35 | 0.000664 | 0.998206 | 1.000000 | 225 | 0 |

Fold check:
| Heldout episode | Method | Gap to Sim chunk mean L2 | Repr reduction ratio |
|---|---|---:|---:|
| episode_000001 | loo_bottom10 | 0.228127 | 0.096611 |
| episode_000001 | loo_top10 | 0.219332 | 0.333247 |
| episode_000002 | loo_bottom10 | 0.231480 | 0.091691 |
| episode_000002 | loo_top10 | 0.245569 | 0.346245 |
| episode_000003 | loo_bottom10 | 0.339243 | 0.083485 |
| episode_000003 | loo_top10 | 0.280316 | 0.340465 |
| episode_000004 | loo_bottom10 | 0.221503 | 0.083189 |
| episode_000004 | loo_top10 | 0.240384 | 0.368731 |
| episode_000005 | loo_bottom10 | 0.342551 | 0.080868 |
| episode_000005 | loo_top10 | 0.287354 | 0.357039 |

Token rankings:
```json
{
  "episode_000001": [
    4,
    3,
    1,
    9,
    10,
    0,
    2,
    5,
    11,
    12,
    13,
    8,
    17,
    16,
    18,
    15,
    14,
    6,
    19,
    20,
    21,
    27,
    7,
    25,
    22,
    24,
    23,
    26,
    32,
    28,
    33,
    34,
    29,
    31,
    30
  ],
  "episode_000002": [
    4,
    3,
    1,
    10,
    9,
    5,
    2,
    0,
    11,
    12,
    13,
    8,
    17,
    16,
    15,
    14,
    6,
    18,
    19,
    20,
    7,
    21,
    25,
    24,
    22,
    27,
    23,
    26,
    28,
    32,
    34,
    33,
    31,
    29,
    30
  ],
  "episode_000003": [
    4,
    3,
    1,
    9,
    10,
    5,
    12,
    11,
    0,
    2,
    13,
    17,
    16,
    8,
    18,
    15,
    14,
    19,
    25,
    6,
    21,
    20,
    27,
    22,
    24,
    23,
    26,
    32,
    7,
    33,
    34,
    28,
    31,
    29,
    30
  ],
  "episode_000004": [
    4,
    3,
    1,
    9,
    10,
    5,
    0,
    11,
    2,
    12,
    13,
    8,
    17,
    18,
    16,
    6,
    15,
    14,
    19,
    20,
    7,
    25,
    27,
    21,
    22,
    24,
    26,
    23,
    32,
    33,
    34,
    28,
    29,
    31,
    30
  ],
  "episode_000005": [
    4,
    3,
    1,
    9,
    10,
    12,
    11,
    5,
    0,
    2,
    13,
    17,
    8,
    16,
    18,
    15,
    14,
    19,
    6,
    20,
    25,
    21,
    27,
    24,
    22,
    7,
    23,
    26,
    32,
    33,
    34,
    28,
    31,
    29,
    30
  ]
}
```

Interpretation:
This is stricter than within-dataset top-k correction because test episode rankings do not use test episode token effects. If top-k only weakly outperforms bottom-k, token-index targeting is less robust than gradient/progress-conditioned hidden correction.

Status:
VERIFIED offline Level-2 held-out token-correction analysis.

Limitation:
Episode-level holdout has only five folds and still uses the same task family/checkpoint. Token indices are not physical-region attributions.

Next decision:
Use LOO result to decide whether token-level targeted correction is worth expanding, or whether correction should target gradient/progress-sensitive hidden directions instead.
