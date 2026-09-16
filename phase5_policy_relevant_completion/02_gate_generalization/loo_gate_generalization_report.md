# LOO Gate Generalization

[Purpose]
Validate whether phase/progress gates selected only on train episodes generalize to held-out episodes.

[Hypothesis]
Train-selected gates should reduce over-correction relative to unconditional progress correction without using held-out episode labels for selection.

[Inputs]
- `/home/ubuntu/a0509_vla_linux_field_bundle_20260903/lhj/phase5_policy_relevant_completion/01_correction_robustness/loo_correction_ablation_frame_metrics.csv`

[Checked]
- unconditional progress correction
- train-selected phase gate
- progress threshold selected by train min-gap objective
- progress threshold selected by zero-worsening train objective
- progress threshold selected by worsened<=1 train objective
- progress threshold selected by mean + lambda*worsened_rate, lambda=0.1
- oracle min(raw, progress) upper bound

[Results]
| Gate method | Gap to Sim chunk mean L2 | Aggregate reduction | Improved | Worsened |
|---|---:|---:|---:|---:|
| oracle_min_raw_progress | 0.071488 | 0.806917 | 192 | 0 |
| phase_gate_train | 0.072086 | 0.805303 | 161 | 14 |
| progress_all | 0.072325 | 0.804656 | 192 | 33 |
| progress_gate_lambda | 0.073059 | 0.802673 | 124 | 2 |
| progress_gate_min_gap | 0.072577 | 0.803976 | 155 | 31 |
| progress_gate_worse_le_1 | 0.072957 | 0.802950 | 125 | 2 |
| progress_gate_zero_worse | 0.081698 | 0.779339 | 112 | 1 |
| raw_no_correction | 0.370245 | 0.000000 | 0 | 0 |

Fold excerpt:
| Heldout | Gate method | Gap | Worsened |
|---|---|---:|---:|
| episode_000001 | oracle_min_raw_progress | 0.067461 | 0 |
| episode_000001 | phase_gate_train | 0.067718 | 0 |
| episode_000001 | progress_all | 0.067559 | 2 |
| episode_000001 | progress_gate_zero_worse | 0.087556 | 0 |
| episode_000002 | oracle_min_raw_progress | 0.052098 | 0 |
| episode_000002 | phase_gate_train | 0.053972 | 1 |
| episode_000002 | progress_all | 0.053680 | 1 |
| episode_000002 | progress_gate_zero_worse | 0.055051 | 1 |
| episode_000003 | oracle_min_raw_progress | 0.093844 | 0 |
| episode_000003 | phase_gate_train | 0.094084 | 3 |
| episode_000003 | progress_all | 0.094340 | 7 |
| episode_000003 | progress_gate_zero_worse | 0.101873 | 0 |
| episode_000004 | oracle_min_raw_progress | 0.034234 | 0 |
| episode_000004 | phase_gate_train | 0.034510 | 2 |
| episode_000004 | progress_all | 0.035089 | 5 |
| episode_000004 | progress_gate_zero_worse | 0.050806 | 0 |
| episode_000005 | oracle_min_raw_progress | 0.113100 | 0 |
| episode_000005 | phase_gate_train | 0.113370 | 8 |
| episode_000005 | progress_all | 0.114245 | 18 |
| episode_000005 | progress_gate_zero_worse | 0.116244 | 0 |

Fold-selected gate config:
```json
[
  {
    "heldout_episode": "episode_000001",
    "thresholds": {
      "progress_gate_min_gap": 26,
      "progress_gate_zero_worse": 21,
      "progress_gate_worse_le_1": 24,
      "progress_gate_lambda": 24
    },
    "phases_on": [
      "alignment",
      "descent_to_grasp",
      "hold",
      "lift"
    ]
  },
  {
    "heldout_episode": "episode_000002",
    "thresholds": {
      "progress_gate_min_gap": 26,
      "progress_gate_zero_worse": 24,
      "progress_gate_worse_le_1": 25,
      "progress_gate_lambda": 24
    },
    "phases_on": [
      "alignment",
      "descent_to_grasp",
      "hold",
      "lift"
    ]
  },
  {
    "heldout_episode": "episode_000003",
    "thresholds": {
      "progress_gate_min_gap": 46,
      "progress_gate_zero_worse": 21,
      "progress_gate_worse_le_1": 24,
      "progress_gate_lambda": 24
    },
    "phases_on": [
      "alignment",
      "descent_to_grasp",
      "hold",
      "lift"
    ]
  },
  {
    "heldout_episode": "episode_000004",
    "thresholds": {
      "progress_gate_min_gap": 46,
      "progress_gate_zero_worse": 21,
      "progress_gate_worse_le_1": 24,
      "progress_gate_lambda": 24
    },
    "phases_on": [
      "alignment",
      "descent_to_grasp",
      "hold",
      "lift"
    ]
  },
  {
    "heldout_episode": "episode_000005",
    "thresholds": {
      "progress_gate_min_gap": 46,
      "progress_gate_zero_worse": 21,
      "progress_gate_worse_le_1": 25,
      "progress_gate_lambda": 25
    },
    "phases_on": [
      "alignment",
      "descent_to_grasp",
      "hold",
      "lift"
    ]
  }
]
```

[Status]
VERIFIED offline Level-2 gate generalization.

[Problems]
- Gate selection still uses planner-derived progress/phase metadata.
- Episode LOO is not unseen-layout generalization.

[Decision]
Use the best train-selected gate as the deployability-oriented baseline for Phase5.

[Next]
Proceed to low-rank policy-sensitive subspace experiment.
