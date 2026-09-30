# Real episode and trajectory inventory

Status: read-only source audit. No model inference, robot operation, ROS operation, source episode edit, deletion, or overwrite was performed. Only the three requested inventory outputs were created.

## Count summary

| Metric | Count |
|---|---:|
| Canonical complete Real episodes | 10 |
| Unique complete Real trajectories | 10 |
| Exact SHA-256/frame-count duplicate rows | 0 |
| Same-ID partial/derived Real paths | 2 |
| Canonical episodes with extractable Reference Command | 10 |
| Phase 10 dual-model Reference-relative Action Gap available | 1 |
| Reference-relative comparison possible with at least one existing model output | 4 |
| Already analyzed by Phase 10 | 1 |
| Not analyzed by Phase 10 | 9 |

## Canonical Real trajectories

| Episode | Condition | Path | Frames | Actions | Reference extract | OpenVLA P10 | OFT P10 | OFT P8 | Phase10 Action Gap |
|---|---|---|---:|---:|---|---|---|---|---|
| episode_000001 | unknown | `/home/ubuntu/a0509_vla_linux_field_bundle_20260903/data/real_world/raw_dataset_oft/episodes/episode_000001` | 46 | 46 | yes | no | no | no | no |
| episode_000002 | unknown | `/home/ubuntu/a0509_vla_linux_field_bundle_20260903/data/real_world/raw_dataset_oft/episodes/episode_000002` | 47 | 47 | yes | no | no | no | no |
| episode_000003 | unknown | `/home/ubuntu/a0509_vla_linux_field_bundle_20260903/data/real_world/raw_dataset_oft/episodes/episode_000003` | 44 | 44 | yes | no | no | no | no |
| episode_000004 | baseline | `/home/ubuntu/a0509_vla_linux_field_bundle_20260903/data/real_world/raw_dataset_oft/episodes/episode_000004` | 45 | 45 | yes | yes | yes | yes | yes |
| episode_000005 | unknown | `/home/ubuntu/a0509_vla_linux_field_bundle_20260903/data/real_world/raw_dataset_oft/episodes/episode_000005` | 43 | 43 | yes | no | no | no | no |
| episode_000006 | unknown | `/home/ubuntu/a0509_vla_linux_field_bundle_20260903/data/real_world/raw_dataset_oft/episodes/episode_000006` | 44 | 44 | yes | no | no | no | no |
| episode_000007 | unknown | `/home/ubuntu/a0509_vla_linux_field_bundle_20260903/data/real_world/raw_dataset_oft/episodes/episode_000007` | 48 | 48 | yes | no | no | no | no |
| episode_000008 | lighting | `/home/ubuntu/a0509_vla_linux_field_bundle_20260903/data/real_world/raw_dataset_oft/episodes/episode_000008` | 45 | 45 | yes | no | no | yes | no |
| episode_000009 | distractor | `/home/ubuntu/a0509_vla_linux_field_bundle_20260903/data/real_world/raw_dataset_oft/episodes/episode_000009` | 45 | 45 | yes | no | no | yes | no |
| episode_000010 | extra_object | `/home/ubuntu/a0509_vla_linux_field_bundle_20260903/data/real_world/raw_dataset_oft/episodes/episode_000010` | 45 | 45 | yes | no | no | yes | no |

All ten canonical Real directories contain images, `steps.jsonl`, `steps_with_actions.jsonl`, metadata, instruction, joint/state fields, EE/TCP pose, planner phase, canonical action, and route poses in metadata. These are recorded scripted Reference Command trajectories, not measured ground-truth policy trajectories.

## Same-ID and derived-path findings

- `episode_000007.backup_20260920_223002` is a distinct partial 28-frame recording with no `steps_with_actions.jsonl`. Its frame hashes differ from canonical episode 7, so it is not an exact duplicate.
- `outputs/sim2real_analysis/dedup_preprocessed/real/episode_000001` is a 31-frame derived near-duplicate-filtered subset whose manifest refers to a 65-frame source sequence. It does not match the current canonical 46-frame episode 1 and is not counted as another complete trajectory.
- The ten `raw_dataset_oft_episode_*_home_relative_joint_replay_images` directories are Sim replay artifacts corresponding to Real episodes 1–10. Matching frame counts do not make them Real duplicates; their image SHA-256 values differ.
- No two inventoried rows had both the same frame count and identical aggregate image SHA-256. Exact duplicate count is therefore zero under the stated rule.

## Additional `trajectories_10` path

`/home/ubuntu/second_a0509_vla_runs/shadow_mode_30rollouts/trajectories_10` contains ten successful **Sim planner trajectories**, not ten additional Real trajectories. Each has `episode.json`, `steps.jsonl`, primary images, per-step Sim observation, control target pose/phase and a 7-D action. Frame/action counts are 100, 98, 113, 108, 127, 106, 115, 88, 102 and 101. None has `steps_with_actions.jsonl`, OpenVLA prediction or OFT prediction. They are inventoried as Sim reference candidates requiring a schema adapter; they do not change the count of ten canonical Real trajectories.

## Why only Episode 4 has Phase 10 analysis

This was not caused by missing Reference Commands: all ten canonical Real episodes have `steps_with_actions.jsonl` and are extractable. It was a processing-scope limitation. Phase 10 recorded-input outputs for both OpenVLA and OFT exist only for episode 4. Phase 8 contains OFT vision outputs for episodes 4, 8, 9 and 10, but those were used for environment-conditioned offline response comparisons and must not be relabeled as Phase 10 dual-model Reference-relative results. Episodes 1, 2, 3, 5, 6 and 7 have no located model predictions in the audited Phase 8/10 outputs.

## Missing data by analysis scope

- Episodes 1–3 and 5–7: Phase 10-compatible OpenVLA and OFT predictions are absent.
- Episodes 8–10: Phase 8 OFT outputs exist, but Phase 10-compatible OpenVLA and recorded-input OFT prediction logs are absent.
- All ten canonical episodes lack measured-action ground truth; stored pose/action provenance is planned/commanded Reference Command.
- The episode 7 backup lacks `steps_with_actions.jsonl` and is partial.
- The deduplicated episode 1 artifact lacks full source frames and canonical action file, and belongs to an older/different source sequence according to its manifest.

## Recommended next targets (inventory only)

1. Episodes 8, 9, 10: known controlled Phase 8 conditions and existing OFT offline outputs make provenance/config reconciliation easiest.
2. Episodes 1, 2, 3, 5, 6, 7: complete Reference Commands exist, but both model prediction logs must be generated later under one frozen Phase 10 configuration.

No inference was launched in this audit. Before any later inference, freeze checkpoint, preprocessing, instruction normalization, gripper semantics, and K=5 expansion rules.

## Scope guardrails

- Phase 8: environment-conditioned offline OFT response gap.
- Phase 10: Reference-relative dual-model recorded-input evaluation.
- The ten rows in a phase summary are model×phase aggregates, not ten trajectories.

## Full artifact inventory

The CSV contains canonical Real episodes, same-ID partial/derived Real paths, ten Sim replay artifact roots, deduplicated derivatives, the prepare-only Sim artifact, and literal Phase 9 `episode_*` Sim frame directories, with full SHA-256 fields and availability flags.
