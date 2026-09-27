# Phase 8 Data Integrity Report

## Scope

Validated baseline and changed-condition episodes before distribution analysis.

Real episodes: `episode_000004`, `episode_000008`, `episode_000009`, `episode_000010`.

Sim replay episodes checked for file availability and image/frame count.

## Summary

| Domain | Episode | Condition | Steps | Actions | Images | Metadata steps | Hz | Status |
|---|---:|---|---:|---:|---:|---:|---:|---|
| real | episode_000004 | baseline_real_episode4 | 45 | 45 | 45 | 45 | 4.999134387364149 | VALID |
| real | episode_000008 | lighting_changed | 45 | 45 | 45 | 45 | 5.000733286500157 | VALID |
| real | episode_000009 | non_target_cube_layout_changed | 45 | 45 | 45 | 45 | 5.002721927035042 | VALID |
| real | episode_000010 | extra_object_added_on_episode9_condition | 45 | 45 | 45 | 45 | 5.001503542374126 | VALID |
| sim | episode_000004 | baseline_sim_episode4 | 0 | 0 | 45 | 45 | 4.999134387364149 | VALID |
| sim | episode_000008 | lighting_changed | 0 | 0 | 45 | 45 | 5.000733286500157 | VALID |
| sim | episode_000009 | non_target_cube_layout_changed | 0 | 0 | 45 | 45 | 5.002721927035042 | VALID |
| sim | episode_000010 | extra_object_added_on_episode9_condition | 0 | 0 | 45 | 45 | 5.001503542374126 | VALID |

## Findings

- Real episode4/8/9/10 each contain 45 steps, 45 action rows, and 45 primary images.
- Real `steps_with_actions.jsonl` action vectors are 7D for all checked rows.
- Real timestamps are monotonic for all checked episodes.
- Metadata reports zero skipped images and zero duplicate images for all checked real episodes.
- `pose_source` is `planned_commanded_pose`; measured robot tracking error is not available from these files.

## Items Requiring Attention

- None from this file-level integrity pass.

## Interpretation Limits

- This report validates file-level integrity only.
- It does not prove physical pose equivalence beyond planned/commanded trajectory metadata.
- Shadow Mode and closed-loop rollout logs were not analyzed in this step.
