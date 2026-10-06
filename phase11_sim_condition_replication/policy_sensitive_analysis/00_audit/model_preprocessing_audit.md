# Model and preprocessing audit

## OpenVLA

- Checkpoint: `models/vanilla_s1_balanced_step8130`
- Variant: `openvla_token`; horizon K=1; action dimension 7.
- Dataset key: `a0509_sim_cube_pick`.
- Recorded health evidence from the prior local run: crop-bottom fraction 0.0 and merged adapter.
- Current server state: unavailable at `127.0.0.1:8766`.
- Dataset statistics mask includes all seven components. Gripper is binary closedness in this checkpoint's statistics.

## OFT

- Checkpoint: `runtime_state/oft_mixed480_step28560_merged`.
- Variant: `oftplus_h5_vision`; horizon K=5; action dimension 7; proprio disabled.
- Dataset key: `a0509_sim_cube_pick`.
- Server default enables center crop unless `--no-center-crop` is supplied.
- Bounded sigmoid gripper: continuous 0=open, 1=closed.
- Dataset-statistics affine mask is `[true,true,true,true,true,true,false]`; gripper is not affine-de-normalized.
- Current server state: unavailable at `127.0.0.1:8765`.

The checkpoints, preprocessing, action statistics and gripper contracts are model-specific and must not be merged. Model output, de-normalized output, canonical output and postprocessed output must remain separate; executed action is always null in this offline study.

## Feature hooks

Reusable extraction code exists at `sim2real_analysis/04_features/extract_vla_features.py`; Phase 8 verified OFT hooks including vision backbone, projector, `action_hidden_states.input`, and action-head output. Cross-model raw hidden L2 is prohibited where dimensions/semantics differ. The current GPU driver is unavailable, so no Phase 11 feature hook was executed.
