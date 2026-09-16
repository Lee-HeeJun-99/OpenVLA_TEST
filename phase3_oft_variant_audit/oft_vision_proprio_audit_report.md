# OFT Vision / Proprio Variant Audit

Date: 2026-09-15 KST

Scope:

- `oftplus_h5_vision`
- `oftplus_h5_proprio`
- Existing 5-episode / 225-pair offline analysis
- Current ROS runtime config

This audit is based on actual code, checkpoint files, config files, and generated feature manifests. No real robot motion was executed.

## Final Judgement

Variant architecture category:

**C. Same base OpenVLA/OFT model class, but some layer/input structure changes when proprio is enabled.**

More specifically:

- Both variants use the same OpenVLA/OFT base model class.
- Both variants use:
  - L1 regression action head
  - FiLM vision backbone
  - 5-step action chunk
  - 7D action output
- `oftplus_h5_proprio` additionally:
  - requires 7D proprio input,
  - loads a `ProprioProjector`,
  - normalizes proprio using dataset proprio statistics,
  - projects proprio to the LLM hidden dimension,
  - appends one proprio token after vision patch tokens before multimodal/action prediction.

Therefore:

- It is **not** a completely different architecture.
- It is **not** equivalent to the vision-only model.
- It is **not** safe to treat `oftplus_h5_vision` and `oftplus_h5_proprio` results as the same learned policy.

## Code Evidence

### Variant Registration

File:

- `/home/ubuntu/a0509_vla_linux_field_bundle_20260903/runtime/openvla-oft/prismatic/vla/a0509_variants.py`

Evidence:

- `A0509OFTVariant` stores:
  - `use_l1_regression`
  - `use_diffusion`
  - `use_film`
  - `use_proprio`
- `oftplus_h5_vision`:
  - `use_l1_regression=True`
  - `use_diffusion=False`
  - `use_film=True`
  - `use_proprio=False`
- `oftplus_h5_proprio`:
  - `use_l1_regression=True`
  - `use_diffusion=False`
  - `use_film=True`
  - `use_proprio=True`
- `required_components` includes `proprio_projector` only when `use_proprio=True`.

### Model / Component Creation

File:

- `/home/ubuntu/a0509_vla_linux_field_bundle_20260903/runtime/openvla-oft/experiments/robot/openvla_utils.py`

Evidence:

- `get_vla()` loads `OpenVLAForActionPrediction` and optionally applies FiLM when `cfg.use_film=True`.
- `get_action_head()` creates `L1RegressionActionHead` when `cfg.use_l1_regression=True`.
- `get_proprio_projector()` creates and loads `ProprioProjector` only for the proprio path.

### Proprio Fusion Point

File:

- `/home/ubuntu/a0509_vla_linux_field_bundle_20260903/runtime_state/oft_mixed480_step28560_merged/modeling_prismatic.py`

Evidence:

- `_process_vision_features()`:
  - vision backbone -> projector -> language embedding space.
- `_process_proprio_features()`:
  - reshapes proprio to `(batch, proprio_dim)`,
  - applies `proprio_projector`,
  - unsqueezes to one token,
  - appends it to projected vision patch embeddings.
- `predict_action()`:
  - sets `use_proprio = proprio_projector is not None and proprio is not None`,
  - appends the proprio token only in that case,
  - increments `NUM_PATCHES` by one when proprio is used.

### Proprio Projector

File:

- `/home/ubuntu/a0509_vla_linux_field_bundle_20260903/runtime/openvla-oft/prismatic/models/projectors.py`

Evidence:

- `ProprioProjector`:
  - `fc1`: `proprio_dim -> llm_dim`
  - GELU
  - `fc2`: `llm_dim -> llm_dim`

For A0509:

- `proprio_dim = 7`
- `llm_dim = 4096`

### Action Head

File:

- `/home/ubuntu/a0509_vla_linux_field_bundle_20260903/runtime/openvla-oft/prismatic/models/action_heads.py`

Evidence:

- `L1RegressionActionHead` reshapes action-token hidden states to `(batch, NUM_ACTIONS_CHUNK, ACTION_DIM * hidden_dim)`.
- With A0509 constants:
  - `NUM_ACTIONS_CHUNK = 5`
  - `ACTION_DIM = 7`
  - hidden dim = `4096`
  - action head first input dimension = `28672`

## Checkpoint / Config Evidence

### Bundle Offline Vision Checkpoint

Path:

- `/home/ubuntu/a0509_vla_linux_field_bundle_20260903/runtime_state/oft_mixed480_step28560_merged`

Config:

- `variant`: `oftplus_h5_vision`
- `use_l1_regression`: `true`
- `use_diffusion`: `false`
- `use_film`: `true`
- `use_proprio`: `false`
- `bounded_gripper`: `true`
- `gripper_loss_weight`: `3.0`
- `action_chunk_size`: `5`
- checkpoint step: `28560`

Components:

- `action_head--28560_checkpoint.pt`
- `vision_backbone--28560_checkpoint.pt`
- no `proprio_projector` checkpoint

### ROS Runtime Proprio Checkpoint

Path:

- `/home/ubuntu/robot_ws/src/openvla/runs/oftplus_h5_proprio_bounded_oft200_9000--6000_chkpt`

Config:

- `use_l1_regression`: `true`
- `use_diffusion`: `false`
- `use_film`: `true`
- `use_proprio`: `true`
- `bounded_gripper`: `true`
- `gripper_loss_weight`: `3.0`
- `action_chunk_size`: `5`
- checkpoint step: `6000`

Components:

- `action_head--6000_checkpoint.pt`
- `vision_backbone--6000_checkpoint.pt`
- `proprio_projector--6000_checkpoint.pt`

### Parameter Shape Comparison

Evidence files:

- `component_shape_compare.csv`
- `oft_vision_proprio_audit_summary.json`

Result:

| Component | Vision Exists | Proprio Exists | Common Params | Shape Mismatch | Note |
|---|---:|---:|---:|---:|---|
| action_head | true | true | 16 | 0 | same shape |
| vision_backbone | true | true | 1311 | 0 | same shape |
| proprio_projector | false | true | 0 | 0 | only proprio checkpoint has it |
| noisy_action_projector | false | false | 0 | 0 | not used |

Interpretation:

- Shared components have matching parameter names/shapes.
- Weights are still different checkpoint files and must be treated as different learned policies.
- Proprio branch introduces additional learned parameters.

## Equivalence Judgement

| Item | Judgement | Reason |
|---|---|---|
| Architecture equivalence | PARTIAL | same base model/action head/FiLM structure, but proprio path adds projector and one token |
| Checkpoint equivalence | NO | different checkpoint paths, steps, component files, and additional proprio projector |
| Input interface equivalence | NO | vision uses image only; proprio uses image + normalized 7D state |
| Preprocessing equivalence | NO for current ROS | offline extraction uses center crop; ROS runtime config disables training preprocess and center crop |
| Action interface equivalence | PARTIAL | both output 5x7 action chunks using same dataset key, but learned weights/runtime action adapter differ |

## Existing 225-Pair Analysis Provenance

Result lineage:

`result -> feature file -> extract_vla_features.py -> serve_a0509_oft.py / OFTRuntime -> runtime_state/oft_mixed480_step28560_merged -> oftplus_h5_vision`

Evidence:

- Feature manifests:
  - `/home/ubuntu/a0509_vla_linux_field_bundle_20260903/outputs/token_distribution_analysis/5_episodes/features/real/feature_manifest.json`
  - `/home/ubuntu/a0509_vla_linux_field_bundle_20260903/outputs/token_distribution_analysis/5_episodes/features/sim/feature_manifest.json`
- Both manifests:
  - records: `225`
  - model: `oft`
  - variant in responses: `oftplus_h5_vision`
  - instruction: `Pick up the orange cube.`
  - normalized instruction: `pick up the orange cube`
  - chunk size: `5`
  - full features: `true`
- Feature extraction script:
  - `/home/ubuntu/a0509_vla_linux_field_bundle_20260903/sim2real_analysis/04_features/extract_vla_features.py`
- Runtime loaded by extraction script:
  - `/home/ubuntu/a0509_vla_linux_field_bundle_20260903/runtime/openvla-oft/vla-scripts/serve_a0509_oft.py`
- Default checkpoint in extraction script:
  - `/home/ubuntu/a0509_vla_linux_field_bundle_20260903/runtime_state/oft_mixed480_step28560_merged`
- Default variant in extraction script:
  - `oftplus_h5_vision`

Therefore the existing 225-pair results are **vision-policy offline results**, not current ROS proprio-policy deployment results.

## Existing Observation / Distribution Analysis Status

The following remain `VERIFIED` for the existing 225-pair offline vision-policy dataset:

- 225 pairs used.
- Episode counts:
  - `episode_000001`: 46
  - `episode_000002`: 47
  - `episode_000003`: 44
  - `episode_000004`: 45
  - `episode_000005`: 43
- Real/Sim image correspondence by paired manifest: `VERIFIED` at index/source-step level.
- Observation metrics: `VERIFIED`.
- Token-level statistics: `VERIFIED`.
- Vision backbone/projector distribution metrics: `VERIFIED`.
- MMD: `VERIFIED`.
- LOO representation correction: `VERIFIED`.
- LOO hidden-state correction/action-gap reduction: `VERIFIED`.
- Comprehensive offline validation numeric rerun diff: `0.0`.

Important limit:

- These results verify `oftplus_h5_vision`, not `oftplus_h5_proprio`.

## Current Research Pipeline Status

| Stage | Status | Scope / Reason |
|---|---|---|
| Environment Gap | EXISTING RESULT | Real/Sim scene/camera/geometry differences exist, but causal attribution remains unresolved |
| Observation Gap | VERIFIED | existing 225-pair offline vision-policy dataset |
| Representation Gap | VERIFIED | vision backbone/projector/action-hidden gaps measured for `oftplus_h5_vision` |
| Policy-Relevant Representation Gap | VERIFIED for offline vision policy / UNVERIFIED for ROS proprio policy | action-facing correlations and hidden correction verified offline; not for current deployment |
| Action Gap | VERIFIED for offline vision policy / UNVERIFIED for ROS proprio policy | final action gap and correction measured for `oftplus_h5_vision` |
| Real Performance | UNVERIFIED | no real robot policy rollout/shadow-mode validation in this audit |

## Continue / Re-run Decision

Continue using existing 225-pair results for:

- offline vision-policy method development,
- representation/action-gap reasoning,
- policy-sensitive direction analysis design.

Re-run required for:

- any claim about current ROS deployment,
- any claim about `oftplus_h5_proprio`,
- any claim involving proprio-conditioned action predictions,
- any real robot performance claim.

## Recommended Next Action

Do not repeat basic Observation/Distribution Gap analysis for the existing 225-pair `oftplus_h5_vision` dataset.

Next recommended analysis:

**Policy-Relevant Representation Gap analysis at `action_hidden_states.input`.**

Initial design:

- Use existing 225-pair vision-policy features as the first method-development dataset.
- For each frame, define:
  - `z_real`
  - `z_sim`
  - `delta_RS = z_sim - z_real`
  - `a_real`
  - `a_sim`
- Measure action sensitivity around `z_real` and/or `z_sim`:
  - directional finite difference along `delta_RS`,
  - random direction baselines,
  - token-wise masking or perturbation,
  - phase-conditioned sensitivity,
  - separate translation / rotation / gripper action sensitivity.
- Prefer starting with finite-difference / VJP-free methods on the saved action head, because the action head is already loadable and reproduces final actions.
- Only move to full gradient/Jacobian/JVP analysis after confirming tensor paths and memory cost.

Deployment branch:

- Build a separate 225-pair feature/action extraction path for `oftplus_h5_proprio` if ROS deployment relevance is required.
- That branch must include synchronized proprio state for Real and Sim frames.

