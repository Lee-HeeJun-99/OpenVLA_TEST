# Full Validation Report

Date: 2026-09-15 KST

Scope:

- Existing 5-episode paired offline bundle dataset.
- No real robot motion was executed.
- Results apply to the bundle offline policy/checkpoint, not the current ROS deployment policy.

## Validation Status

| Item | Status |
|---|---|
| Real feature/image manifest integrity | PASS |
| Sim feature/image manifest integrity | PASS |
| Paired manifest integrity | PASS |
| Observation/token gap rerun reproducibility | PASS |
| LOO representation shift rerun reproducibility | PASS |
| Policy/action shift rerun reproducibility | PASS |
| Representation-action gap rerun reproducibility | PASS |
| Current ROS deployment equivalence | INVALID / RE-RUN REQUIRED |
| Real robot performance | UNVERIFIED / REQUIRES REAL ROBOT APPROVAL |
| Causal gap attribution | UNVERIFIED / REQUIRES CONTROLLED ABLATION |

## Integrity Checks

Evidence:

- `comprehensive_offline_validation_summary.json`
- `comprehensive_offline_validation_status.csv`

Real:

- Records: `225`
- Unique frames: `225`
- Missing features: `0`
- Missing images: `0`
- NPZ files checked: `225`
- Shape errors: `0`
- Image size: `1280x720`
- Image mode: `RGB`
- Instruction: `Pick up the orange cube.`
- Variant: `oftplus_h5_vision`
- Chunk size: `5`
- Action shape errors: `0`

Sim:

- Records: `225`
- Unique frames: `225`
- Missing features: `0`
- Missing images: `0`
- NPZ files checked: `225`
- Shape errors: `0`
- Image size: `1280x720`
- Image mode: `RGB`
- Instruction: `Pick up the orange cube.`
- Variant: `oftplus_h5_vision`
- Chunk size: `5`
- Action shape errors: `0`

Paired manifest:

- Total pairs: `225`
- Episode counts:
  - `episode_000001`: 46
  - `episode_000002`: 47
  - `episode_000003`: 44
  - `episode_000004`: 45
  - `episode_000005`: 43
- Missing paired images: `0`

## Rerun Reproducibility

The following analyses were rerun into `lhj/phase2_full_validation` and numerically compared against the earlier outputs.

| Analysis | Numeric Count | Max Abs Diff | Status |
|---|---:|---:|---|
| Observation/token gap | 86 | 0.0 | PASS |
| LOO representation shift | 90 | 0.0 | PASS |
| Policy-relevant action shift | 183 | 0.0 | PASS |
| Representation-action gap | 43 | 0.0 | PASS |

This means the current offline outputs are reproducible from the existing files/scripts.

## Verified Offline Gap Results

Observation gap:

- Brightness absolute difference mean: `61.0193`
- RGB mean L2 mean: `106.1935`
- Resized PSNR mean: `10.6124`
- Resized luma SSIM global mean: `0.5888`

LOO representation shift:

- `vision_backbone.output` cosine:
  - `0.141749 -> 0.040102`
- `projector.output` cosine:
  - `0.092312 -> 0.018920`
- `vision_backbone.output` MMD:
  - `0.273608 -> 0.002791`
- `projector.output` MMD:
  - `0.339874 -> 0.002052`

Policy-relevant hidden shift:

- Final action chunk mean L2:
  - `0.370262 -> 0.072325`
- First action L2:
  - `0.316387 -> 0.051591`

No-collapse check:

- Real hidden feature variance mean: `0.346409`
- Sim hidden feature variance mean: `0.360631`
- Corrected hidden feature variance mean: `0.415532`
- Hidden L2 worsened frames: `8 / 225`
- Action worsened frames: `35 / 225`

Top policy relevance:

- `action_head_output_pooled_l2` vs `action_chunk_mean_l2`
  - Spearman: `0.960294`
  - Pearson: `0.960058`
- `action_hidden_states_input_pooled_cosine_distance` vs `action_chunk_mean_l2`
  - Spearman: `0.958045`
  - Pearson: `0.950559`

## Current Deployment Mismatch

Current ROS config:

- `/home/ubuntu/robot_ws/src/openvla_doosan_runtime/config/runtime_oft.yaml`

Mismatch:

- ROS runtime uses `oftplus_h5_proprio`.
- Offline bundle analysis used `oftplus_h5_vision`.
- ROS checkpoint:
  - `/home/ubuntu/robot_ws/src/openvla/runs/oftplus_h5_proprio_bounded_oft200_9000--6000_chkpt`
- Offline bundle checkpoint:
  - `/home/ubuntu/a0509_vla_linux_field_bundle_20260903/runtime_state/oft_mixed480_step28560_merged`
- ROS default instruction:
  - `pick up the cube`
- Offline analysis instruction:
  - `Pick up the orange cube.`
- ROS preprocessing flags:
  - `use_training_image_preprocess: false`
  - `center_crop_enabled: false`
- Offline analysis path used the bundle OFT feature extraction path and `oftplus_h5_vision`.

Decision:

- Current ROS deployment equivalence is `INVALID / RE-RUN REQUIRED`.
- The offline result should not be used as direct deployment evidence for the ROS runtime.

## Final Interpretation

The following is verified for the existing 5-episode offline bundle dataset:

1. Real/Sim observation and token gaps exist.
2. Real/Sim representation gaps exist.
3. Action gap exists.
4. Action-facing representation gap is strongly related to action gap.
5. Leave-One-Episode-Out progress-conditioned correction reduces representation gap.
6. Action-hidden progress-conditioned correction reduces final offline action gap.
7. Correction does not show obvious representation collapse.

The following is not verified:

1. The causal source of the gap.
2. Whether camera, geometry, lighting, control, or preprocessing is the dominant factor.
3. Whether the result transfers to the current ROS proprio policy.
4. Whether the result improves real robot task performance.
5. Whether the correction generalizes to unseen layouts beyond the current five episodes.

Conclusion:

The existing offline evidence reaches Level 2:

`Representation correction -> Offline Action Gap reduction`

It does not reach Level 3:

`Representation correction -> Real-world performance improvement`

