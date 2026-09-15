# Preprocessing Audit Summary

Date: 2026-09-15 KST

## Bundle Offline OFT Analysis

Code path:

- `sim2real_analysis/04_features/extract_vla_features.py`
- `runtime/openvla-oft/vla-scripts/serve_a0509_oft.py`
- `runtime/openvla-oft/experiments/robot/openvla_utils.py`

Current offline feature extraction uses:

- variant: `oftplus_h5_vision`
- checkpoint:
  - `/home/ubuntu/a0509_vla_linux_field_bundle_20260903/runtime_state/oft_mixed480_step28560_merged`
- `center_crop=True`
- image preparation:
  1. validate `uint8 H x W x 3`
  2. if not `224 x 224`, JPEG encode/decode
  3. resize to `224 x 224` with Lanczos3 antialias
  4. convert to PIL RGB
  5. apply center crop if `cfg.center_crop`
     - crop scale hard-coded in `center_crop_image`: `0.9`
  6. processor applies Prismatic fused vision processing

Processor config:

- `image_resize_strategy`: `resize-naive`
- `input_sizes`: two fused backbones, both `[3, 224, 224]`
- `use_fused_vision_backbone`: `true`
- normalization:
  - first backbone ImageNet-style mean/std
  - second backbone `[0.5, 0.5, 0.5]`

Status:

- Offline preprocessing path: `VERIFIED at code/config level`

## Current ROS Runtime Config

Files:

- `/home/ubuntu/robot_ws/src/openvla_doosan_runtime/config/runtime_oft.yaml`
- `/home/ubuntu/robot_ws/src/openvla_doosan_runtime/openvla_doosan_runtime/camera_adapter_node.py`
- `/home/ubuntu/robot_ws/src/openvla_doosan_runtime/openvla_doosan_runtime/openvla_oft_inference_node.py`

Camera adapter:

- Converts incoming ROS image to `rgb8`.
- Does not resize.
- Does not crop.

Current `runtime_oft.yaml`:

- `use_training_image_preprocess: false`
- `image_resize_size: 224`
- `center_crop_enabled: false`
- `center_crop_scale: 0.9`
- variant: `oftplus_h5_proprio`
- requires proprio: true

Status:

- Current ROS preprocessing config: `VERIFIED at config/code level`
- Equivalence with bundle offline preprocessing: `INVALID / RE-RUN REQUIRED`

## Decision

The existing LHJ 5-episode analysis is internally consistent for the bundle offline OFT policy, but it should not be treated as equivalent to the current ROS deployment path.

Before deployment/action claims, one of these must be done:

1. Configure ROS runtime to use the same bundle `oftplus_h5_vision` checkpoint and preprocessing.
2. Regenerate Real/Sim features/actions with the current ROS `oftplus_h5_proprio` policy, proprio input, and preprocessing.

## Impact

Preprocessing mismatch can change:

- vision tokens
- projector tokens
- action hidden states
- final action predictions

Therefore it is high impact for policy-relevant representation conclusions when transferring from offline analysis to real runtime.
