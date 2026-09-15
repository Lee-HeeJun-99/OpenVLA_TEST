# Camera Audit Summary

Date: 2026-09-15 KST

## Real Camera

Verified from 5-episode metadata:

- camera: `Stereolabs ZED`
- camera mount: `eye_in_hand_vertical`
- image topic: `/zed/zed_node/rgb/color/rect/image`
- image format: `jpg`
- image camera in dataset manifest: `primary`

Current evidence level:

- Stream/topic: `VERIFIED`
- Left/right physical mapping: `UNVERIFIED`
- Intrinsic matrix used during collection: `UNVERIFIED`
- Distortion/rectification parameters used during collection: `UNVERIFIED`
- Camera-to-tool/base extrinsic: `UNVERIFIED`

Notes:

- `/home/ubuntu/robot_ws/src/doosan-robot2/dsr_example2/dsr_visualservoing/config/real_camera.yaml` exists, but it is a Doosan visual-servoing example config with `2304x1536` image size.
- No evidence was found that this file was used by the 5-episode dataset collection or by `/zed/zed_node/rgb/color/rect/image`.
- ZED wrapper configs exist under `/home/ubuntu/robot_ws/src/zed-ros2-wrapper/zed_wrapper/config`, including `zed2i.yaml`, but these do not provide the dataset-time calibrated `CameraInfo` values by themselves.

Decision:

- Do not use `real_camera.yaml` as the 5-episode Real camera calibration.
- Treat Real camera intrinsics/extrinsics as missing until a recorded `/camera_info` topic, ZED calibration dump, or dataset-time calibration evidence is found.

## Sim Camera

Verified from authored USD extract:

- camera prim:
  - `/World/a0509/link_6/tool0/left_Camera`
- local translate:
  - `[-0.060222793661, -0.10842505344, 0.064837748142]`
- local rotateXYZ degree:
  - `[155.0000187483, 0, 0]`
- focal length:
  - `2.12`
- horizontal/vertical aperture:
  - `[5.376, 3.024]`
- clipping range:
  - `[0.05, 100]`

Current evidence level:

- Authored local camera values: `VERIFIED`
- Composed world transform after USD references/composition: `UNVERIFIED`
- Equivalence to Real ZED left/right stream: `UNVERIFIED`
- Distortion model: no authored distortion found in current extract.

## Impact

Camera mismatch remains a high-impact audit risk for interpreting Observation Gap.

Current representation/action analyses remain valid as paired offline measurements for the existing images, but camera-cause attribution is not yet supported.
