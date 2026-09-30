# Real episode provenance audit

Status: source and recorded-data audit only. Original datasets, code, Phase 8/10 results and model outputs were not modified. No inference, ROS or robot operation was executed.

## Final classification

All ten canonical episodes are classified `REAL_INDEPENDENT_COLLECTION` in the taxonomy requested here. This means the RGB observations were independently captured from the physical ZED/robot run and the executed reference route was a Real-side scripted TCP route—not that the experiment was independent of every Sim asset. Cube layouts were supplied by Sim/Real mapping JSON files.

| episode_id | image_domain | trajectory_origin | action_origin | pose_source | measured_feedback_available | sim_trajectory_id | confidence | final_classification |
|---|---|---|---|---|---|---|---|---|
| episode_000001 | REAL_ZED_CAMERA | REAL_SCRIPTED_TCP_REFERENCE_ROUTE_WITH_SIM_DERIVED_LAYOUT | offline delta from planned pose; commanded gripper | planned/interpolated | joint only; no measured EE/gripper | NOT_MAPPED | HIGH (0.90) | REAL_INDEPENDENT_COLLECTION |
| episode_000002 | REAL_ZED_CAMERA | REAL_SCRIPTED_TCP_REFERENCE_ROUTE_WITH_SIM_DERIVED_LAYOUT | offline delta from planned pose; commanded gripper | planned/interpolated | joint only; no measured EE/gripper | NOT_MAPPED | HIGH (0.90) | REAL_INDEPENDENT_COLLECTION |
| episode_000003 | REAL_ZED_CAMERA | REAL_SCRIPTED_TCP_REFERENCE_ROUTE_WITH_SIM_DERIVED_LAYOUT | offline delta from planned pose; commanded gripper | planned/interpolated | joint only; no measured EE/gripper | NOT_MAPPED | HIGH (0.90) | REAL_INDEPENDENT_COLLECTION |
| episode_000004 | REAL_ZED_CAMERA | REAL_SCRIPTED_TCP_REFERENCE_ROUTE_WITH_SIM_DERIVED_LAYOUT | offline delta from planned pose; commanded gripper | planned/interpolated | joint only; no measured EE/gripper | NOT_MAPPED | HIGH (0.90) | REAL_INDEPENDENT_COLLECTION |
| episode_000005 | REAL_ZED_CAMERA | REAL_SCRIPTED_TCP_REFERENCE_ROUTE_WITH_SIM_DERIVED_LAYOUT | offline delta from planned pose; commanded gripper | planned/interpolated | joint only; no measured EE/gripper | NOT_MAPPED | HIGH (0.90) | REAL_INDEPENDENT_COLLECTION |
| episode_000006 | REAL_ZED_CAMERA | REAL_SCRIPTED_TCP_REFERENCE_ROUTE_WITH_SIM_DERIVED_LAYOUT | offline delta from planned pose; commanded gripper | planned/interpolated | joint only; no measured EE/gripper | NOT_MAPPED | HIGH (0.90) | REAL_INDEPENDENT_COLLECTION |
| episode_000007 | REAL_ZED_CAMERA | REAL_SCRIPTED_TCP_REFERENCE_ROUTE_WITH_SIM_DERIVED_LAYOUT | offline delta from planned pose; commanded gripper | planned/interpolated | joint only; no measured EE/gripper | NOT_MAPPED | HIGH (0.90) | REAL_INDEPENDENT_COLLECTION |
| episode_000008 | REAL_ZED_CAMERA | REAL_SCRIPTED_TCP_REFERENCE_ROUTE_WITH_SIM_DERIVED_LAYOUT | offline delta from planned pose; commanded gripper | planned/interpolated | joint only; no measured EE/gripper | NOT_MAPPED | HIGH (0.95) | REAL_INDEPENDENT_COLLECTION |
| episode_000009 | REAL_ZED_CAMERA | REAL_SCRIPTED_TCP_REFERENCE_ROUTE_WITH_SIM_DERIVED_LAYOUT | offline delta from planned pose; commanded gripper | planned/interpolated | joint only; no measured EE/gripper | NOT_MAPPED | HIGH (0.95) | REAL_INDEPENDENT_COLLECTION |
| episode_000010 | REAL_ZED_CAMERA | REAL_SCRIPTED_TCP_REFERENCE_ROUTE_WITH_SIM_DERIVED_LAYOUT | offline delta from planned pose; commanded gripper | planned/interpolated | joint only; no measured EE/gripper | NOT_MAPPED | HIGH (0.95) | REAL_INDEPENDENT_COLLECTION |

## Evidence that images are Real camera observations

- 452 Real episode JPEGs were compared by SHA-256 against 1510 available Sim images from `trajectories_10` and `outputs/sim2real_analysis`; exact matches: **0**.
- Every episode stores 1280×720 JPEG frames and identifies `Stereolabs ZED`, eye-in-hand mount and `/zed/zed_node/rgb/color/rect/image`.
- Every record has `image_timestamp`, `timestamp_ns`, image age and a process monotonic receive/sample timestamp. `timestamp_ns` matches the ZED/ROS image stamp conversion for all 452 rows.
- Direct visual inspection of representative Real and Sim frames shows a physical camera scene in the former and Isaac-rendered geometry/shadows in the latter. Resolution equality alone was not used as domain evidence.

## Trajectory and action provenance

- Metadata records exactly three commanded motion segments: alignment, descent-to-grasp and lift. These are absolute TCP waypoints issued by the Real collection path.
- The collection code default is `tcp-z-offset`: it loads a route template from `/home/ubuntu/robot_ws/src/doosan-robot2/raw_dataset_oft/episodes/episode_000001/steps.jsonl`, itself a ZED/Doosan Real recording, then overrides target XY from the layout JSON.
- The layout files contain both `real_layout_mm` and `layout_m`; therefore layouts are Sim/Real coordinated. This is evidence of a Sim-derived layout, not evidence that Sim per-step waypoints were replayed.
- `steps_with_actions.jsonl` is produced after recording: translation and relative rotation are calculated from consecutive stored planned poses, with the following step’s commanded gripper state. It is not a log of raw Doosan executed commands.
- Per-step `tcp_pose`/`end_effector_pose` were overwritten/finalized from interpolation over commanded motion segments (`planned_actual_duration`). Metadata reports `feedback_pose_samples=0` for every episode.
- Joint position/velocity came from JointState while recording. The gripper scalar came from the recorder’s internal commanded state updated around digital-output commands; no measured gripper feedback was found.

## Test of the proposed 100/98/113/108/127/106/115/88/102/101 mapping

- Those numbers are the **step counts** of the ten `trajectories_10` Sim directories, not stored trajectory IDs.
- The Sim collection occurred on 2026-09-23, after Real episodes were recorded on 2026-09-15, 2026-09-20 and 2026-09-22. It therefore cannot be the source replay for those Real recordings.
- No Real metadata or step file contains a `trajectories_10` path, those directory names, or their step counts as source IDs.
- Sequential target-position matching fails. Episode 1’s Sim-layout target is `[0.4005, -0.3174]`, whereas Sim trajectory 01 is approximately `[0.4806, -0.1868]`. Episode 4/8/9/10 happen to be within 1.42 mm of trajectory 01’s target, but all four cannot map one-to-one to Sim IDs 04/08/09/10, and the later creation time rules out source replay.
- Result: `sim_trajectory_id=NOT_MAPPED` for episodes 1–10.

## Classification boundary

- `SIM_TRAINING_DATA`: rejected; images/timestamps and physical scene support Real ZED capture.
- `REAL_CAMERA_WITH_SIM_TRAJECTORY_REPLAY`: rejected for the inspected records; Sim-coordinated layouts exist, but no Sim step/waypoint source is recorded and the later `trajectories_10` cannot be causal.
- `REAL_INDEPENDENT_COLLECTION`: selected, specifically **Real camera + Real robot scripted TCP reference collection with Sim-derived layout coordinates**.
- This classification does not upgrade planned poses to measured ground truth. Reference actions remain planned/commanded provenance.

## Episode-specific evidence


### episode_000001

46 physical-scene JPEGs 1280x720; ZED topic /zed/zed_node/rgb/color/rect/image; image_timestamp/timestamp_ns present and consistent; exact SHA matches to 1510 audited Sim images=0. metadata block_layout_source=/home/ubuntu/a0509_vla_linux_field_bundle_20260903/outputs/sim2real_analysis/trajectory001_manual_layout.json; motion_segments=3 (alignment/descent/lift); planned_pose_samples=46, feedback_pose_samples=0. steps_with_actions derives observation_t→t+1 deltas from planned poses. gripper_value_semantics=commanded_state. Nearest later trajectories_10 item=trajectory_03_red_x0p520_yneg0p300 (113 steps), target distance=120.68 mm; no source-path/ID reference found.

### episode_000002

47 physical-scene JPEGs 1280x720; ZED topic /zed/zed_node/rgb/color/rect/image; image_timestamp/timestamp_ns present and consistent; exact SHA matches to 1510 audited Sim images=0. metadata block_layout_source=/home/ubuntu/a0509_vla_linux_field_bundle_20260903/outputs/sim2real_analysis/trajectory002_manual_layout.json; motion_segments=3 (alignment/descent/lift); planned_pose_samples=47, feedback_pose_samples=0. steps_with_actions derives observation_t→t+1 deltas from planned poses. gripper_value_semantics=commanded_state. Nearest later trajectories_10 item=trajectory_03_red_x0p520_yneg0p300 (113 steps), target distance=43.48 mm; no source-path/ID reference found.

### episode_000003

44 physical-scene JPEGs 1280x720; ZED topic /zed/zed_node/rgb/color/rect/image; image_timestamp/timestamp_ns present and consistent; exact SHA matches to 1510 audited Sim images=0. metadata block_layout_source=/home/ubuntu/a0509_vla_linux_field_bundle_20260903/outputs/sim2real_analysis/trajectory003_manual_layout.json; motion_segments=3 (alignment/descent/lift); planned_pose_samples=44, feedback_pose_samples=0. steps_with_actions derives observation_t→t+1 deltas from planned poses. gripper_value_semantics=commanded_state. Nearest later trajectories_10 item=trajectory_01_red_x0p481_yneg0p188 (100 steps), target distance=80.09 mm; no source-path/ID reference found.

### episode_000004

45 physical-scene JPEGs 1280x720; ZED topic /zed/zed_node/rgb/color/rect/image; image_timestamp/timestamp_ns present and consistent; exact SHA matches to 1510 audited Sim images=0. metadata block_layout_source=/home/ubuntu/a0509_vla_linux_field_bundle_20260903/outputs/sim2real_analysis/trajectory004_manual_layout.json; motion_segments=3 (alignment/descent/lift); planned_pose_samples=45, feedback_pose_samples=0. steps_with_actions derives observation_t→t+1 deltas from planned poses. gripper_value_semantics=commanded_state. Nearest later trajectories_10 item=trajectory_01_red_x0p481_yneg0p188 (100 steps), target distance=1.42 mm; no source-path/ID reference found.

### episode_000005

43 physical-scene JPEGs 1280x720; ZED topic /zed/zed_node/rgb/color/rect/image; image_timestamp/timestamp_ns present and consistent; exact SHA matches to 1510 audited Sim images=0. metadata block_layout_source=/home/ubuntu/a0509_vla_linux_field_bundle_20260903/outputs/sim2real_analysis/trajectory005_manual_layout.json; motion_segments=3 (alignment/descent/lift); planned_pose_samples=43, feedback_pose_samples=0. steps_with_actions derives observation_t→t+1 deltas from planned poses. gripper_value_semantics=commanded_state. Nearest later trajectories_10 item=trajectory_10_red_x0p491_yneg0p081 (101 steps), target distance=92.79 mm; no source-path/ID reference found.

### episode_000006

44 physical-scene JPEGs 1280x720; ZED topic /zed/zed_node/rgb/color/rect/image; image_timestamp/timestamp_ns present and consistent; exact SHA matches to 1510 audited Sim images=0. metadata block_layout_source=/home/ubuntu/a0509_vla_linux_field_bundle_20260903/outputs/sim2real_analysis/trajectory006_manual_layout.json; motion_segments=3 (alignment/descent/lift); planned_pose_samples=44, feedback_pose_samples=0. steps_with_actions derives observation_t→t+1 deltas from planned poses. gripper_value_semantics=commanded_state. Nearest later trajectories_10 item=trajectory_08_red_x0p454_ypos0p097 (88 steps), target distance=36.54 mm; no source-path/ID reference found.

### episode_000007

48 physical-scene JPEGs 1280x720; ZED topic /zed/zed_node/rgb/color/rect/image; image_timestamp/timestamp_ns present and consistent; exact SHA matches to 1510 audited Sim images=0. metadata block_layout_source=/home/ubuntu/a0509_vla_linux_field_bundle_20260903/outputs/sim2real_analysis/trajectory007_manual_layout.json; motion_segments=3 (alignment/descent/lift); planned_pose_samples=48, feedback_pose_samples=0. steps_with_actions derives observation_t→t+1 deltas from planned poses. gripper_value_semantics=commanded_state. Nearest later trajectories_10 item=trajectory_02_red_x0p420_ypos0p280 (98 steps), target distance=115.84 mm; no source-path/ID reference found.

### episode_000008

45 physical-scene JPEGs 1280x720; ZED topic /zed/zed_node/rgb/color/rect/image; image_timestamp/timestamp_ns present and consistent; exact SHA matches to 1510 audited Sim images=0. metadata block_layout_source=/home/ubuntu/a0509_vla_linux_field_bundle_20260903/outputs/sim2real_analysis/trajectory008_from_episode004_layout.json; motion_segments=3 (alignment/descent/lift); planned_pose_samples=45, feedback_pose_samples=0. steps_with_actions derives observation_t→t+1 deltas from planned poses. gripper_value_semantics=commanded_state. Nearest later trajectories_10 item=trajectory_01_red_x0p481_yneg0p188 (100 steps), target distance=1.42 mm; no source-path/ID reference found.

### episode_000009

45 physical-scene JPEGs 1280x720; ZED topic /zed/zed_node/rgb/color/rect/image; image_timestamp/timestamp_ns present and consistent; exact SHA matches to 1510 audited Sim images=0. metadata block_layout_source=/home/ubuntu/a0509_vla_linux_field_bundle_20260903/outputs/sim2real_analysis/trajectory009_from_episode004_layout.json; motion_segments=3 (alignment/descent/lift); planned_pose_samples=45, feedback_pose_samples=0. steps_with_actions derives observation_t→t+1 deltas from planned poses. gripper_value_semantics=commanded_state. Nearest later trajectories_10 item=trajectory_01_red_x0p481_yneg0p188 (100 steps), target distance=1.42 mm; no source-path/ID reference found.

### episode_000010

45 physical-scene JPEGs 1280x720; ZED topic /zed/zed_node/rgb/color/rect/image; image_timestamp/timestamp_ns present and consistent; exact SHA matches to 1510 audited Sim images=0. metadata block_layout_source=/home/ubuntu/a0509_vla_linux_field_bundle_20260903/outputs/sim2real_analysis/trajectory010_from_episode004_layout.json; motion_segments=3 (alignment/descent/lift); planned_pose_samples=45, feedback_pose_samples=0. steps_with_actions derives observation_t→t+1 deltas from planned poses. gripper_value_semantics=commanded_state. Nearest later trajectories_10 item=trajectory_01_red_x0p481_yneg0p188 (100 steps), target distance=1.42 mm; no source-path/ID reference found.
