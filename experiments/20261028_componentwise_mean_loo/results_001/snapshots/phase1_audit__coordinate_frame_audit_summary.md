# Coordinate Frame / TCP / EEF Audit Summary

Date: 2026-09-15 KST

## Real Dataset Pose Source

Real collection code:

- `/home/ubuntu/robot_ws/src/doosan-robot2/dsr_example2/dsr_example/dsr_example/simple/single_robot_simple.py`

Relevant functions/logic:

- `robot_pose_to_dataset_pose(pose_mm_deg)`
  - converts:
    - xyz: `mm -> m`
    - Rx/Ry/Rz: `deg -> rad`
- `tcp_pose_to_end_effector_pose(tcp_pose_m_rad)`
  - interprets the last three `tcp_pose` values as roll/pitch/yaw-like angles and converts them with `rpy_to_quaternion_wxyz`.
- `EpisodeRecorder.finalize_planned_poses()`
  - rewrites recorded steps with planned pose interpolation when planned pose mode is used.

5-episode metadata:

- `pose_source`: `planned_commanded_pose`
- step-level `pose_source`: `planned_actual_duration`
- `feedback_pose_samples`: `0`
- `planned_pose_samples`: equals frame count

Interpretation:

- The saved Real `tcp_pose` is the planned/commanded TCP pose in robot base coordinates.
- It is not measured controller feedback for these 5 episodes.
- The last three `tcp_pose` components are radian-converted Doosan pose orientation fields, not guaranteed to be a true rotation-vector representation.
- The dataset action rotation uses quaternion delta via `quaternion_delta_rotvec`, so action rotation convention is different from simply subtracting the last three `tcp_pose` fields.

Status:

- Real commanded pose storage path: `VERIFIED`
- Real measured physical TCP tracking: `UNVERIFIED`
- Real orientation field semantics beyond code conversion: `PARTIALLY VERIFIED`

## Sim EEF Pose Source

Sim replay code:

- `/home/ubuntu/a0509_vla_linux_field_bundle_20260903/isaac_sim/a0509_control_app.py`
- `/home/ubuntu/a0509_vla_linux_field_bundle_20260903/isaac_sim/a0509_control/lula_planner.py`

Relevant settings:

- `--end-effector-frame`: default `tool0`
- `LulaPlanner.get_end_effector_pose()`
  - uses `ArticulationKinematicsSolver.compute_end_effector_pose()`
  - returns position and quaternion for `tool0`.

Replay report fields:

- `sim_end_effector_pose.position_m`
- `sim_end_effector_pose.quaternion_wxyz`

Status:

- Sim EEF frame used in replay reports: `VERIFIED as tool0`
- Sim EEF pose computation source: `VERIFIED at code level`

## Real vs Sim Direct Pose Comparison

Previously observed:

- Real initial TCP position:
  - around `[0.305, -0.0118, 0.7237] m`
- Sim initial EEF/tool0 position:
  - around `[0.308, -0.0115, 1.5390] m`

Decision:

- This numeric difference must not be interpreted as a physical mismatch yet.
- Reasons:
  - Real value is planned-commanded TCP pose, not measured feedback.
  - Real TCP frame and Sim `tool0` frame are not proven identical.
  - Real orientation fields and Sim quaternion are produced by different conventions.
  - World/base height origins may differ.

Status:

- Direct Real TCP vs Sim EEF comparison: `INVALID / RE-RUN REQUIRED` until transform definitions are reconciled.

## Valid Current Correspondence

The valid replay correspondence is joint-based:

```text
Sim target joints = Sim HOME + (Real source joints - Real first source joints)
```

This correspondence is verified by replay reports:

- Real source joint values are copied from `steps.jsonl`.
- Sim target vs actual joint error after snap is near zero.

Current valid claim:

- Sim replay reproduces the recorded Real joint deltas relative to each domain's HOME joint configuration.

Current invalid claim:

- Sim replay reproduces the exact Real TCP/EEF physical pose.

## Required Work For Strong Frame Claims

To compare Real and Sim TCP/EEF physically:

1. Define Real base frame and Sim world/base frame relation.
2. Define Real TCP/tool frame and Sim `tool0` frame relation.
3. Confirm whether Doosan Rx/Ry/Rz fields are Euler, rotation vector, or controller-specific orientation convention.
4. Obtain measured Real feedback pose or controller logs for the 5Hz samples.
5. Compute transforms through a single convention before comparing positions/orientations.
