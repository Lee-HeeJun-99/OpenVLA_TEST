# Active tool/TCP static audit — 2026-10-02

## Finding

No repository source establishes the active controller TCP name or the flange-to-TCP
transform used by the currently connected A0509.

Evidence:

- A0509 URDF contains the six-joint chain through `link_6`.
- The `joint_6-tool0` block is commented out and its translation is zero even in the
  commented definition; it does not describe the installed gripper tip.
- The collection scripts call controller TCP getters and execute task poses but do not
  configure or persist an active TCP transform.
- `openvla_doosan_runtime` polls controller TCP pose and sends tool digital-output
  gripper commands; it does not declare the mechanical gripper/TCP offset.
- The gripper is controlled with Tool DO pulses. Command state is tracked, but no
  measured gripper position/closed sensor mapping was found.

Therefore the JointState-derived FK result is valid only as a computed `link_6` flange
observation. It must not be labeled measured TCP or used to approve workspace limits.

## Required stationary pendant evidence

The local operator must record, without moving the robot:

1. Active TCP/tool name shown by the controller/pendant.
2. Configured TCP translation `[x, y, z]` and orientation `[A, B, C]`, including units.
3. Current controller TCP pose `[x, y, z, A, B, C]`, reference frame, and units.
4. Whether the displayed pose is flange, active TCP, or tool-tip pose.
5. Gripper model and mounting adapter identity.
6. Whether any gripper open/closed feedback sensor exists and its documented signal.

No service getter should be retried for this collection. A photograph or manual
transcription of the stationary pendant screen is sufficient for the first cross-check.

## Blockers

- `UNVERIFIED_FLANGE_TO_TCP_OFFSET_AND_NO_MEASURED_TCP_CROSSCHECK`
- `MEASURED_GRIPPER_FEEDBACK_NOT_FOUND`

Motion readiness remains false.
