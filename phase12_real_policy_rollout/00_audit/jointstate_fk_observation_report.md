# JointState-derived flange FK observation — 2026-10-02

## Purpose and classification

This is a command-free alternative observation path that avoids the unstable
`get_current_posx` service. It computes the A0509 `link_6` flange pose from measured
joint positions and the repository URDF.

It is explicitly classified as:

```text
computed_from_measured_joint_state_urdf_fk
computed_link6_flange_not_measured_tcp
```

It is not controller-measured Cartesian feedback and is not yet accepted for motion
readiness.

## Static contract

- URDF: `/home/ubuntu/robot_ws/src/doosan-robot2/dsr_description2/urdf/a0509.urdf`
- Parent frame: `base_link`
- Child frame: `link_6`
- Joint values: radians, reordered by names `joint_1` through `joint_6`
- URDF transform: fixed-axis RPY, followed by each joint axis rotation
- Tool/TCP offset: not applied
- The URDF's `joint_6-tool0` block is commented out, so no validated tool transform is available from this file.

## Live subscriber-only result

One `/dsr01/joint_states` message was received without creating a publisher, service
client, or action client. Wire order was
`joint_1,joint_2,joint_4,joint_5,joint_3,joint_6`; name-based canonical reordering was
applied.

Computed `link_6` position in `base_link`:

```text
[0.3065574091, -0.0121740200, 0.7232624636] m
```

Computed quaternion (`wxyz`):

```text
[-0.0067012468, 0.9771601281, -0.0293125011, 0.2103662390]
```

Artifact: `03_shadow_mode/computed_flange_observation_20261002.json`.

## Offline consistency check

Episode 4 step 0 uses `pose_source=planned_actual_duration`, so it is not measured
feedback. For the recorded joints, URDF `link_6` FK differed from that planned position
by 1.112 mm L2. This supports implementation consistency only; it is not an independent
accuracy validation.

## Tests

- FK-specific tests: 4/4 passed.
- Full Phase 12 suite with bundle Python: 31/31 passed.
- Missing/non-finite joints rejected.
- Noncanonical wire order is reordered by name.
- No publisher/service/action/command capability in the FK module.

## Remaining blocker

`UNVERIFIED_FLANGE_TO_TCP_OFFSET_AND_NO_MEASURED_TCP_CROSSCHECK`

To accept this path for safety monitoring, a local operator must provide or verify the
active tool/TCP transform and cross-check the stationary flange/TCP pose against an
independent controller/pendant reading at multiple poses. That future comparison may
require physical pose changes and therefore needs explicit motion approval; it was not
performed here.

## Stationary pendant follow-up

The local operator subsequently reported:

- active TCP: `Tool_v1`
- configured offset: x/y/z/A/B/C all zero
- current pose: `[307.10, -12.75, 722.39] mm`, rotation displayed as
  `[179.74, -155.72, 3.30] deg`

Against the same-session JointState FK, position differed by 1.178 mm. Interpreting
the displayed rotation using the audited Doosan ZYZ convention gives a rotation-matrix
geodesic difference of 0.177°. Euler triples themselves differ because ZYZ
representations are non-unique.

Updated classification:

```text
ONE_POSE_STATIONARY_PENDANT_CROSSCHECK_PASS
ACTIVE_TCP_ZERO_OFFSET_MATCHES_LINK6_CONFIGURATION
CONTINUOUS_MEASURED_TCP_STILL_NOT_AVAILABLE
```

The previous blocker is narrowed: the tool offset is now operator-confirmed as zero and
the single stationary cross-check is consistent, but FK remains computed from measured
joints rather than independently measured controller TCP. Multi-pose validation would
require explicit motion approval and was not performed.
