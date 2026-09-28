# Real control dataflow

## Existing runtime (unsafe to launch remotely)

`Image → inference (/vla/raw_action, m/rad, gripper openness) → ActionAdapter → absolute /vla/target_pose [mm,deg] + /vla/gripper_open → DoosanBridge workspace/rate/filter processing → ServolStream/SpeedlStream publisher or MoveLine/MoveBlending service → driver → GetCurrentPosx service → /doosan/current_pose`.

The action adapter applies `axis_sign`, converts metres to millimetres and radians to degrees, limits each step, optionally suppresses rotation (`apply_rotation=false` in current configs), adds the delta to measured pose, and validates the workspace. The bridge applies its own workspace checks and streaming filters/rate limits. Joint-limit enforcement was not found in this package; Cartesian velocity parameters exist, but hardware-side enforcement is unresolved.

The physical separation point is explicit: inference publishes only `/vla/raw_action`; `ActionAdapterNode._action_callback` creates robot-facing targets; `DoosanBridgeNode._target_callback` accepts them; `_publish_servol_command`, `_publish_speedl_command`, `_execute_motion`, and `_set_gripper` contain state-changing paths. These files must not be imported/launched for recorded-input Shadow Mode.

## Phase 10 recorded-input Shadow Mode

`Observed camera + measured state + observed planner/model target/command → RealRuntimeObserverAdapter → RealEpisodeRecorder → immutable episode → OpenVLA adapter (prediction only) + OFT adapter K=5 (prediction only)`.

The new adapter/recorder contains no ROS import, publisher, service client, or action client. Robot-facing output is absent, and both `openvla_executed_action` and `oft_executed_action` are null. An observed `/dsr01/servol_stream` sample may later be logged as evidence of what the existing controller sent; observation never republishes it.

## Canonical mapping status

- Model raw translation: delta metres; rotation: delta radians; gripper: `0=close, 1=open` per `ActionAdapterNode`; canonical gripper closedness is `1 - openness`.
- Runtime target/EE pose: absolute `[x,y,z,rx,ry,rz]` in millimetres/degrees.
- Rotation is labelled degrees by source, but exact Doosan posx Euler convention and reference frame value `reference=0` meaning are `UNRESOLVED`; target poses are therefore not silently converted to rotvec.
- Control horizon for raw actions, dedicated planner action, camera/state rates, measured gripper state, joint/velocity limit guarantees, and cross-clock synchronization remain blockers.
