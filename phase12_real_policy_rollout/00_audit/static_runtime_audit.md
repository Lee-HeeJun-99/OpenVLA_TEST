# Phase 12 static Real runtime audit

## Revision and preservation

- ROS workspace Git root: `/home/ubuntu/robot_ws`
- HEAD: `86eaa9632d651eb907332334d02f32c1461850d7`
- Working tree: dirty before Phase 12. Existing modifications/untracked files were not reset, deleted, stashed, or edited.
- Research bundle is not a Git working tree.
- Runtime source itself was not modified; Phase 12 additions are under the new Phase 12 directory only.

## Command path

`runtime.launch.py` starts camera relay, inference, ActionAdapter, DoosanBridge and EpisodeManager together. It is unsafe for read-only inspection and was not launched.

Closed-loop path from source:

`/zed/.../image -> /vla/image_rgb -> inference -> /vla/raw_action -> ActionAdapter -> /vla/target_pose + /vla/gripper_open -> DoosanBridge -> MoveLine/MoveBlending or Servol/SpeedL + tool digital output`.

EpisodeManager publishes `/vla/enable` and `/vla/emergency_stop`; enabling causes ActionAdapter and DoosanBridge to request an initial gripper open. Therefore even enabling without target motion is state-changing.

## Critical findings

1. **Gripper polarity mismatch:** `ActionAdapter._resolve_gripper_state()` maps high values to open and low values to close. Phase 11/model contract is `0=open, 1=closed`; runtime behavior is reversed.
2. **OFT K=5 discarded:** `OpenVLAOFTInferenceNode._predict_action()` validates `(5,7)` and returns only `action_array[0]`. No K=5 queue exists.
3. **Wrong OFT experiment config:** `runtime_oft.yaml` points to `oftplus_h5_proprio`, not Phase 11 `oftplus_h5_vision` step 28560.
4. **Instruction mismatch:** runtime configs use `pick up the cube`, not `Pick up the orange cube.`
5. **Preprocessing mismatch:** OFT config disables training preprocessing and center crop, unlike audited Phase 11 OFT server center crop.
6. **Unsafe/unverified workspace:** ActionAdapter OFT bounds are `[-2500,-4000,-2000]..[7500,4000,7000]` mm; these are not a measured experiment volume. Vanilla runtime also contains warning placeholders.
7. **Noncanonical translation scale:** runtime uses 2800 mm per model translation unit instead of a documented canonical meter-to-mm mapping. This requires experimental justification before motion.
8. **Command rate mismatch:** inference minimum period is 0.2 s while OFT is expected at 1 Hz with K=5 consumption. Doosan servo publisher is 20 Hz, not the requested policy execution contract of 5 Hz.
9. **No measured gripper feedback:** `/vla/gripper_open` is a command topic. OFT subscribes to it as proprio gripper state, so this is commanded state, not measured feedback.
10. **TCP pose:** `/doosan/current_pose` is published from `GetCurrentPosx`; message has no source header. Receive-time policy is required.
11. **Hold acknowledgement:** no positive hold acknowledgement channel/state machine was found. Software emergency stop requests `MoveStop`, but this is not hardware E-stop.
12. **Joint/velocity limits:** TCP workspace and rate limits exist, but explicit robot joint-position limit validation was not found in the AI command path.

## Orientation

ActionAdapter treats model rotation as rotation-vector radians then adds converted degrees componentwise to Doosan pose fields. Doosan `posx` orientation is controller Euler-style degree fields, not proven to be a rotation vector. Componentwise addition is unresolved and `apply_rotation=true` in OFT config is blocked pending local/controller convention validation.

## Read-only runtime check

ROS2 CLI commands were attempted read-only only. The sandbox denied socket access before graph discovery, so nodes/topics/services/actions could not be observed. Model health endpoints 8765/8766 were not running. No service/action/publish operation was attempted.
