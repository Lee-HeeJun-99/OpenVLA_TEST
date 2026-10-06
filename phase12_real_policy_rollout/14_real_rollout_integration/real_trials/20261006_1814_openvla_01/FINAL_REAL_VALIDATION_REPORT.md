# Real hardware validation — 2026-10-06

Verdict: BLOCKED_TECHNICAL_VALIDATION_FAILED. Motion not authorized.

Branch: lhj-research. HEAD: 49649abeef643ba6713e6089a7ebfc65e973de4c. Working tree was clean before this observation.

Read-only operations: Git branch/HEAD/status; ROS node/topic/service listing after sourcing Humble and robot_ws; process listing; a 35-second rclpy subscriber-only JointState and camera observation. No getter or command service was called. Endpoint presence does not establish driver health. The node listing omitted controller_manager/dsr_controller2 while their service endpoints remained discoverable; this does not prove a process crash.

## Actual observation

Duration: 35.002709 seconds.

JointState: 83 messages; first receive delay 28.083221 s; receive rate over observed first-to-last samples 11.850646 Hz; max receive gap 3.068483 s; max source timestamp gap 3.060014 s; two receive gaps >=100 ms; no duplicate/non-monotonic source timestamps; no invalid position/velocity or missing joint names. Actual order: joint_1, joint_2, joint_4, joint_5, joint_3, joint_6. Effort was not used for validity. Result: FAIL. The initial discovery delay and the subsequent source gap are separate observations. Cause is not established by this test; concurrent image handling or host effects have not been isolated.

Camera: 1,233 messages; 35.338253 Hz over received interval; 1280x720 bgra8; first delay 0.134455 s; max receive gap 0.175974 s; 16 gaps >=100 ms. Reception confirmed, but model conversion/continuity validation is not complete. Existing preprocessing must explicitly support this encoding; RGB must not be assumed.

## Stage status

- Robot connection: UNVERIFIED (endpoints observed only).
- JointState: FAIL.
- TCP: NOT_EXECUTED.
- Hardware preflight/operator confirmations: UNVERIFIED.
- OpenVLA GPU health: NOT_EXECUTED.
- Live prediction-only dry-run: NOT_EXECUTED.
- Minimum-motion: NOT_EXECUTED.
- Hold/Stop: NOT_EXECUTED.
- Gripper polarity: NOT_EXECUTED.
- Short-horizon: NOT_EXECUTED.
- OFT: NOT_STARTED.
- Full-task: NOT_AUTHORIZED.

All physical robot commands, motion service/action calls, gripper commands, Home, trajectory, Hold/E-stop, AI command publications and real rollout executions in this attempt: 0. No driver restart or controller state change was performed.

Next allowed stage: read-only JointState-only continuity verification after discovery, with contemporaneous driver/host logs, to distinguish feedback/transport stall from observation harness effects. No motion-stage progression until continuity passes and operator/hardware/model gates are confirmed. Preserve previous results rather than treating this observation as a root-cause diagnosis.
