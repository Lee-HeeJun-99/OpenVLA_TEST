# Work log

## 2026-10-02 — Gate 0 static preparation

- Audited dirty Real runtime at HEAD `86eaa9632d651eb907332334d02f32c1461850d7`; preserved every existing change.
- Did not launch `runtime.launch.py` because it connects inference, adapter, bridge and episode manager to physical command paths.
- Confirmed reversed gripper polarity and OFT K=5 index-0-only behavior.
- Identified model/config, instruction, preprocessing, workspace, scale, frequency, orientation, feedback and hold-ack blockers.
- Implemented Phase 12 offline-only safety, chunk queue, gripper and logger contracts.
- Eight mock tests passed.
- Read-only ROS graph attempt was blocked by sandbox socket permission; model servers were not running.
- Robot/motion/gripper/Home/Hold/E-stop/closed-loop operations: zero.

## 2026-10-02 — Gate 1 read-only integration

- Preserved runtime HEAD `86eaa9632d651eb907332334d02f32c1461850d7` and its pre-existing dirty tree; no runtime source/config was edited.
- Executed only read-only ROS list/info/hz commands. Observed `/robot_state_publisher`; required ZED, Doosan, and VLA interfaces were absent.
- Health GETs to ports 8766, 8765, and 8000 were connection-refused; no model or robot node was launched.
- Added safe Shadow config/core, passive subscription specification, offline-only rotation candidate, Gate 1 tests, and reports under Phase 12.
- Full offline suite: 13 passed, 0 failed. Live recorder was not executed because observation topics and prediction servers were absent.
- Action blockers remain `UNRESOLVED_TRANSLATION_SCALE` and `UNRESOLVED_ROTATION_CONVENTION`; operator and hardware safety confirmations remain unresolved.
- Robot/motion/gripper/Home/trajectory/Hold/E-stop/AI publish/closed-loop operations: zero.

### Gate 1 graph recheck after connection

- ZED RGB became available at 1280×720 and roughly 31.5–38.4 Hz with source header timestamp.
- `/dsr01/joint_states` still had no publisher, `/doosan/current_pose` was absent, and both model health endpoints remained down.
- Configured the Phase 12 passive camera topic to the observed ZED rectified RGB topic. No runtime source was modified.
- State-changing services and command topics were discovered but never called/published. Live Shadow remained blocked rather than accepting camera-only data.

## Gate 1 live observation recovery

- Recovered seven service schemas using bundle-Python introspection because `ros2 interface show` fails on the installed generated `//` comment.
- Audited `dsr_controller2.cpp`: six allowlisted callbacks call only SDK getters or copy monitor state. Added Phase 12-only allowlisted query and state canonicalization modules.
- Sent one request to each of six verified getters: control mode, control space and last alarm succeeded; current TCP, robot state and robot mode timed out.
- Before isolated TCP retry, getter services and the JointState publisher disappeared. The retry sent no request because service discovery failed. Applied the communication-loss stop condition.
- Measured TCP and stationary synchronized samples: 0. Offline tests: 18/18 passed.
- Robot/motion/gripper/Home/trajectory/Hold/E-stop/AI publish/closed-loop operations remain zero.

## Gate 1 Doosan 30-second stability audit

- Passive subscriber/graph monitor ran for 30.0107 s with no service client or publisher.
- Received 1,196 valid JointState messages at 99.9997 Hz within an 11.9500 s timestamp span (39.82% wall-duration coverage); timestamps and finite position/velocity were valid, effort remained unsupported.
- Getter services and publisher endpoint were present in 31/31 polls, but `dsr_controller2` and `joint_state_broadcaster` nodes were each absent in 7/31 polls.
- Stability gate failed as `DOOSAN_STATE_STACK_UNSTABLE`; no getter was called and no stationary observation was started.
- Robot/motion/gripper/Home/trajectory/Hold/E-stop/AI publish/closed-loop operations remain zero.
