# Hardware state readiness

Status: `BLOCKED_REQUIRES_LOCAL_OPERATOR_AND_HARDWARE`; motion readiness is false.

## Verified read-only observations

- Doosan nodes currently discovered: controller manager, `dsr_controller2`, joint-state broadcaster and robot-state publisher.
- `/dsr01/joint_states`: publisher present; the startup DDS transient is mitigated by a 2-second/180-sample readiness gate. Three gated 60-second runs each accepted 6,001 samples with no gap >=100 ms.
- `/dsr01/error`: `RobotError`, RELIABLE/VOLATILE event topic. The absence of an event during a short subscription is not proof of a safe state.
- `/dsr01/robot_disconnection`: `RobotDisconnection`, RELIABLE/VOLATILE event topic. Same limitation.
- `/dsr01/io/ctrl_box_digital_input_state`: UInt8 array endpoint exists, but no audited mapping identifies physical E-stop, protective stop or gripper sensors.
- ZED health sample reported no low-image-quality, low-lighting, depth or motion-sensor warnings.

## Safety state matrix

| Item | Status | Evidence/limitation |
|---|---|---|
| Joint feedback continuity | `PASS_WITH_READINESS_GATE` | Three gated 60-second runs |
| Control mode | `VERIFIED_SIGNAL` | prior getter: 3 / position |
| Control space | `VERIFIED_SIGNAL` | prior getter: 1 / joint |
| Last alarm | `UNVERIFIED_CURRENT_SAFETY` | prior zero response does not prove E-stop/protective-stop state |
| Robot state | `NOT_FOUND` | getter timed out; no retry performed |
| Robot mode | `NOT_FOUND` | getter timed out; no retry performed |
| Measured TCP | `NOT_FOUND` | getter timed out; no stale fallback allowed |
| Servo state | `NOT_FOUND` | no verified live signal |
| Protective stop | `NOT_FOUND` | event/error topics are insufficient as current-state proof |
| Hardware E-stop | `OPERATOR_CONFIRMED` | location/access/function attested; not controller telemetry |
| Hold acknowledgement | `NOT_FOUND` | no verified acknowledgement path |
| Measured gripper feedback | `NOT_FOUND` | command/DO state is not measured gripper position |
| Workspace/velocity/acceleration | `OPERATOR_CONFIRMED` | workspace, 20-profile and 30 s maximum approved |

No motion, gripper actuation, Home, Hold or E-stop call was made.

## Latest post-operator-check attempt

- A command-free JointState readiness probe was attempted before any getter.
- The first invocation used the active Conda Python 3.14 and failed before ROS initialization because Humble `rclpy` is Python 3.10; it created no ROS entity.
- The retry used `/usr/bin/python3`, waited 15.055 seconds and received zero JointState messages.
- `/dsr01/joint_states` still advertised one `joint_state_broadcaster` publisher; controller manager, controller, broadcaster, and the three getter endpoints remained discoverable.
- Because the data-plane gate failed, measured TCP, robot state and robot mode getters were not called.

Classification: `JOINTSTATE_ENDPOINT_PRESENT_BUT_NO_DATA`. Further progress is blocked until the broadcaster/controller again emits samples continuously.

## Follow-up revalidation after field report

The prior observation above is retained as historical evidence. A fresh check was performed after the local operator reported about 100 Hz feedback:

- Valid accepted window: 10.000 seconds, 1,001 messages, source rate 100.0005 Hz.
- Maximum accepted source gap: 12.024 ms; maximum accepted receive gap: 12.014 ms.
- Gaps >=50/100/500/1000 ms in the accepted window: 0/0/0/0.
- Position invalid: 0; velocity invalid: 0; missing canonical joint: 0.
- Wire order remained `joint_1,joint_2,joint_4,joint_5,joint_3,joint_6`; name-based reorder remains mandatory.
- Startup before readiness again contained the known ~3.06-second transient; it was excluded by the gate.

After this pass, `/dsr01/aux_control/get_current_posx(ref=0)` was called once. It timed out after 3.000606 seconds. No robot-state or robot-mode getter was called. Immediately afterwards, node/service/publisher endpoints remained discoverable, but a 15.054-second JointState probe received zero messages.

Classification: `GET_CURRENT_POSX_TIMEOUT_CORRELATED_WITH_FEEDBACK_LOSS`. This establishes temporal correlation, not causation. Stationary observation was not started.

## Current post-relaunch state

The historical getter-correlated failure above is retained. The current minimal
observer deliberately omits `dsr_controller2` and all getters. In this configuration:

- JointState passed three 60-second runs and a post-relaunch 60-second run at about
  100 Hz with no gap >=100 ms.
- An integrated 31.76-second stationary bag contained continuous JointState and ZED
  input, both with no gap >=100 ms.
- OpenVLA and OFT stationary prediction-only smoke tests both passed with every
  executed/delivered action null and every `command_issued` false.
- The local operator subsequently reported no unusual condition at the site.

These results clear the observation/model-data path. The operator also confirmed the
physical checklist and numerical bounds. Controller-readable protective-stop/servo/
Hold acknowledgement and measured gripper feedback remain unavailable.
