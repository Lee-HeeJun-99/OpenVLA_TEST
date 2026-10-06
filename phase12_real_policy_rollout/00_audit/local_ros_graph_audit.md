# Gate 1 local ROS graph audit

Audit time: 2026-10-02 KST. Commands were read-only: `ros2 node list --no-daemon`, `ros2 topic list -t --no-daemon`, `ros2 service list -t --no-daemon`, `ros2 action list`, `ros2 topic info -v /joint_states --no-daemon`, and a bounded passive `ros2 topic hz /joint_states`.

Observed graph:

- Node: `/robot_state_publisher` only.
- Topics initially listed: `/joint_states`, `/parameter_events`, `/robot_description`, `/rosout`, `/tf`, `/tf_static`.
- A subsequent query reported `/joint_states` as unknown/not currently published; no rate sample was received.
- Services were parameter services of `robot_state_publisher` only.
- No actions were listed. (`ros2 action list -t --no-daemon` was unsupported by this CLI, so it was rerun as `ros2 action list`.)

| Requested interface | Exists | Type/rate/publisher | Classification | Shadow usable |
|---|---:|---|---|---:|
| ZED RGB image | no | NOT_FOUND | camera feedback | no |
| `/vla/image_rgb` | no | NOT_FOUND | processed camera | no |
| `/dsr01/joint_states` | no | NOT_FOUND | expected measured feedback | no |
| `/doosan/current_pose` | no | NOT_FOUND | expected measured feedback, headerless in audited code | no |
| `/vla/raw_action` | no | NOT_FOUND | AI command-path topic | no |
| `/vla/target_pose` | no | NOT_FOUND | command | no |
| `/vla/gripper_open` | no | NOT_FOUND | command, never measured feedback | no |
| `/vla/emergency_stop` | no | NOT_FOUND | stop command | no |
| `/dsr01/servol_stream` | no | NOT_FOUND | motion command | no |
| `/dsr01/speedl_stream` | no | NOT_FOUND | motion command | no |

No echo, publish, service call, action goal, parameter write, or launch was performed. Because required observation topics were absent, the optional 10–30 s subscriber-only recorder was **NOT_EXECUTED**.

## Recheck after hardware connection

The graph was rechecked read-only after the user connected the system:

- ZED RGB `/zed/zed_node/rgb/color/rect/image`: `sensor_msgs/msg/Image`, one publisher (`/zed/zed_node`), reliable/volatile QoS, approximately 31.5–38.4 Hz during the bounded sample.
- CameraInfo confirmed 1280×720, frame `zed_left_camera_frame_optical`, and a source header timestamp.
- `/dsr01/joint_states`: type exists but publisher count was 0 (only `robot_state_publisher` subscriber); no rate sample.
- `/doosan/current_pose`: `NOT_FOUND`.
- `/vla/image_rgb`, raw action, target pose, gripper and emergency-stop topics: `NOT_FOUND`.
- `/dsr01/speedl_stream`: command subscriber exists, publisher count 0. No publication was made.
- `/dsr01/servol_stream`: not active at the detailed-query time.
- Doosan services are discoverable, including state queries and motion/system mutation services. None was called.

This is a partial connection: camera input is usable, but synchronized robot state/TCP pose and prediction servers are unavailable. Subscriber-only Shadow validation remains `NOT_EXECUTED` rather than logging camera-only records as valid samples.

### Second connection recheck

- ZED RGB remained active, with a bounded observed rate of approximately 35.3–46.3 Hz.
- `/dsr01/joint_states` again had publisher count 0 and yielded no rate samples.
- `/dsr01/dynamic_joint_states` existed with one anonymous reliable/transient-local publisher, but yielded no continuous rate sample; it is not accepted as the required timestamped 5 Hz measured JointState without schema/sample validation.
- `/doosan/current_pose` and `/vla/image_rgb` remained absent.
- The visible node list still did not include a Doosan state/bridge node that publishes measured TCP pose.

Conclusion remains: camera-only partial connection; live Shadow recorder is blocked.
