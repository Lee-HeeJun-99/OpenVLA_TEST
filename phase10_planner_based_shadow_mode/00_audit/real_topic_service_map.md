# Real topic/service/action map (static audit)

Source snapshot: Git `86eaa9632d651eb907332334d02f32c1461850d7`, with local modifications/untracked runtime files. This is a source/config audit; no ROS graph was queried.

| 기능 | Topic/Service/Action | Message Type | Publisher | Subscriber | 예상 Rate | Clock Source | Source File |
|---|---|---|---|---|---:|---|---|
| Camera raw | `/zed/zed_node/rgb/color/rect/image` | `sensor_msgs/msg/Image` | ZED node (external) | `camera_adapter` | NOT_FOUND | message header | `camera_adapter_node.py` |
| Camera | `/vla/image_rgb` | `sensor_msgs/msg/Image` | `camera_adapter` | OpenVLA/OFT inference | NOT_FOUND | preserved input header | `camera_adapter_node.py`, `openvla_inference_node.py` |
| Joint State | `/dsr01/joint_states` | `sensor_msgs/msg/JointState` | external Doosan driver | OFT inference | NOT_FOUND | message header; callback freshness also monotonic | `openvla_oft_inference_node.py`, `runtime_oft.yaml` |
| EE Pose | `/doosan/current_pose` | `std_msgs/msg/Float64MultiArray` `[mm,deg]` | `doosan_bridge` | action adapter/replay | configured poll 20 Hz | unstamped message; poll uses node timer | `doosan_bridge_node.py` |
| Gripper State | `/vla/gripper_open` | `std_msgs/msg/Bool` | action adapter | bridge and OFT | action-dependent | unstamped; OFT receipt monotonic | `action_adapter_node.py`, `openvla_oft_inference_node.py` |
| Planner Target | `/vla/target_pose` | `std_msgs/msg/Float64MultiArray` absolute `[mm,deg]` | action adapter/replay/teleop | bridge | input-dependent | unstamped | `action_adapter_node.py`, `doosan_bridge_node.py` |
| Planner Raw Action | `NOT_FOUND` | `NOT_FOUND` | `NOT_FOUND` | `NOT_FOUND` | `NOT_FOUND` | `NOT_FOUND` | no dedicated planner source in package |
| Model Raw Action | `/vla/raw_action` | `std_msgs/msg/Float64MultiArray` 7-D | OpenVLA/OFT inference | action adapter | config minimum 0.1 s; OFT config 1 Hz intent needs runtime verification | unstamped; internal monotonic throttling | inference nodes |
| Executed Command | `/dsr01/servol_stream` or `/dsr01/speedl_stream` | `dsr_msgs2/msg/ServolStream` or `SpeedlStream` | `doosan_bridge` | external driver | configured 20 Hz | unstamped publish timer | `doosan_bridge_node.py` |
| Executed service command | `/dsr01/motion/move_line`, `/dsr01/motion/move_blending` | `dsr_msgs2/srv/MoveLine`, `MoveBlending` | bridge client | driver service | event-driven | request/response, no logged source stamp | `doosan_bridge_node.py` |
| Hold | `/dsr01/motion/move_stop` | `dsr_msgs2/srv/MoveStop` | bridge client | driver service | event-driven | request/response | `doosan_bridge_node.py` |
| Hold Ack | `NOT_FOUND` | `NOT_FOUND` | `NOT_FOUND` | `NOT_FOUND` | `NOT_FOUND` | `NOT_FOUND` | no explicit acknowledgement topic/state found |
| Home/Reset | `NOT_FOUND` | `NOT_FOUND` | `NOT_FOUND` | `NOT_FOUND` | `NOT_FOUND` | `NOT_FOUND` | package has episode state reset, not verified robot home motion |
| E-stop request | `/vla/emergency_stop` | `std_msgs/msg/Bool` | episode manager/replay/teleop | bridge | event-driven | unstamped | manager/bridge/replay/teleop nodes |
| E-stop physical state/recovery | `NOT_FOUND` | `NOT_FOUND` | `NOT_FOUND` | `NOT_FOUND` | `NOT_FOUND` | `NOT_FOUND` | not exposed in audited package |
| Gripper command to driver | `/dsr01/io/set_tool_digital_output` | `dsr_msgs2/srv/SetToolDigitalOutput` | bridge client | driver service | event/pulse driven | request/response | `doosan_bridge_node.py` |
| OpenVLA Output | `/vla/raw_action` | `std_msgs/msg/Float64MultiArray` | `openvla_inference` | action adapter | min-period configured | unstamped | `openvla_inference_node.py` |
| OFT Output | `/vla/raw_action` | `std_msgs/msg/Float64MultiArray` (chunks emitted through inherited path) | `openvla_oft_inference` | action adapter | OFT config-dependent | unstamped | `openvla_oft_inference_node.py` |

No ROS actions (`ActionClient`) were found. `/vla/gripper_open` is a command topic reused by OFT as a state proxy; it is not measured gripper feedback.
