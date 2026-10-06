# Static topic/service/action map

| Function | Interface | Type | Producer | Consumer | Static status |
|---|---|---|---|---|---|
| ZED RGB | `/zed/zed_node/rgb/color/rect/image` | `sensor_msgs/Image` | ZED | camera_adapter | resolution runtime-only |
| Model RGB | `/vla/image_rgb` | `sensor_msgs/Image` | camera_adapter | inference | header preserved |
| Joint state | `/dsr01/joint_states` | `sensor_msgs/JointState` | Doosan | OFT inference | source stamp available |
| TCP pose | `/doosan/current_pose` | `Float64MultiArray` | DoosanBridge | ActionAdapter | receive-time only |
| Raw action | `/vla/raw_action` | `Float64MultiArray` | inference | ActionAdapter | K=1 publication contract |
| Target pose | `/vla/target_pose` | `Float64MultiArray` | ActionAdapter | DoosanBridge | robot command path |
| Gripper command | `/vla/gripper_open` | `Bool` | ActionAdapter | DoosanBridge/OFT | command, not measurement |
| Enable | `/vla/enable` | `Bool` | EpisodeManager | command nodes | triggers initial open |
| Emergency stop request | `/vla/emergency_stop` | `Bool` | EpisodeManager/operator | DoosanBridge | software request only |
| Current pose service | `/dsr01/aux_control/get_current_posx` | `GetCurrentPosx` | controller | DoosanBridge | read service, not called here |
| Move line | `/dsr01/motion/move_line` | `MoveLine` | controller | DoosanBridge | prohibited |
| Move blending | `/dsr01/motion/move_blending` | `MoveBlending` | controller | DoosanBridge | prohibited |
| Move stop | `/dsr01/motion/move_stop` | `MoveStop` | controller | DoosanBridge | prohibited in this stage |
| Tool output | `/dsr01/io/set_tool_digital_output` | `SetToolDigitalOutput` | controller | DoosanBridge | prohibited |
| Servo stream | `/dsr01/servol_stream` | `ServolStream` | DoosanBridge | controller | prohibited |
| Speed stream | `/dsr01/speedl_stream` | `SpeedlStream` | DoosanBridge | controller | prohibited |
| Hold acknowledgement | `NOT_FOUND` | `NOT_FOUND` | | | blocker |
| Hardware E-stop status | `NOT_FOUND` | `NOT_FOUND` | | | local operator/controller required |
| Measured gripper state | `NOT_FOUND` | `NOT_FOUND` | | | command state must remain separate |
