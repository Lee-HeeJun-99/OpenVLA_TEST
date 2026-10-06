# Doosan state path audit

Runtime HEAD: `86eaa9632d651eb907332334d02f32c1461850d7`. The pre-existing dirty tree was preserved; no runtime file was modified.

Python introspection succeeded with the bundle Python (system Python lacked NumPy required by generated messages):

| Service | Request | Response |
|---|---|---|
| GetCurrentPosx | `ref:int8` | `task_pos_info: Float64MultiArray[]`, `success` |
| GetCurrentPose | `space_type:int8` | `pos:float64[6]`, `success` |
| GetRobotState | empty | `robot_state:int8`, `success` |
| GetRobotMode | empty | `robot_mode:int8`, `success` |
| GetLastAlarm | empty | `log_alarm:LogAlarm`, `success` |
| GetControlMode | empty | `control_mode:int8`, `success` |
| GetControlSpace | empty | `space:int8`, `success` |

`ros2 interface show dsr_msgs2/srv/GetCurrentPosx` fails because the installed generated interface text contains `// generated ...`; Humble's `rosidl_adapter` tries to parse `//` as a type and raises `InvalidResourceName`. The source `.srv` and generated Python types are present, so this is a CLI parser defect, not evidence that the service type is absent.

Source evidence is in `dsr_controller2/src/dsr_controller2.cpp`: `get_current_posx_cb` calls only `Drfl->get_current_posx(ref)`; `get_robot_state_cb`, `get_robot_mode_cb`, and `get_last_alarm_cb` call their matching SDK getters; control mode/space copy `g_stDrState.nActualMode/nActualSpace`. None of these callbacks calls motion, servo, force, IO, realtime-write, or set APIs.

`GetCurrentPosx(ref=0)` is documented as current task position in `DR_BASE`; response elements are x/y/z and A/B/C plus solution space. Existing runtime treats position as mm and A/B/C as degrees. Orientation order remains `UNRESOLVED_ROTATION_CONVENTION`. `get_desired_posx` is commanded/desired state and is explicitly excluded.

During the one-shot audit, the driver stopped being discoverable before TCP could be obtained. No stale prior TCP value was reused.
