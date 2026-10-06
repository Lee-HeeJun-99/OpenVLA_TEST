# Servo/mode/authority source audit

Source: `/home/ubuntu/robot_ws/src/doosan-robot2`.

- `dsr_common2/include/DRFC.h` ROBOT_MODE_MANUAL=0, ROBOT_MODE_AUTONOMOUS=1; access-control GRANT=2, LOSS=3. `DRFS.h` confirms mode numeric interpretation.
- `dsr_controller2.cpp:get_robot_mode_cb` → DRFL get_robot_mode; `system/get_robot_mode` uses dsr_msgs2/srv/GetRobotMode. Code exists, but prior blocking behavior precludes silently adding polling. No getter was called in this task.
- Optional selected RT publisher at `/rt_topic/robot_mode`, `/rt_topic/robot_state` uses Float32MultiArray and read_data_rt. Subscriber adapter implemented; existing driver must already provide approved endpoints. No RT connect/start was performed.
- RobotState.msg includes disconnected, robot_state, access_control. actual_mode means position/torque, **not** manual/auto. Protective stop codes5/10 and emergency6/7 are interpreted as driver-reported states, not proof that a physical E-stop was tested.
- RobotDisconnection and RobotError publishers exist. Full RobotState message definition exists but no default full-state publisher was identified. No invented default topic is configured.
- No dedicated servo-enabled signal was identified. Classified `HARDWARE_SOURCE_NOT_AVAILABLE_IN_CURRENT_SOFTWARE_INTERFACE`; hardware source verification/operator policy remains required.

Adapter outputs contain connected, servo_enabled, robot_mode(operation mode), protective_stop, emergency_stop, motion_state, authority, timestamp, source and per-field provenance: DRIVER_REPORTED / OPERATOR_CONFIRMED / UNKNOWN. Operator values expire; absent/UNKNOWN values cannot authorize motion. Fake test states cannot authorize non-dry execution.
