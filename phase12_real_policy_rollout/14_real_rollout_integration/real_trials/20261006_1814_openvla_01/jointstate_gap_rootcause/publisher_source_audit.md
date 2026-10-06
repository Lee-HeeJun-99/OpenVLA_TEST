# JointState publisher provenance

Installed package: ros-humble-joint-state-broadcaster 2.53.1-1jammy.20260505.183509. Local library: /opt/ros/humble/lib/libjoint_state_broadcaster.so. Upstream matching-version source: https://github.com/ros-controls/ros2_controllers/blob/2.53.1/joint_state_broadcaster/src/joint_state_broadcaster.cpp (distribution patches/binary equivalence not independently proved).

on_configure creates joint_states using SystemDefaultsQoS. update reads state interfaces; on successful realtime publisher trylock, header.stamp is assigned the update argument `time`, then unlockAndPublish is called. A failed trylock can omit publication. This is a ros2_control update clock stamp, NOT a controller feedback timestamp or DRFL RT packet timestamp. A source-header jump cannot alone prove controller packet loss.

Local /home/ubuntu/robot_ws/src/doosan-robot2/dsr_hardware2/src/dsr_hw_interface2.cpp:350–373: DRHWInterface::read(real) calls Drfl.read_data_rt(), copies actual_joint_position/velocity degree values to radian state interfaces. It does not supply a controller timestamp to JointState. Existing instrumentation READ_CYCLE_GAP/READ_DATA_RT_START/END/STATE_INTERFACES_UPDATED is conditional on trace settings. No trace setting, driver code, QoS publisher setting or controller lifecycle was modified for this test.

Configured controller_manager update_rate: 100 Hz in dsr_controller2/config/dsr_controller2.yaml. Path: controller manager read → hardware state interfaces → broadcaster update → realtime publisher worker → RMW/DDS → subscribers/bag.

Targeted source search found no 3000-ms/3-second periodic wait in these local publisher/hardware-read snippets. Historical query_doosan_state_once.py uses a 3.0-second CLIENT timeout; that script was not run. DRFC OPERATION_SERVER_START=3000 is an enum, not timeout. Vendor binaries/threads or middleware may contain other waits; the observed duration alone does not link them causally. RT read, update, realtime publish worker and middleware traces are needed to distinguish them if all receivers share a gap.

Initial graph query during this attempt showed publisher count 0 while driver PID 3017586 persisted; prior query showed RELIABLE/TRANSIENT_LOCAL. Audit RELIABLE and bag request TRANSIENT_LOCAL, depth1000; BEST_EFFORT uses VOLATILE depth5, matching the earlier sensor-data observer. No QoS tuning of the driver was performed. Subscriber receive order is locally numbered, not a fabricated publisher sequence number.
