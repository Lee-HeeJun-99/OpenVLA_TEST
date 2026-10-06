# JointState root-cause — not fixed

## Exact gap provenance

gap_events_precise.json contains both prior-loaded and new low-load BE/reliable/rosbag message indices, source before/after, receive times, wall times. Several >=3s events occur beyond index1 and recur during the connection window: not simply one cached first sample. Alternating OLD_HEADER_RECEIVE_AFTER_WAIT and SOURCE_JUMP_BURST_RECEIVE patterns must be kept separate. A source jump with a millisecond receive gap is not itself a3s subscriber receive stall, but preceding old-header wait events also exist.

## Timestamp generation — source verified

Installed joint_state_broadcaster2.53.1 counterpart source update(time,period): header.stamp=time; it reads numeric state interfaces and uses RealtimePublisher trylock()/unlockAndPublish(). Official source: https://github.com/ros-controls/ros2_controllers/blob/2.53.1/joint_state_broadcaster/src/joint_state_broadcaster.cpp lines325–355.
Installed controller_manager2.54.0 counterpart loop passes cm->now() to read/update/write. Official source: https://github.com/ros-controls/ros2_control/blob/2.54.0/controller_manager/src/ros2_control_node.cpp lines108–127. Package-version source correspondence is established; exact running binary was not decompiled against these sources in this test.
Thus header is host/node ROS-clock update time, not the controller RT feedback timestamp. Source gap alone cannot prove controller network packet generation stalled.

Local DRHWInterface::read line351 onwards calls Drfl.read_data_rt() and copies actual_joint_position/velocity into state interfaces (degree→radian); it does not assign JointState header. Vendor read_data_rt() internal transport/mutex code remains unavailable. The local real read null check logs but still dereferences data; a potential null bug, not a demonstrated cause here and not modified.

## Blocking candidates /3s search

Installed realtime_publisher.hpp: update-side trylock checks turn/mutex; publishingLoop waits on condition variable, copies outgoing, releases lock, then publisher_->publish(outgoing). Until publisher loop returns to next REALTIME turn, updates may skip serialization. A blocked rcl/rmw publish could cause old outgoing header followed by a fresh jump, matching observed pattern; this is a candidate, not runtime proof. Other candidates include hardware/read/update lock stalls and common DDS loss. No exact recurring3.07s logic was found in visible Doosan real read/update code. Startup500ms/1000ms waits and enum3000 values are not connected to current events. Prior client getter timeout3s was client-side and no getter ran here.

## Controlled low-load comparison

ZED launch3020648/OpenVLA3020619/RViz3017580 were SIGINT-stopped after PID/source validation; driver3017586 unchanged. Original parent launch preserved. Sixty-second JointState-only gate still failed with~3.07s max gap, no invalid values/duplicates/regressions. Removing these loads alone did not fix it; this does not rule out DDS/publish blocking or internal driverCPU. No claim of HOST_LOAD_RELATED as a confirmed cause.

Driver log delta during low-load trial was empty: no Skip-dt event can be aligned to these gaps. Static3s matches do not prove causal timing. Host raw stats/ROS-clock vsreceive-monotonic domains kept separate.

## Classification / next instrumentation

JOINTSTATE_STILL_INCONCLUSIVE. No speculative driver/core patch or restart was performed. Next safe evidence, if pursued: timestamps at cm read/update entry/exit; DRHWInterface read_data_rt entry/exit; broadcaster trylock success/failure; RealtimePublisher outgoing-copy→rcl/rmw publish entry/exit. Existing hardware traces require actual runtime enable evidence, not string presence. No current root-cause fix can honestly be declared.
