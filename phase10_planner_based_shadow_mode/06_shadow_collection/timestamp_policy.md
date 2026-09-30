# Timestamp policy

Every observed sample stores `source_timestamp`, `receive_monotonic_timestamp`, `receive_ros_timestamp`, and an explicit `clock_domain`. Camera and JointState headers are source clocks. `/doosan/current_pose` has no header, so its `source_timestamp` is null and only subscriber receive clocks may be populated. Reference commands use their command-generation clock; model latency uses `perf_counter`/monotonic and is not subtracted from ROS or camera epochs.

Direct subtraction is permitted only when clock-domain identifiers are equal and their synchronization contract has been verified. Otherwise alignment uses frame/sequence association and the record is marked `CLOCK_DOMAIN_UNRESOLVED_NO_DIRECT_SUBTRACTION`. No timestamp is fabricated from another domain.
