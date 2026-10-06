# JointState live recovery audit

## Latest interpretation

- Branch `lhj-research`, HEAD `f7eeb7129c9dda38afbd19ddc3742d30a642b99a`, initial worktree clean.
- CLI topic-info initially reported publisher count 0. Direct endpoint discovery after the subscriber phase reported one RELIABLE/TRANSIENT_LOCAL publisher, GID `01.0f.0b.ce.72.0b.38.39.00.00.00.00.00.00.46.03.00.00.00.00.00.00.00.00`; node/namespace were UNKNOWN. A persistent publisher identity/name was not established.
- Both 30-second subscribers (BEST_EFFORT/VOLATILE and RELIABLE/TRANSIENT_LOCAL) received 0 messages. Both are compatible with the observed offer. `topic hz` and `echo --once` produced no sample output before observation timeout.
- Both controller-manager read-only list commands waited for service discovery and were bounded at 12 seconds. They supplied no controller/hardware response. Controller and broadcaster active/inactive status therefore remains UNKNOWN, not INACTIVE.
- Subsequent **separate** 25-second rosbag-only phase recorded 467 JointState messages over 4.65997 seconds at 100.00237 Hz; source max gap 10.986633 ms, receive max gap 11.235484 ms, source gaps >=100 ms: 0. First sample arrived approximately 19.982 seconds after recorder `Recording...` log time. This delay is ROS/log-clock-derived, not a monotonic measurement.
- Bag receiving real messages proves the stream was not continuously absent during the entire audit. It does NOT prove QoS was the earlier cause: subscriber and bag measurements were at different times and do not constitute a simultaneous controlled comparison.
- Less than five seconds of valid bag coverage does not meet the required 20-second fresh gate. No `JOINTSTATE_RECOVERED` motion-readiness approval is issued.
- Driver PID3017586 was alive: STAT Sl+, CPU203%, 39 threads, RSS119104 KiB, memory0.3%. Process alive is not hardware feedback/connection verification.
- Only newly appended driver logs were saved: two Skip-dt warnings at ROS times1791282578.0667/2578.0694. No new disconnect/move/set-mode callback was observed in these appended bytes. Startup warnings are excluded. Absence of a disconnection event does not prove connection.
- Source audit: `OnMonitoringStateCB`/`OnMonitoringAccessControlCB` cache state internally; `OnDisConnected` publishes an event (not continuous affirmative status). Selected RT data topics are config-disabled (`use_rt_topic_pub: false`). No current numeric hardware-interface read-value/freshness API was established. Hardware feedback and controller connection remain UNKNOWN; no unsafe robot getter was called.
- Verdict **STILL_INCONCLUSIVE**. Publisher can produce messages, but live continuity and discovery/controller-state reliability remain unverified. Next allowed stage **BLOCKED**.
- Read-only source evidence: installed ros2controlcli list verbs call `ListControllers`/`ListHardwareInterfaces` only. No activate/deactivate/load/switch/setter or robot motion service was requested. Compile check PASS; real measurements completed. No full regression suite claimed.
- All robot motion/gripper/Home/trajectory/Hold/Stop/rollout calls: **0**. Driver unchanged; only this audit's observer/CLI/recorder processes were terminated after their time limits.

The embedded initial summary below is preserved; the updated machine-readable summary includes supplemental bag continuity statistics above.

```json
{
  "subscribers": {
    "best_effort": {
      "count": 0,
      "first_receive_delay": null,
      "receive_rate": 0,
      "max_source_gap": null,
      "max_receive_gap": null,
      "latest_age": null,
      "invalid": 0,
      "duplicate": 0,
      "regression": 0,
      "recent_20s_pass": false
    },
    "reliable": {
      "count": 0,
      "first_receive_delay": null,
      "receive_rate": 0,
      "max_source_gap": null,
      "max_receive_gap": null,
      "latest_age": null,
      "invalid": 0,
      "duplicate": 0,
      "regression": 0,
      "recent_20s_pass": false
    }
  },
  "publishers": [
    {
      "name": "_NODE_NAME_UNKNOWN_",
      "namespace": "_NODE_NAMESPACE_UNKNOWN_",
      "gid": [
        1,
        15,
        11,
        206,
        114,
        11,
        56,
        57,
        0,
        0,
        0,
        0,
        0,
        0,
        70,
        3,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0
      ],
      "type": "sensor_msgs/msg/JointState",
      "qos": "QoSProfile(history=HistoryPolicy.UNKNOWN, depth=0, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL, lifespan=Infinite, deadline=Infinite, liveliness=LivelinessPolicy.AUTOMATIC, liveliness_lease_duration=Infinite, avoid_ros_namespace_conventions=False)"
    }
  ],
  "rosbag_count": 467,
  "rosbag_phase_elapsed": 25.464036878198385,
  "rosbag_receive_rate": 100.00236662253131,
  "rosbag_first_receive_delay": "UNVERIFIED_CLOCK_DOMAIN_ALIGNMENT",
  "classification": "STILL_INCONCLUSIVE",
  "hardware_feedback": "UNKNOWN",
  "controller_connection": "UNKNOWN",
  "physical_commands": 0,
  "next_allowed_stage": "BLOCKED"
}
```

Controller queries are read-only; timeout is not proof of inactive controllers. Interface lists enumerate names/claims, not numeric values or freshness. No read-only numeric hardware interface value source was established. Publisher-present/not-publishing classification means no live samples observed by both QoS, CLI echo/hz and separate rosbag; it does not prove publish() was never called. Event silence cannot prove controller connected. No robot getter, motion, setter, restart or activation was performed. Bag timestamps are ROS/system receive time, not local monotonic.
