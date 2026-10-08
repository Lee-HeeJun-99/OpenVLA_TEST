# JointState startup/warmup validation

```json
{
  "verdict": "STARTUP_ARTIFACT_ONLY_RUNTIME_PASS",
  "started_monotonic": 2255853.536896712,
  "first_fresh_sample_by_subscriber": {
    "best_effort": 2255853.562793433,
    "reliable": 2255853.56302494
  },
  "shared_first_fresh_sample": 2255853.56302494,
  "startup_duration_s": 0.026128227822482586,
  "warmup_duration_s": 10,
  "runtime_t0": 2255863.56302494,
  "runtime_t1": 2255893.56302494,
  "runtime_duration_s": 30,
  "runtime_gate": "JOINTSTATE_RUNTIME_CLEAN_PASS",
  "phases": {
    "STARTUP_DISCOVERY_PHASE": {
      "best_effort": {
        "count": 1,
        "coverage_s": 0,
        "rate_hz": 0,
        "latest_receive_age_s": 40.00027761468664,
        "max_source_gap_s": null,
        "max_receive_gap_s": null,
        "duplicate": 0,
        "regression": 0,
        "invalid_position": 0,
        "invalid_velocity": 0,
        "missing_joints": 0,
        "event_ge_100ms": 0,
        "pass_runtime": false
      },
      "reliable": {
        "count": 0,
        "coverage_s": 0,
        "rate_hz": 0,
        "latest_receive_age_s": null,
        "max_source_gap_s": null,
        "max_receive_gap_s": null,
        "duplicate": 0,
        "regression": 0,
        "invalid_position": 0,
        "invalid_velocity": 0,
        "missing_joints": 0,
        "event_ge_100ms": 0,
        "pass_runtime": false
      }
    },
    "POST_DISCOVERY_WARMUP": {
      "best_effort": {
        "count": 1000,
        "coverage_s": 9.990145722869784,
        "rate_hz": 99.99854133389216,
        "latest_receive_age_s": 30.00031244708225,
        "max_source_gap_s": 0.012760473,
        "max_receive_gap_s": 0.01273083733394742,
        "duplicate": 0,
        "regression": 0,
        "invalid_position": 0,
        "invalid_velocity": 0,
        "missing_joints": 0,
        "event_ge_100ms": 0,
        "pass_runtime": false
      },
      "reliable": {
        "count": 1001,
        "coverage_s": 9.999904986005276,
        "rate_hz": 100.00095014897498,
        "latest_receive_age_s": 30.000141121912748,
        "max_source_gap_s": 0.012760473,
        "max_receive_gap_s": 0.012719538062810898,
        "duplicate": 0,
        "regression": 0,
        "invalid_position": 0,
        "invalid_velocity": 0,
        "missing_joints": 0,
        "event_ge_100ms": 0,
        "pass_runtime": false
      }
    },
    "RUNTIME_CLEAN_WINDOW": {
      "best_effort": {
        "count": 3000,
        "coverage_s": 29.990246711764485,
        "rate_hz": 99.99917736000356,
        "latest_receive_age_s": 0.00011242600157856941,
        "max_source_gap_s": 0.011584388,
        "max_receive_gap_s": 0.011701610870659351,
        "duplicate": 0,
        "regression": 0,
        "invalid_position": 0,
        "invalid_velocity": 0,
        "missing_joints": 0,
        "event_ge_100ms": 0,
        "pass_runtime": true
      },
      "reliable": {
        "count": 2999,
        "coverage_s": 29.980275911744684,
        "rate_hz": 99.99907968910794,
        "latest_receive_age_s": 0.009907790925353765,
        "max_source_gap_s": 0.011584388,
        "max_receive_gap_s": 0.011802678927779198,
        "duplicate": 0,
        "regression": 0,
        "invalid_position": 0,
        "invalid_velocity": 0,
        "missing_joints": 0,
        "event_ge_100ms": 0,
        "pass_runtime": true
      }
    }
  },
  "publisher_discovery": [
    {
      "receive_monotonic": 2255853.561306373,
      "publishers": [
        {
          "name": "joint_state_broadcaster",
          "namespace": "/dsr01",
          "gid": [
            1,
            15,
            11,
            206,
            177,
            38,
            198,
            238,
            0,
            0,
            0,
            0,
            0,
            0,
            71,
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
          "qos": "QoSProfile(history=HistoryPolicy.UNKNOWN, depth=0, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL, lifespan=Infinite, deadline=Infinite, liveliness=LivelinessPolicy.AUTOMATIC, liveliness_lease_duration=Infinite, avoid_ros_namespace_conventions=False)"
        }
      ]
    },
    {
      "receive_monotonic": 2255854.563363965,
      "publishers": [
        {
          "name": "joint_state_broadcaster",
          "namespace": "/dsr01",
          "gid": [
            1,
            15,
            11,
            206,
            177,
            38,
            198,
            238,
            0,
            0,
            0,
            0,
            0,
            0,
            71,
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
          "qos": "QoSProfile(history=HistoryPolicy.UNKNOWN, depth=0, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL, lifespan=Infinite, deadline=Infinite, liveliness=LivelinessPolicy.AUTOMATIC, liveliness_lease_duration=Infinite, avoid_ros_namespace_conventions=False)"
        }
      ]
    },
    {
      "receive_monotonic": 2255855.572750319,
      "publishers": [
        {
          "name": "joint_state_broadcaster",
          "namespace": "/dsr01",
          "gid": [
            1,
            15,
            11,
            206,
            177,
            38,
            198,
            238,
            0,
            0,
            0,
            0,
            0,
            0,
            71,
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
          "qos": "QoSProfile(history=HistoryPolicy.UNKNOWN, depth=0, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL, lifespan=Infinite, deadline=Infinite, liveliness=LivelinessPolicy.AUTOMATIC, liveliness_lease_duration=Infinite, avoid_ros_namespace_conventions=False)"
        }
      ]
    },
    {
      "receive_monotonic": 2255856.573141398,
      "publishers": [
        {
          "name": "joint_state_broadcaster",
          "namespace": "/dsr01",
          "gid": [
            1,
            15,
            11,
            206,
            177,
            38,
            198,
            238,
            0,
            0,
            0,
            0,
            0,
            0,
            71,
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
          "qos": "QoSProfile(history=HistoryPolicy.UNKNOWN, depth=0, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL, lifespan=Infinite, deadline=Infinite, liveliness=LivelinessPolicy.AUTOMATIC, liveliness_lease_duration=Infinite, avoid_ros_namespace_conventions=False)"
        }
      ]
    },
    {
      "receive_monotonic": 2255857.582796713,
      "publishers": [
        {
          "name": "joint_state_broadcaster",
          "namespace": "/dsr01",
          "gid": [
            1,
            15,
            11,
            206,
            177,
            38,
            198,
            238,
            0,
            0,
            0,
            0,
            0,
            0,
            71,
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
          "qos": "QoSProfile(history=HistoryPolicy.UNKNOWN, depth=0, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL, lifespan=Infinite, deadline=Infinite, liveliness=LivelinessPolicy.AUTOMATIC, liveliness_lease_duration=Infinite, avoid_ros_namespace_conventions=False)"
        }
      ]
    },
    {
      "receive_monotonic": 2255858.582914159,
      "publishers": [
        {
          "name": "joint_state_broadcaster",
          "namespace": "/dsr01",
          "gid": [
            1,
            15,
            11,
            206,
            177,
            38,
            198,
            238,
            0,
            0,
            0,
            0,
            0,
            0,
            71,
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
          "qos": "QoSProfile(history=HistoryPolicy.UNKNOWN, depth=0, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL, lifespan=Infinite, deadline=Infinite, liveliness=LivelinessPolicy.AUTOMATIC, liveliness_lease_duration=Infinite, avoid_ros_namespace_conventions=False)"
        }
      ]
    },
    {
      "receive_monotonic": 2255859.592883889,
      "publishers": [
        {
          "name": "joint_state_broadcaster",
          "namespace": "/dsr01",
          "gid": [
            1,
            15,
            11,
            206,
            177,
            38,
            198,
            238,
            0,
            0,
            0,
            0,
            0,
            0,
            71,
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
          "qos": "QoSProfile(history=HistoryPolicy.UNKNOWN, depth=0, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL, lifespan=Infinite, deadline=Infinite, liveliness=LivelinessPolicy.AUTOMATIC, liveliness_lease_duration=Infinite, avoid_ros_namespace_conventions=False)"
        }
      ]
    },
    {
      "receive_monotonic": 2255860.592930372,
      "publishers": [
        {
          "name": "joint_state_broadcaster",
          "namespace": "/dsr01",
          "gid": [
            1,
            15,
            11,
            206,
            177,
            38,
            198,
            238,
            0,
            0,
            0,
            0,
            0,
            0,
            71,
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
          "qos": "QoSProfile(history=HistoryPolicy.UNKNOWN, depth=0, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL, lifespan=Infinite, deadline=Infinite, liveliness=LivelinessPolicy.AUTOMATIC, liveliness_lease_duration=Infinite, avoid_ros_namespace_conventions=False)"
        }
      ]
    },
    {
      "receive_monotonic": 2255861.593052272,
      "publishers": [
        {
          "name": "joint_state_broadcaster",
          "namespace": "/dsr01",
          "gid": [
            1,
            15,
            11,
            206,
            177,
            38,
            198,
            238,
            0,
            0,
            0,
            0,
            0,
            0,
            71,
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
          "qos": "QoSProfile(history=HistoryPolicy.UNKNOWN, depth=0, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL, lifespan=Infinite, deadline=Infinite, liveliness=LivelinessPolicy.AUTOMATIC, liveliness_lease_duration=Infinite, avoid_ros_namespace_conventions=False)"
        }
      ]
    },
    {
      "receive_monotonic": 2255862.60265987,
      "publishers": [
        {
          "name": "joint_state_broadcaster",
          "namespace": "/dsr01",
          "gid": [
            1,
            15,
            11,
            206,
            177,
            38,
            198,
            238,
            0,
            0,
            0,
            0,
            0,
            0,
            71,
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
          "qos": "QoSProfile(history=HistoryPolicy.UNKNOWN, depth=0, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL, lifespan=Infinite, deadline=Infinite, liveliness=LivelinessPolicy.AUTOMATIC, liveliness_lease_duration=Infinite, avoid_ros_namespace_conventions=False)"
        }
      ]
    },
    {
      "receive_monotonic": 2255863.602758106,
      "publishers": [
        {
          "name": "joint_state_broadcaster",
          "namespace": "/dsr01",
          "gid": [
            1,
            15,
            11,
            206,
            177,
            38,
            198,
            238,
            0,
            0,
            0,
            0,
            0,
            0,
            71,
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
          "qos": "QoSProfile(history=HistoryPolicy.UNKNOWN, depth=0, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL, lifespan=Infinite, deadline=Infinite, liveliness=LivelinessPolicy.AUTOMATIC, liveliness_lease_duration=Infinite, avoid_ros_namespace_conventions=False)"
        }
      ]
    },
    {
      "receive_monotonic": 2255864.602851355,
      "publishers": [
        {
          "name": "joint_state_broadcaster",
          "namespace": "/dsr01",
          "gid": [
            1,
            15,
            11,
            206,
            177,
            38,
            198,
            238,
            0,
            0,
            0,
            0,
            0,
            0,
            71,
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
          "qos": "QoSProfile(history=HistoryPolicy.UNKNOWN, depth=0, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL, lifespan=Infinite, deadline=Infinite, liveliness=LivelinessPolicy.AUTOMATIC, liveliness_lease_duration=Infinite, avoid_ros_namespace_conventions=False)"
        }
      ]
    },
    {
      "receive_monotonic": 2255865.602913163,
      "publishers": [
        {
          "name": "joint_state_broadcaster",
          "namespace": "/dsr01",
          "gid": [
            1,
            15,
            11,
            206,
            177,
            38,
            198,
            238,
            0,
            0,
            0,
            0,
            0,
            0,
            71,
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
          "qos": "QoSProfile(history=HistoryPolicy.UNKNOWN, depth=0, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL, lifespan=Infinite, deadline=Infinite, liveliness=LivelinessPolicy.AUTOMATIC, liveliness_lease_duration=Infinite, avoid_ros_namespace_conventions=False)"
        }
      ]
    },
    {
      "receive_monotonic": 2255866.612689038,
      "publishers": [
        {
          "name": "joint_state_broadcaster",
          "namespace": "/dsr01",
          "gid": [
            1,
            15,
            11,
            206,
            177,
            38,
            198,
            238,
            0,
            0,
            0,
            0,
            0,
            0,
            71,
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
          "qos": "QoSProfile(history=HistoryPolicy.UNKNOWN, depth=0, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL, lifespan=Infinite, deadline=Infinite, liveliness=LivelinessPolicy.AUTOMATIC, liveliness_lease_duration=Infinite, avoid_ros_namespace_conventions=False)"
        }
      ]
    },
    {
      "receive_monotonic": 2255867.613135736,
      "publishers": [
        {
          "name": "joint_state_broadcaster",
          "namespace": "/dsr01",
          "gid": [
            1,
            15,
            11,
            206,
            177,
            38,
            198,
            238,
            0,
            0,
            0,
            0,
            0,
            0,
            71,
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
          "qos": "QoSProfile(history=HistoryPolicy.UNKNOWN, depth=0, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL, lifespan=Infinite, deadline=Infinite, liveliness=LivelinessPolicy.AUTOMATIC, liveliness_lease_duration=Infinite, avoid_ros_namespace_conventions=False)"
        }
      ]
    },
    {
      "receive_monotonic": 2255868.622731447,
      "publishers": [
        {
          "name": "joint_state_broadcaster",
          "namespace": "/dsr01",
          "gid": [
            1,
            15,
            11,
            206,
            177,
            38,
            198,
            238,
            0,
            0,
            0,
            0,
            0,
            0,
            71,
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
          "qos": "QoSProfile(history=HistoryPolicy.UNKNOWN, depth=0, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL, lifespan=Infinite, deadline=Infinite, liveliness=LivelinessPolicy.AUTOMATIC, liveliness_lease_duration=Infinite, avoid_ros_namespace_conventions=False)"
        }
      ]
    },
    {
      "receive_monotonic": 2255869.62285827,
      "publishers": [
        {
          "name": "joint_state_broadcaster",
          "namespace": "/dsr01",
          "gid": [
            1,
            15,
            11,
            206,
            177,
            38,
            198,
            238,
            0,
            0,
            0,
            0,
            0,
            0,
            71,
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
          "qos": "QoSProfile(history=HistoryPolicy.UNKNOWN, depth=0, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL, lifespan=Infinite, deadline=Infinite, liveliness=LivelinessPolicy.AUTOMATIC, liveliness_lease_duration=Infinite, avoid_ros_namespace_conventions=False)"
        }
      ]
    },
    {
      "receive_monotonic": 2255870.633018716,
      "publishers": [
        {
          "name": "joint_state_broadcaster",
          "namespace": "/dsr01",
          "gid": [
            1,
            15,
            11,
            206,
            177,
            38,
            198,
            238,
            0,
            0,
            0,
            0,
            0,
            0,
            71,
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
          "qos": "QoSProfile(history=HistoryPolicy.UNKNOWN, depth=0, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL, lifespan=Infinite, deadline=Infinite, liveliness=LivelinessPolicy.AUTOMATIC, liveliness_lease_duration=Infinite, avoid_ros_namespace_conventions=False)"
        }
      ]
    },
    {
      "receive_monotonic": 2255871.642735035,
      "publishers": [
        {
          "name": "joint_state_broadcaster",
          "namespace": "/dsr01",
          "gid": [
            1,
            15,
            11,
            206,
            177,
            38,
            198,
            238,
            0,
            0,
            0,
            0,
            0,
            0,
            71,
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
          "qos": "QoSProfile(history=HistoryPolicy.UNKNOWN, depth=0, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL, lifespan=Infinite, deadline=Infinite, liveliness=LivelinessPolicy.AUTOMATIC, liveliness_lease_duration=Infinite, avoid_ros_namespace_conventions=False)"
        }
      ]
    },
    {
      "receive_monotonic": 2255872.642824572,
      "publishers": [
        {
          "name": "joint_state_broadcaster",
          "namespace": "/dsr01",
          "gid": [
            1,
            15,
            11,
            206,
            177,
            38,
            198,
            238,
            0,
            0,
            0,
            0,
            0,
            0,
            71,
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
          "qos": "QoSProfile(history=HistoryPolicy.UNKNOWN, depth=0, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL, lifespan=Infinite, deadline=Infinite, liveliness=LivelinessPolicy.AUTOMATIC, liveliness_lease_duration=Infinite, avoid_ros_namespace_conventions=False)"
        }
      ]
    },
    {
      "receive_monotonic": 2255873.642892969,
      "publishers": [
        {
          "name": "joint_state_broadcaster",
          "namespace": "/dsr01",
          "gid": [
            1,
            15,
            11,
            206,
            177,
            38,
            198,
            238,
            0,
            0,
            0,
            0,
            0,
            0,
            71,
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
          "qos": "QoSProfile(history=HistoryPolicy.UNKNOWN, depth=0, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL, lifespan=Infinite, deadline=Infinite, liveliness=LivelinessPolicy.AUTOMATIC, liveliness_lease_duration=Infinite, avoid_ros_namespace_conventions=False)"
        }
      ]
    },
    {
      "receive_monotonic": 2255874.652688118,
      "publishers": [
        {
          "name": "joint_state_broadcaster",
          "namespace": "/dsr01",
          "gid": [
            1,
            15,
            11,
            206,
            177,
            38,
            198,
            238,
            0,
            0,
            0,
            0,
            0,
            0,
            71,
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
          "qos": "QoSProfile(history=HistoryPolicy.UNKNOWN, depth=0, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL, lifespan=Infinite, deadline=Infinite, liveliness=LivelinessPolicy.AUTOMATIC, liveliness_lease_duration=Infinite, avoid_ros_namespace_conventions=False)"
        }
      ]
    },
    {
      "receive_monotonic": 2255875.652833665,
      "publishers": [
        {
          "name": "joint_state_broadcaster",
          "namespace": "/dsr01",
          "gid": [
            1,
            15,
            11,
            206,
            177,
            38,
            198,
            238,
            0,
            0,
            0,
            0,
            0,
            0,
            71,
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
          "qos": "QoSProfile(history=HistoryPolicy.UNKNOWN, depth=0, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL, lifespan=Infinite, deadline=Infinite, liveliness=LivelinessPolicy.AUTOMATIC, liveliness_lease_duration=Infinite, avoid_ros_namespace_conventions=False)"
        }
      ]
    },
    {
      "receive_monotonic": 2255876.652963191,
      "publishers": [
        {
          "name": "joint_state_broadcaster",
          "namespace": "/dsr01",
          "gid": [
            1,
            15,
            11,
            206,
            177,
            38,
            198,
            238,
            0,
            0,
            0,
            0,
            0,
            0,
            71,
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
          "qos": "QoSProfile(history=HistoryPolicy.UNKNOWN, depth=0, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL, lifespan=Infinite, deadline=Infinite, liveliness=LivelinessPolicy.AUTOMATIC, liveliness_lease_duration=Infinite, avoid_ros_namespace_conventions=False)"
        }
      ]
    },
    {
      "receive_monotonic": 2255877.652965953,
      "publishers": [
        {
          "name": "joint_state_broadcaster",
          "namespace": "/dsr01",
          "gid": [
            1,
            15,
            11,
            206,
            177,
            38,
            198,
            238,
            0,
            0,
            0,
            0,
            0,
            0,
            71,
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
          "qos": "QoSProfile(history=HistoryPolicy.UNKNOWN, depth=0, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL, lifespan=Infinite, deadline=Infinite, liveliness=LivelinessPolicy.AUTOMATIC, liveliness_lease_duration=Infinite, avoid_ros_namespace_conventions=False)"
        }
      ]
    },
    {
      "receive_monotonic": 2255878.662866623,
      "publishers": [
        {
          "name": "joint_state_broadcaster",
          "namespace": "/dsr01",
          "gid": [
            1,
            15,
            11,
            206,
            177,
            38,
            198,
            238,
            0,
            0,
            0,
            0,
            0,
            0,
            71,
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
          "qos": "QoSProfile(history=HistoryPolicy.UNKNOWN, depth=0, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL, lifespan=Infinite, deadline=Infinite, liveliness=LivelinessPolicy.AUTOMATIC, liveliness_lease_duration=Infinite, avoid_ros_namespace_conventions=False)"
        }
      ]
    },
    {
      "receive_monotonic": 2255879.662988602,
      "publishers": [
        {
          "name": "joint_state_broadcaster",
          "namespace": "/dsr01",
          "gid": [
            1,
            15,
            11,
            206,
            177,
            38,
            198,
            238,
            0,
            0,
            0,
            0,
            0,
            0,
            71,
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
          "qos": "QoSProfile(history=HistoryPolicy.UNKNOWN, depth=0, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL, lifespan=Infinite, deadline=Infinite, liveliness=LivelinessPolicy.AUTOMATIC, liveliness_lease_duration=Infinite, avoid_ros_namespace_conventions=False)"
        }
      ]
    },
    {
      "receive_monotonic": 2255880.672715787,
      "publishers": [
        {
          "name": "joint_state_broadcaster",
          "namespace": "/dsr01",
          "gid": [
            1,
            15,
            11,
            206,
            177,
            38,
            198,
            238,
            0,
            0,
            0,
            0,
            0,
            0,
            71,
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
          "qos": "QoSProfile(history=HistoryPolicy.UNKNOWN, depth=0, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL, lifespan=Infinite, deadline=Infinite, liveliness=LivelinessPolicy.AUTOMATIC, liveliness_lease_duration=Infinite, avoid_ros_namespace_conventions=False)"
        }
      ]
    },
    {
      "receive_monotonic": 2255881.672822772,
      "publishers": [
        {
          "name": "joint_state_broadcaster",
          "namespace": "/dsr01",
          "gid": [
            1,
            15,
            11,
            206,
            177,
            38,
            198,
            238,
            0,
            0,
            0,
            0,
            0,
            0,
            71,
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
          "qos": "QoSProfile(history=HistoryPolicy.UNKNOWN, depth=0, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL, lifespan=Infinite, deadline=Infinite, liveliness=LivelinessPolicy.AUTOMATIC, liveliness_lease_duration=Infinite, avoid_ros_namespace_conventions=False)"
        }
      ]
    },
    {
      "receive_monotonic": 2255882.672901314,
      "publishers": [
        {
          "name": "joint_state_broadcaster",
          "namespace": "/dsr01",
          "gid": [
            1,
            15,
            11,
            206,
            177,
            38,
            198,
            238,
            0,
            0,
            0,
            0,
            0,
            0,
            71,
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
          "qos": "QoSProfile(history=HistoryPolicy.UNKNOWN, depth=0, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL, lifespan=Infinite, deadline=Infinite, liveliness=LivelinessPolicy.AUTOMATIC, liveliness_lease_duration=Infinite, avoid_ros_namespace_conventions=False)"
        }
      ]
    },
    {
      "receive_monotonic": 2255883.68272523,
      "publishers": [
        {
          "name": "joint_state_broadcaster",
          "namespace": "/dsr01",
          "gid": [
            1,
            15,
            11,
            206,
            177,
            38,
            198,
            238,
            0,
            0,
            0,
            0,
            0,
            0,
            71,
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
          "qos": "QoSProfile(history=HistoryPolicy.UNKNOWN, depth=0, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL, lifespan=Infinite, deadline=Infinite, liveliness=LivelinessPolicy.AUTOMATIC, liveliness_lease_duration=Infinite, avoid_ros_namespace_conventions=False)"
        }
      ]
    },
    {
      "receive_monotonic": 2255884.682786578,
      "publishers": [
        {
          "name": "joint_state_broadcaster",
          "namespace": "/dsr01",
          "gid": [
            1,
            15,
            11,
            206,
            177,
            38,
            198,
            238,
            0,
            0,
            0,
            0,
            0,
            0,
            71,
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
          "qos": "QoSProfile(history=HistoryPolicy.UNKNOWN, depth=0, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL, lifespan=Infinite, deadline=Infinite, liveliness=LivelinessPolicy.AUTOMATIC, liveliness_lease_duration=Infinite, avoid_ros_namespace_conventions=False)"
        }
      ]
    },
    {
      "receive_monotonic": 2255885.682813048,
      "publishers": [
        {
          "name": "joint_state_broadcaster",
          "namespace": "/dsr01",
          "gid": [
            1,
            15,
            11,
            206,
            177,
            38,
            198,
            238,
            0,
            0,
            0,
            0,
            0,
            0,
            71,
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
          "qos": "QoSProfile(history=HistoryPolicy.UNKNOWN, depth=0, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL, lifespan=Infinite, deadline=Infinite, liveliness=LivelinessPolicy.AUTOMATIC, liveliness_lease_duration=Infinite, avoid_ros_namespace_conventions=False)"
        }
      ]
    },
    {
      "receive_monotonic": 2255886.682890167,
      "publishers": [
        {
          "name": "joint_state_broadcaster",
          "namespace": "/dsr01",
          "gid": [
            1,
            15,
            11,
            206,
            177,
            38,
            198,
            238,
            0,
            0,
            0,
            0,
            0,
            0,
            71,
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
          "qos": "QoSProfile(history=HistoryPolicy.UNKNOWN, depth=0, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL, lifespan=Infinite, deadline=Infinite, liveliness=LivelinessPolicy.AUTOMATIC, liveliness_lease_duration=Infinite, avoid_ros_namespace_conventions=False)"
        }
      ]
    },
    {
      "receive_monotonic": 2255887.682899371,
      "publishers": [
        {
          "name": "joint_state_broadcaster",
          "namespace": "/dsr01",
          "gid": [
            1,
            15,
            11,
            206,
            177,
            38,
            198,
            238,
            0,
            0,
            0,
            0,
            0,
            0,
            71,
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
          "qos": "QoSProfile(history=HistoryPolicy.UNKNOWN, depth=0, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL, lifespan=Infinite, deadline=Infinite, liveliness=LivelinessPolicy.AUTOMATIC, liveliness_lease_duration=Infinite, avoid_ros_namespace_conventions=False)"
        }
      ]
    },
    {
      "receive_monotonic": 2255888.683021937,
      "publishers": [
        {
          "name": "joint_state_broadcaster",
          "namespace": "/dsr01",
          "gid": [
            1,
            15,
            11,
            206,
            177,
            38,
            198,
            238,
            0,
            0,
            0,
            0,
            0,
            0,
            71,
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
          "qos": "QoSProfile(history=HistoryPolicy.UNKNOWN, depth=0, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL, lifespan=Infinite, deadline=Infinite, liveliness=LivelinessPolicy.AUTOMATIC, liveliness_lease_duration=Infinite, avoid_ros_namespace_conventions=False)"
        }
      ]
    },
    {
      "receive_monotonic": 2255889.692675526,
      "publishers": [
        {
          "name": "joint_state_broadcaster",
          "namespace": "/dsr01",
          "gid": [
            1,
            15,
            11,
            206,
            177,
            38,
            198,
            238,
            0,
            0,
            0,
            0,
            0,
            0,
            71,
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
          "qos": "QoSProfile(history=HistoryPolicy.UNKNOWN, depth=0, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL, lifespan=Infinite, deadline=Infinite, liveliness=LivelinessPolicy.AUTOMATIC, liveliness_lease_duration=Infinite, avoid_ros_namespace_conventions=False)"
        }
      ]
    },
    {
      "receive_monotonic": 2255890.692686422,
      "publishers": [
        {
          "name": "joint_state_broadcaster",
          "namespace": "/dsr01",
          "gid": [
            1,
            15,
            11,
            206,
            177,
            38,
            198,
            238,
            0,
            0,
            0,
            0,
            0,
            0,
            71,
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
          "qos": "QoSProfile(history=HistoryPolicy.UNKNOWN, depth=0, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL, lifespan=Infinite, deadline=Infinite, liveliness=LivelinessPolicy.AUTOMATIC, liveliness_lease_duration=Infinite, avoid_ros_namespace_conventions=False)"
        }
      ]
    },
    {
      "receive_monotonic": 2255891.692745188,
      "publishers": [
        {
          "name": "joint_state_broadcaster",
          "namespace": "/dsr01",
          "gid": [
            1,
            15,
            11,
            206,
            177,
            38,
            198,
            238,
            0,
            0,
            0,
            0,
            0,
            0,
            71,
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
          "qos": "QoSProfile(history=HistoryPolicy.UNKNOWN, depth=0, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL, lifespan=Infinite, deadline=Infinite, liveliness=LivelinessPolicy.AUTOMATIC, liveliness_lease_duration=Infinite, avoid_ros_namespace_conventions=False)"
        }
      ]
    },
    {
      "receive_monotonic": 2255892.692804046,
      "publishers": [
        {
          "name": "joint_state_broadcaster",
          "namespace": "/dsr01",
          "gid": [
            1,
            15,
            11,
            206,
            177,
            38,
            198,
            238,
            0,
            0,
            0,
            0,
            0,
            0,
            71,
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
          "qos": "QoSProfile(history=HistoryPolicy.UNKNOWN, depth=0, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL, lifespan=Infinite, deadline=Infinite, liveliness=LivelinessPolicy.AUTOMATIC, liveliness_lease_duration=Infinite, avoid_ros_namespace_conventions=False)"
        }
      ]
    }
  ],
  "physical_commands": 0,
  "next_allowed_stage": "TCP_PRECHECK",
  "clock_domains": {
    "source": "ROS_HEADER",
    "receive_ros": "LOCAL_NODE_ROS_CLOCK",
    "receive_monotonic": "HOST_MONOTONIC",
    "receive_wall": "SYSTEM_WALL"
  },
  "clock_caveat": "Header-age compared only in ROS clock domain; no source-minus-monotonic arithmetic. Negative age samples never FIRST_FRESH_SAMPLE."
}
```

Startup/warmup raw samples and all gap events preserved. Fixed runtime T0+30s never restarts after gaps. Boundary-straddling >=100ms intervals arriving in runtime are conservatively counted as runtime failures. Runtime thresholds unchanged (100ms gap/500ms age). Both subscriber first-fresh clocks preserved; shared gate starts after the later first-fresh +10s. No getters/services/publishers/actions, driver restart, robot command or mode/tool/servo change. Rosbag omitted to minimize additional host load.
