# JointState startup/warmup validation

```json
{
  "verdict": "STARTUP_ARTIFACT_ONLY_RUNTIME_PASS",
  "started_monotonic": 2255748.622163284,
  "first_fresh_sample_by_subscriber": {
    "best_effort": 2255748.652740331,
    "reliable": 2255748.65292325
  },
  "shared_first_fresh_sample": 2255748.65292325,
  "startup_duration_s": 0.030759966000914574,
  "warmup_duration_s": 10,
  "runtime_t0": 2255758.65292325,
  "runtime_t1": 2255788.65292325,
  "runtime_duration_s": 30,
  "runtime_gate": "JOINTSTATE_RUNTIME_CLEAN_PASS",
  "phases": {
    "STARTUP_DISCOVERY_PHASE": {
      "best_effort": {
        "count": 1,
        "coverage_s": 0,
        "rate_hz": 0,
        "latest_receive_age_s": 40.01003981521353,
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
        "coverage_s": 9.98987797088921,
        "rate_hz": 100.00122152754163,
        "latest_receive_age_s": 30.01008395012468,
        "max_source_gap_s": 0.011274732,
        "max_receive_gap_s": 0.011332014109939337,
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
        "coverage_s": 9.999947319738567,
        "rate_hz": 100.00052680538955,
        "latest_receive_age_s": 30.00990957627073,
        "max_source_gap_s": 0.011274732,
        "max_receive_gap_s": 0.011371363885700703,
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
        "count": 3001,
        "coverage_s": 30.000012089964002,
        "rate_hz": 99.99995970013623,
        "latest_receive_age_s": 9.435601532459259e-05,
        "max_source_gap_s": 0.01095753,
        "max_receive_gap_s": 0.011782654095441103,
        "duplicate": 0,
        "regression": 0,
        "invalid_position": 0,
        "invalid_velocity": 0,
        "missing_joints": 0,
        "event_ge_100ms": 0,
        "pass_runtime": true
      },
      "reliable": {
        "count": 3000,
        "coverage_s": 29.989936545025557,
        "rate_hz": 100.0002115875582,
        "latest_receive_age_s": 0.010008484125137329,
        "max_source_gap_s": 0.01095753,
        "max_receive_gap_s": 0.01196177490055561,
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
      "receive_monotonic": 2255748.64667694,
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
      "receive_monotonic": 2255749.652776772,
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
      "receive_monotonic": 2255750.652788797,
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
      "receive_monotonic": 2255751.652921641,
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
      "receive_monotonic": 2255752.653030695,
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
      "receive_monotonic": 2255753.663187732,
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
      "receive_monotonic": 2255754.672836398,
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
      "receive_monotonic": 2255755.672860826,
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
      "receive_monotonic": 2255756.672875857,
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
      "receive_monotonic": 2255757.682667765,
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
      "receive_monotonic": 2255758.682810663,
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
      "receive_monotonic": 2255759.683212441,
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
      "receive_monotonic": 2255760.693296936,
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
      "receive_monotonic": 2255761.702781089,
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
      "receive_monotonic": 2255762.70278334,
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
      "receive_monotonic": 2255763.702959703,
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
      "receive_monotonic": 2255764.712942183,
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
      "receive_monotonic": 2255765.722709363,
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
      "receive_monotonic": 2255766.72305671,
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
      "receive_monotonic": 2255767.732786636,
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
      "receive_monotonic": 2255768.732998244,
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
      "receive_monotonic": 2255769.742694645,
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
      "receive_monotonic": 2255770.742821379,
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
      "receive_monotonic": 2255771.742874764,
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
      "receive_monotonic": 2255772.752824424,
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
      "receive_monotonic": 2255773.752958191,
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
      "receive_monotonic": 2255774.762721752,
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
      "receive_monotonic": 2255775.76283003,
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
      "receive_monotonic": 2255776.762906947,
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
      "receive_monotonic": 2255777.763040056,
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
      "receive_monotonic": 2255778.763130655,
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
      "receive_monotonic": 2255779.77277971,
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
      "receive_monotonic": 2255780.772915895,
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
      "receive_monotonic": 2255781.782735749,
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
      "receive_monotonic": 2255782.782850572,
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
      "receive_monotonic": 2255783.782928623,
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
      "receive_monotonic": 2255784.78319929,
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
      "receive_monotonic": 2255785.792968893,
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
      "receive_monotonic": 2255786.792979546,
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
      "receive_monotonic": 2255787.793018191,
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
