# Final integrated read-only precheck

**Latest verdict: MINIMUM_MOTION_BLOCKED.** Runtime JointState and post-precheck FAIL (including post-warm-up3.06 s source gaps), camera short current sample PASS, OpenVLA current health/inference PASS, getters0, physical commands0. See [current source/evidence interpretation](CURRENT_SOURCE_FINDINGS.md) for gap timeline, TCP cache semantics, prior enum snapshots, limitations and54-test result.

```json
{
  "jointstate_runtime": "FAIL",
  "jointstate_post": "FAIL",
  "jointstate_before": {
    "phase": "RUNTIME_READY",
    "fault": null,
    "first_fresh_sample": 2246717.952253882,
    "discovery_delay_s": 10.629798683803529,
    "latest_receive_age_s": 0.00952609395608306,
    "source_gap": 0.009986639022827148,
    "receive_gap": 0.00947469612583518
  },
  "jointstate_after": {
    "phase": "RUNTIME_FAULT",
    "fault": "joint_state_receive_timeout",
    "first_fresh_sample": 2246717.952253882,
    "discovery_delay_s": 10.629798683803529,
    "latest_receive_age_s": 2.9211588138714433,
    "source_gap": 0.009991168975830078,
    "receive_gap": 0.011035810224711895
  },
  "camera": {
    "status": "PASS",
    "receive": 2246729.687300158,
    "age_s": 0.00023231562227010727,
    "encoding": "bgra8",
    "resolution": [
      1280,
      720
    ],
    "selected_frame_age_s": 0.43938358779996634,
    "selected_frame_failure": false
  },
  "model_health": "MODEL_HEALTH_PASS",
  "mode_state": {
    "mode": {
      "status": "NOT_EXECUTED",
      "endpoint": "/dsr01/system/get_robot_mode",
      "provenance": "UNKNOWN",
      "before": {
        "phase": "RUNTIME_FAULT",
        "fault": "joint_state_receive_timeout",
        "first_fresh_sample": 2246717.952253882,
        "discovery_delay_s": 10.629798683803529,
        "latest_receive_age_s": 0.9024647953920066,
        "source_gap": 0.009744644165039062,
        "receive_gap": 0.011764287948608398
      },
      "reason": "runtime_gate_failed"
    },
    "state": {
      "status": "NOT_EXECUTED",
      "endpoint": "/dsr01/system/get_robot_state",
      "provenance": "UNKNOWN",
      "before": {
        "phase": "RUNTIME_FAULT",
        "fault": "joint_state_receive_timeout",
        "first_fresh_sample": 2246717.952253882,
        "discovery_delay_s": 10.629798683803529,
        "latest_receive_age_s": 0.9033040590584278,
        "source_gap": 0.009744644165039062,
        "receive_gap": 0.011764287948608398
      },
      "reason": "runtime_gate_failed"
    },
    "system": {
      "status": "NOT_EXECUTED",
      "endpoint": "/dsr01/system/get_robot_system",
      "provenance": "UNKNOWN",
      "before": {
        "phase": "RUNTIME_FAULT",
        "fault": "joint_state_receive_timeout",
        "first_fresh_sample": 2246717.952253882,
        "discovery_delay_s": 10.629798683803529,
        "latest_receive_age_s": 0.9042126862332225,
        "source_gap": 0.009744644165039062,
        "receive_gap": 0.011764287948608398
      },
      "reason": "runtime_gate_failed"
    }
  },
  "tcp_classification": "TCP_FLANGE_ONLY",
  "tcp_contract_verified": false,
  "hardware_unknown": [
    "connection",
    "authority",
    "servo",
    "protective_stop",
    "emergency_stop",
    "physical_gripper_state"
  ],
  "getter_requests": 0,
  "tcp_tool_getter_requests": 0,
  "gripper_command_disabled": true,
  "minimum_motion_precheck": "MINIMUM_MOTION_BLOCKED",
  "physical_commands": 0,
  "next_allowed_stage": "BLOCKED"
}
```

No physical motion, setters, gripper, Hold/Stop or Cartesian getter. TCP/tool name getters not repeated. One persistent JointState subscriber throughout; raw source candidates are not verified TCP. Runtime100 ms/camera0.5 s thresholds unchanged. Mode/state/system are timestamped DRFL getter snapshots, not continuous safety telemetry.
