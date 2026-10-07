# Read-only minimum-motion precheck

Latest interpretation: see [SOURCE_AND_RESULT_INTERPRETATION.md](SOURCE_AND_RESULT_INTERPRETATION.md) and actual [driver file delta](driver_file_log_delta.txt). TCP/tool returned empty strings, not verified names/offsets. Mode1=AUTO, state1=STANDBY and system0=REAL are DRFL getter snapshots (DRIVER_REPORTED); they do not prove servo/authority/protection/continuous connection.51 integration-directory tests PASS. Minimum-motion BLOCKED, physical commands0.

```json
{
  "jointstate_runtime": "PASS",
  "before": {
    "phase": "RUNTIME_READY",
    "fault": null,
    "first_fresh_sample": 2246247.682536927,
    "discovery_delay_s": 10.841079744044691,
    "latest_receive_age_s": 0.006477667950093746,
    "source_gap": 0.009999275207519531,
    "receive_gap": 0.010019085835665464
  },
  "after": {
    "phase": "RUNTIME_READY",
    "fault": null,
    "first_fresh_sample": 2246247.682536927,
    "discovery_delay_s": 10.841079744044691,
    "latest_receive_age_s": 0.00467432476580143,
    "source_gap": 0.009996891021728516,
    "receive_gap": 0.010885538998991251
  },
  "getter_requests": 5,
  "active_tcp": "UNKNOWN",
  "active_tool": "UNKNOWN",
  "tcp_offset": "NOT_VERIFIED",
  "current_tcp_pose": "NOT_VERIFIED",
  "hardware": {
    "controller_connection": "UNKNOWN",
    "robot_mode": 1,
    "robot_state": 1,
    "authority": "UNKNOWN",
    "servo": "UNKNOWN",
    "protective_stop": "UNKNOWN",
    "emergency_stop": "UNKNOWN",
    "provenance": "CONTROLLER_REPORTED_FOR_SUCCESSFUL_RESPONSES_ONLY"
  },
  "camera_passive_samples": 1600,
  "previous_shadow": "HISTORICAL_PASS_NOT_CURRENT_MOTION_GATE",
  "gripper_command_disabled": true,
  "gripper_pulse_disabled": true,
  "model_gripper_ignored": true,
  "minimum_motion_precheck": "MINIMUM_MOTION_BLOCKED",
  "physical_commands": 0,
  "next_allowed_stage": "BLOCKED"
}
```

Same JointState subscriber retained through startup, all conditional getter calls and10 s post-call observation. No motion/setter/Hold/Stop client created. Unknown safety states are not PASS. Search results are candidates, not current TCP binding.
