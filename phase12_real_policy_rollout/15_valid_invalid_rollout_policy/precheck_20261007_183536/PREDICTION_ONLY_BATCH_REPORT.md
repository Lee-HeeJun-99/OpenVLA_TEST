# Live prediction-only batch

Real camera, JointState and OpenVLA; persistent subscribers. No command clients or sinks.

Normal completion has runtime_status=VALID, status=null and task_success=null; no physical task outcome.
INVALID is terminal; no further inference is dispatched. Already in-flight replies are retained separately.
Raw translation/rotation hard-limit exceedances terminate the trial. TCP-dependent workspace, gripper phase and
motion safety remain unverified; runtime VALID is not motion readiness. Startup/warm-up samples are preserved.

```json
{
  "total_attempts": 1,
  "valid_trials": 0,
  "invalid_trials": 1,
  "invalid_rate": 1.0,
  "invalid_reason_counts": {
    "joint_state_receive_timeout": 1
  },
  "invalid_category_counts": {
    "INVALID_JOINTSTATE_GAP": 1
  },
  "task_success_rate": null,
  "physical_commands": 0,
  "verdict": "PREDICTION_ONLY_BATCH_UNSTABLE",
  "target_valid_trials": 1,
  "max_attempts": 1,
  "trial_duration_s": 20.0,
  "model_failure_count": 0,
  "fatal_error": null,
  "command_clients_created": 0,
  "dispatch_after_invalid": 0,
  "task_success": null,
  "classification_errors": 0,
  "next_stage": "BLOCKED",
  "motion_authorized": false,
  "remaining_motion_gates": [
    "TCP_TOOL_CONTRACT",
    "HARDWARE_STATE",
    "MANUAL_SAFETY_CONFIRMATIONS"
  ]
}
```
