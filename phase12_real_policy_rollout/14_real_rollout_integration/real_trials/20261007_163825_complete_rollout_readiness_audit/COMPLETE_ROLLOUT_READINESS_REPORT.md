# Complete read-only comparison

**Latest verdict: MINIMUM_MOTION_BLOCKED_MULTIPLE.** A PASS / B FAIL / C JointState PASS / D FAIL. Current C camera selected-frame validation FAIL (1/147), action step sanity FAIL (4/147), GPU identity/inference PASS. TCP/hardware/manual blockers remain. **Physical commands0, getters0.** See [detailed source, results and limitations](SOURCE_HARDWARE_AND_LIMITATIONS.md) for service-flag correction and failed follow-up discovery, process restoration and59-test result.

```json
{
  "verdict": "MINIMUM_MOTION_BLOCKED_MULTIPLE",
  "jointstate_classification": "JOINTSTATE_STILL_INCONCLUSIVE",
  "run_results": [
    "PASS",
    "FAIL",
    "PASS",
    "FAIL"
  ],
  "shadow": {
    "predictions": 147,
    "camera_failure": 1,
    "max_selected_age": 0.5190789611078799,
    "model_failures": 0,
    "action_anomalies": 4,
    "translation_median": 3.208621061441586,
    "translation_max": 4.613487840673505,
    "rotation_max": 3.7395391096315937,
    "jointstate": "PASS",
    "status": "FAIL",
    "physical_commands": 0
  },
  "blockers": [
    "CURRENT_CAMERA_FRESHNESS_FAILURE_1_OF_147",
    "RAW_TRANSLATION_STEP_EXCEEDANCE_4_OF_147",
    "TCP_OFFSET_AND_CURRENT_POSE_UNVERIFIED",
    "HARDWARE_STATE_UNKNOWN",
    "MANUAL_CONFIRMATION_REQUIRED",
    "JOINTSTATE_RUNTIME_FAIL"
  ],
  "physical_commands": 0,
  "getter_requests": 0,
  "next_allowed_stage": "BLOCKED"
}
```

One persistent JointState subscriber across feature-flag runs. Each separate experimental condition has new FIRST_FRESH +10 s warm-up and fixed runtime; failed windows never restarted. Driver preserved. Camera/model restored after minimal run. Perturbation windows30 s are shorter than A-D60 s. No getter, motion, setter, gripper, Hold/Stop clients/publishers.
