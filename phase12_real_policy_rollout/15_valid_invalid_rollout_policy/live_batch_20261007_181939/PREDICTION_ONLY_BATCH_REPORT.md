# Live prediction-only batch

## Final measured finding

Verdict: **PREDICTION_ONLY_BATCH_UNSTABLE**. Policy classification: **PASS**.
20 attempts / 0 runtime-valid trials / 20 INVALID / invalid rate 100%.
Every trial ended on `raw_translation_step_limit` (INVALID_SAFETY_REJECTION).
Target of 10 valid trials was NOT reached; no threshold was relaxed.

Baseline branch `lhj-research`, HEAD `21bfe424e08e6f68647c23cbf2026b0343cb4a93`.
An external commit during execution changed HEAD to
`d2fe86ca08c1ca3ba8a2f3f6675d8665951487c9`. Executed source hashes are
preserved in `environment.json` and remained unchanged at completion.

Real OpenVLA step8130 / K=1 / dim=7 / proprio=false identity and processor
validation passed. There were 38 actual live predictions; model failures 0,
NaN/Inf 0, close candidates 0. No fixture or synthetic task-success was used.

Observed selected-camera maximum age: **0.459322 s**; no camera invalidations.
Observed runtime JointState maximum source/receive gaps: **11.097 / 18.591 ms**;
no JointState invalidations. These are SHORT windows ending on action rejection,
not successful 20-second sensor/runtime trials. Runtime durations summed to
17.634 s over all attempts; startup and individual 10-second warm-ups are separate.
Consequently, neither a 20-second trial PASS nor long-duration stability is claimed.

VALID-only action distribution has **zero predictions**, so all its quantiles are
null. Separate INVALID diagnostics (not policy performance): translation median /
p95 / max **4.095 / 5.852 / 5.852 mm**; rotation median / p95 / max
**0.990 / 1.477 / 2.633 degrees**. Translation >4 mm: 20 predictions.
Inference latency median / p95 / max **0.399 / 0.439 / 0.441 s**.

Post-run artifact inspection found all five required files in every trial,
consistent INVALID/runtime_valid=false/task_success=null classifications,
and no recorded inference dispatch timestamp after that trial's invalid event.
All 38 requested predictions are accounted for; no physical command clients
were created and command_issued was false throughout.

Regression tests: **216 PASS / 0 FAIL / 0 SKIP** (100 core, 66 real integration,
22 prior analysis, 4 readiness, 24 policy including 4 new runtime-only tests).
Existing gimbal-lock warnings are warnings, not failures. `git diff --check`: PASS.

Next stage: **BLOCKED**. First review the current live translation-limit
exceedances using saved frames/actions. Do not raise the safety threshold simply
to reach the valid-trial target. TCP/tool, hardware-state and manual safety
confirmations remain independent motion blockers. No motion, gripper, Home,
trajectory, physical Hold/Stop or real task rollout was executed.

---

Real camera, JointState and OpenVLA; persistent subscribers. No command clients or sinks.

Normal completion has runtime_status=VALID, status=null and task_success=null; no physical task outcome.
INVALID is terminal; no further inference is dispatched. Already in-flight replies are retained separately.
Raw translation/rotation hard-limit exceedances terminate the trial. TCP-dependent workspace, gripper phase and
motion safety remain unverified; runtime VALID is not motion readiness. Startup/warm-up samples are preserved.

```json
{
  "total_attempts": 20,
  "valid_trials": 0,
  "invalid_trials": 20,
  "invalid_rate": 1.0,
  "invalid_reason_counts": {
    "raw_translation_step_limit": 20
  },
  "invalid_category_counts": {
    "INVALID_SAFETY_REJECTION": 20
  },
  "task_success_rate": null,
  "physical_commands": 0,
  "verdict": "PREDICTION_ONLY_BATCH_UNSTABLE",
  "target_valid_trials": 10,
  "max_attempts": 20,
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
