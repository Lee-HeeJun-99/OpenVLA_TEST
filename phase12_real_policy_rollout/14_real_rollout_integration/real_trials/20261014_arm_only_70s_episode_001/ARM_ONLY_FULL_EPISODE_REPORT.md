# Operator-approved 70-second arm-only episode

Scope ARM_ONLY_FULL_EPISODE_SYSTEM_INTEGRATION_TEST. Operator explicitly
selected70s. This run-specific duration override leaves the production30s
default unchanged and keeps its150-action cap. All original action/sensor
gates and dynamic limits retained. Initial hardware REAL/AUTO/STANDBY,
fresh clean readiness and start error0.00004775degrees passed.

26 fresh predictions logged;25 sequential MoveLine commands completed with
successful ACKs. Final prediction not dispatched after the duration limit.
Observed70.385s includes termination/cleanup. No automatic retry, next episode,
Home or recovery. Gripper IO0 and physical Stop0.

Max raw translation11.054mm, delivered4.000mm; raw/delivered rotation0.813degrees.
No model, logger, ACK, hard-gate or workspace failure. JointState source/receive
max14.846/19.867ms and >=100ms events0; camera-start age max19.524ms.
Last post-command state STANDBY, final recorded joint velocities allzero.

All26 model gripper predictions open-like; physical gripper disabled throughout.
Gripper abort remains UNVERIFIED/UNUSED, formal TCP supported-not-verified.
Engineering execution acceptance is not formal TCP verification.

Technical termination normal. Status OPERATOR_RESULT_CONFIRMATION_PENDING until
actual motion and E-stop confirmation. Task SUCCESS/FAILURE NOT_APPLICABLE;
excluded from task-performance denominator. A70s horizon is not a claim that
a precomputed full trajectory or the cube-pick task was completed.

## Final operator confirmation

Operator replied "좋았어" to the expected-motion/no-unexpected-motion/no-E-stop
question. Positive confirmation recorded with verbatim question and response in
operator_confirmation_result.json. Final result ARM_ONLY_FULL_EPISODE_PASS.
No additional command sent. This supersedes the pending confirmation above,
not the original command/timing evidence. Task performance remains not applicable.
