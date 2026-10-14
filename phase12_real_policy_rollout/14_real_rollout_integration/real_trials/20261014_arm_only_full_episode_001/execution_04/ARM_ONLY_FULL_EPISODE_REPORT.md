# Arm-only episode execution 04

Scope: ARM_ONLY_FULL_EPISODE_SYSTEM_INTEGRATION_TEST. Ten inference steps and
ten sequential arm MoveLine commands completed with successful ACKs. Existing
duration limit30s reached; observed duration30.318s includes loop termination
and cleanup. No extra step, retry, Home, recovery or next episode. Gripper
and gripper-abort IO0, physical Stop0.

Max raw translation11.054mm, delivered4.000mm; max raw/delivered rotation
0.813degrees. Existing13mm hard gate and dynamic limiter preserved. All targets
passed workspace. Model, ACK, logger and safety failure counts0.

JointState max source13.843ms, receive18.428ms; >=100ms events0. Max camera
inference-start age41.332ms. Final reported robot stateSTANDBY and final recorded
joint velocities0.

200ms exact-zero settle windows reset105 times across pre-observation,
pre-dispatch and post-command checks; all waits completed within the existing
2s timeout. No velocity tolerance or runtime watchdog was relaxed. Clean
timer resets are not task INVALID events. These data demonstrate bounded
settle recovery during this episode, not universal immunity to runtime faults.

All ten gripper predictions open-like; raw values logged only. Hardware gripper
abort behavior remains UNVERIFIED/UNUSED. Formal TCP stays supported-not-verified,
execution using explicit arm-only engineering acceptance.

Technical completion normal; status OPERATOR_RESULT_CONFIRMATION_PENDING until
operator confirms expected motion and no E-stop/unexpected behavior. Do not
infer physical behavior solely from ACKs. Task SUCCESS/FAILURE NOT_APPLICABLE,
excluded from task-performance denominator. Prior INVALID trials remain intact.
