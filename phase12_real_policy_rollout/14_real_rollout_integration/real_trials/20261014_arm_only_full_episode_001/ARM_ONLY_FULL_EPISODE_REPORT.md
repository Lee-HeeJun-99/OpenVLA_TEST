# Arm-only full episode preparation

Explicit `ARM_ONLY_FULL_EPISODE` Controller mode, separate from arm-one-step,
legacy short-horizon and full-policy gripper state machine. The new
ArmOnlyEpisodeSink refuses all gripper/IO requests, including emergency IO;
send_gripper also refuses. Arm HOLD/Stop remains available. Gripper abort
behavior remains UNVERIFIED and UNUSED, abort_value unchanged.

Launcher `run_arm_only_episode.py` rechecks real hardware, named-joint start
pose, consecutive10s readiness, camera and strict model identity. It serializes
MoveLine, ACK, fresh state and settle, uses unchanged raw13mm/rotation gates,
velocity/acceleration limiter and workspace checks, and keeps sensor subscribers
and watchdog alive. Existing full_task bounds reused: duration30s/actions150,
whichever first; no dispatch after deadline, retry/home/recovery/batch disabled.
Fault inhibits future commands and invokes existing arm abort only. An
operator_abort.request marker and KeyboardInterrupt also terminate the episode;
neither chat nor the file replaces physical E-stop.

Current read-only preflight: REAL/AUTO/STANDBY; start maxerror0.00004775degrees;
JointState source/receive>=100ms events0, max12.834/53.627ms; camera stream and
OpenVLA identity PASS. This is not reused instead of the launcher's live checks.
Formal TCP remains supported-not-verified; explicit arm-only engineering basis
is distinct from formal verification.

Tests250 PASS: integration117, safety103, INVALID policy30. Includes multi-step
close-like predictions with zero IO, emergency gripper rejection, overlap
rejection, fault stop/inhibit, legacy one-step preservation and original bounds.

Actual episode NOT STARTED, awaiting fresh operator approval requested through
the conversation. Physical commands in this request0. No system-integration
PASS/FAIL or task outcome fabricated. After execution, runtime-clean technical
completion still requires actual operator behavior/E-stop confirmation before
final result. Task SUCCESS/FAILURE remains NOT_APPLICABLE.
