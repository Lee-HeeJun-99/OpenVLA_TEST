# Latest readiness: BLOCKED (physical commands 0)

## Primary experiment: runtime JointState PASS

- Branch `lhj-research`; initial HEAD `2c7fa00758d2dbb2c70f87eb667002537f5ddd34`; initial worktree clean.
- Existing driver PID3061204 was preserved. This PID differed from yesterday before this task began; no driver restart was performed here.
- ZED launch3030200, OpenVLA3030376, RViz3061198 were SIGINT-stopped after exact PID verification. Driver and parent launch were untouched; other desktop/remote-access/simulation processes were not forcibly stopped.
- Shared FIRST_FRESH_SAMPLE arrived 12.462367s after observer start. Individual timestamps are in summary.json. Fresh means names/position/velocity valid, no source regression, same-ROS-clock header age in [0,100ms).
- Fixed warm-up10s, then nominal runtime30s; no window reset or sample deletion. Runtime BE3001 / REL3000, ~100Hz. Max source gap11.265ms, max receive gap11.888/12.271ms; duplicates/regressions/invalid/missing0. Latest receive age0.143/10.275ms.
- >=100ms events by phase, aggregated across both subscribers: startup0, warmup4, runtime0. These four are two timing events observed by each subscriber: receive wait~3.062s with header becoming~3.053s old, followed by source jump~3.060s and receive interval~10.5ms. Preserved in gap_phase_classification.json; one spans the shared startup→warmup boundary and is assigned by arrival phase, with phase_before retained.
- Primary verdict: **STARTUP_ARTIFACT_ONLY_RUNTIME_PASS**. The low-load runtime gate may be treated as PASS for this trial. This does not retroactively delete prior failed trials or establish the root cause of every historical gap. Runtime100ms watchdog remains unchanged.

## Automatic next-stage attempts (not motion)

Camera and OpenVLA were restored using existing verified launch/server commands. RViz remains off. Model health was strictly validated using a **recorded real image from 20261006**, with actual GPU sample inference, producing MODEL_HEALTH_PASS in model_health_warmup_recorded_image.json. This sample is model health/warm-up only, NOT a new current-camera Shadow prediction.

The existing command-disabled live Shadow was attempted using current camera/JointState/FK flange context. Its own subscriber received **0 JointState messages during its65s readiness wait**, so it failed at `pre_shadow_jointstate_gate_failed`. Prediction count0. It did not bypass the gate using the earlier PASS artifact. Today's30s selected-frame freshness validation therefore remains **NOT_EXECUTED/BLOCKED**, not PASS from yesterday's warm-window result.

A separate current passive TCP/state audit is saved at `../20261007_160557_automatic_hardware_state_audit/`. Camera850frames/20s passive stream PASS; actual model selected-frame freshness untested. JointState0 there also reflects that observer's window, not a gap measured in the successful runtime experiment. Discovery/subscriber behavior remains context-dependent.

Active TCP name/tool name/offset/current measured TCP: UNKNOWN. TF/URDF only provides flange/FK capability; no arbitrary zero offset or historical Tool_v1 reuse. Controller mode/authority/servo/protection/emergency/connection affirmative values remain UNKNOWN. No fresh verified read-only getter meets all prerequisites; requests0, no unstable posx invocation. Driver-reported event silence does not prove connection or safety.

Operator present, workspace physically clear, E-stop accessible: MANUAL_CONFIRMATION_REQUIRED. No repeated user questions and no auto-confirmation.

## Allowed stage / remaining blockers

The primary JointState experiment permits proceeding to TCP/camera prechecks, which were attempted. **Overall current readiness remains BLOCKED** by new live-observer discovery failure, incomplete current-camera prediction freshness, active TCP/tool contract and hardware/manual safety evidence. No minimum-motion/short-horizon/full-task stage was authorized or executed.

All physical robot/motion/gripper/Home/trajectory/Hold/Stop/rollout commands0. No service/action/command publisher capability in the primary validator; no driver/controller restart, mode/tool/servo change. Five focused phase/capability tests PASS,0FAIL,0SKIP; compile check PASS. No full-suite claim.
