# Safety readiness

## Input/output contract update

- PASS offline: gripper `0=open, 1=closed`, 0.3/0.7 hysteresis and duplicate suppression.
- BLOCKED: measured gripper feedback is not identified; available Doosan DI fields have no validated gripper sensor-bit mapping.
- PASS offline: physical translation conversion is 1000 mm/m; empirical 2800 is rejected.
- PASS offline: Doosan A/B/C is Euler ZYZ degrees and dataset delta is a world/base rotvec; matrix composition tests pass.
- BLOCKED integration: existing runtime still performs component-wise A/B/C addition.
- PASS offline: OFT K=5 is consecutive temporal order and the queue consumes index 0→4 with stale/duplicate/underrun rejection.
- BLOCKED integration: existing runtime still consumes only index 0.

These results do not authorize motion. `motion_readiness=false`.

## Runtime safety completion

`runtime_safety_supervisor.py` is command-free and now validates camera/JointState/TCP watchdogs, inference and communication failures, action age, duplicate ID, NaN/Inf, command period, per-step translation/rotation, velocity and acceleration. Every rejection returns `hold_required=true` and `command_issued=false`; it does not call a Hold service.

`integrated_logger.py` now checks free disk space, writes a crash-visible partial marker, adds sequence and wall timestamps, fsyncs each JSONL record, and atomically replaces a completion status file. Disk-full and completion tests pass.

The suite now passes 31 checks: 27 unittest checks and four feedback-readiness checks. These modules are not wired to the existing command-capable runtime, so physical rollout remains blocked.

Status: `BLOCKED_REQUIRES_LOCAL_OPERATOR_AND_HARDWARE`

Gate 1 offline contracts implemented and tested (13/13):

- Correct closedness polarity and 0.3/0.7 hysteresis.
- repeated command suppression with explicit flag.
- `first_only` and `sequential_k5` queue modes.
- K=5 ordering, duplicate/stale/underrun/NaN rejection.
- timeout, communication, logger, workspace, translation and rotation failure -> `hold_required=true` state only.
- append/flush/fsync logger and null AI executed-action contract.
- unsafe offline startup configuration rejection.

These are pure Phase 12 modules. They are connected to a command-incapable Shadow logging core, but not to the dirty Real runtime. No physical Hold client exists. Live subscriber validation was not possible because the required ROS observation topics and both model servers were absent.

Hard blockers:

- runtime gripper polarity reversed;
- OFT only returns chunk index 0;
- requested model/checkpoint/preprocessing not wired;
- workspace, home and joint limits unverified; runtime 2800 empirical gain not approved;
- rotation conversion verified offline but not integrated;
- no measured gripper state;
- no hold acknowledgement;
- observed graph lacks ZED, Doosan and VLA interfaces;
- hardware E-stop state, Hold acknowledgement and local operator confirmations unavailable;
- model servers are not running;
- measured gripper feedback and corrected runtime integration remain unresolved.

Motion readiness: **false**. Gripper polarity, 1000 mm/m conversion, ZYZ matrix composition and sequential K=5 are verified only inside Phase 12; hardware limits, physical stop chain, measured gripper feedback and runtime integration remain unapproved.

Pre-rollout update: JointState-only rosbag independently confirmed multi-second source-timestamp stalls. Logger flush/fsync and null-execution tests pass, but atomic creation, disk-full/partial-record behavior, full rate/acceleration watchdog integration, and a physical Hold acknowledgement remain incomplete. Safety readiness is `FAIL/BLOCKED`, not merely awaiting motion approval.

## Post-relaunch readiness correction — 2026-10-02

The stall statement immediately above remains historical. Under the current
single-instance minimal observer, JointState and ZED continuity passed, and both
OpenVLA and OFT prediction-only stationary Shadow tests passed without any command
capability. The local operator reported no unusual site condition.

Current classification remains `BLOCKED_REQUIRES_LOCAL_OPERATOR_AND_HARDWARE`, not
because the observation/model path failed, but because the following motion-specific
items are not yet explicitly verified and recorded:

- protective-stop and servo-state current signals;
- Hold acknowledgement path;
- measured gripper feedback or an explicitly approved no-feedback operating policy;
- numerical workspace, joint/TCP velocity, TCP rotation and acceleration limits;
- maximum rollout duration;
- integration of the validated Phase 12 conversion/safety logic with a separately
  reviewed command runtime.

No motion gate is opened by the general “no unusual condition” attestation alone.

## Operator-selected 20 profile offline integration

The 20 mm/s, 20 deg/s, 20 mm/s² and 20 deg/s² candidate limits are now connected to
the command-free Phase 12 safety supervisor. Boundary and rejection tests pass. On
actual saved Phase 11 prediction data, 4/6 candidate actions passed and 2/6 were
rejected for translation acceleration. The supervisor emitted `hold_required` states
only; it cannot call Hold or issue a command.

This is a meaningful remaining runtime blocker: before motion, the command runtime
must implement a reviewed limiter/smoothing policy that guarantees the selected
acceleration profile, and the resulting candidate stream must pass offline. The
selected limits will not be silently increased.

Follow-up: a stateful speed/acceleration limiter was implemented and the saved
OpenVLA/OFT candidate stream now passes 6/6. Four actions were modified. Two initial
exact-boundary rejections were traced to floating-point comparison and fixed with a
numerical-only tolerance; physical limits remain 20. This clears the offline limiter
algorithm gate, but not runtime integration or physical motion readiness.

## Command-disabled pre-motion gate dry-run

A fail-closed gate now requires explicit true values for operator approval, E-stop,
protective-stop, servo state, robot mode, alarm clear, Hold acknowledgement, gripper
feedback or approved no-feedback policy, and site-approved workspace. It also checks
the candidate 30-second duration and camera/JointState/TCP/logger/model health.

Tests pass 45/45. The current dry-run returns `ready=false`, `hold_required=true`,
`command_issued=false`. Remaining blockers are protective stop, servo state, robot
mode, Hold acknowledgement, gripper feedback/no-feedback policy and workspace site
approval. The 30-second duration remains a candidate rather than an approved value.

## Command-disabled runtime integration

The wrapper connects model-specific K=1/K=5 handling, 1000 mm/m conversion,
world-rotvec/Doosan-ZYZ composition, gripper hysteresis, the stateful 20-profile
limiter, workspace checking and runtime safety inspection. It has no ROS dependency
or delivery API. Physical safety values remain deferred to the pre-motion checklist.

Tests pass 49/49. Replaying one actual stationary OpenVLA K=1 response and one OFT
K=5 response produced six candidates: four valid and two blocked. OFT chunk indices
3 and 4 requested close during `alignment` and were blocked as
`premature_gripper_close`. Executed/delivered actions stayed null and
`command_issued` stayed false. Motion readiness remains false.

## Open-loop gripper policy approval and implementation

The operator approved proceeding without measured gripper feedback under the
documented fail-closed open-loop policy. `OpenLoopGripperSupervisor` now starts in
UNKNOWN, requires operator-confirmed initial open, allows one close candidate only in
`grasp_close`, suppresses duplicates and changes to UNKNOWN on stale/timeout,
communication/logger failure, invalid values or an unknown command result. UNKNOWN
blocks lift and further automation. Command knowledge is never logged as measured
state; measured state remains null.

The complete suite passes 62/62 and the saved OpenVLA K=1/OFT K=5 dry-run retained
null execution/delivery for all six candidates. This clears the missing-feedback
policy decision, but not actual command-runtime integration or motion approval.
