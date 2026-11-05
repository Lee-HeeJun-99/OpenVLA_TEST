# Real rollout integration — command-disabled implementation

## Latest real read-only validation — 2026-10-07

Current real readiness: **BLOCKED_MULTIPLE**. [Long end-to-end report](real_trials/20261007_170908_final_end_to_end_readiness/FINAL_END_TO_END_READINESS_REPORT.md) supersedes older real-readiness assumptions, not the historical offline/fake implementation results below.

Same persistent JointState/camera subscribers: 180 s baseline PASS (18,000 samples, 100 Hz, ≥100 ms events 0). Subsequent 120 s OpenVLA Shadow FAIL: one 101.998 ms receive gap (corresponding source gap 9.998 ms), one unavailable ≤10 ms camera snapshot. Successful predictions 291/292 attempts; selected-frame max age 0.49473 s, age-limit violations 0, GPU/model failures 0. TCP active binding and independent hardware safety states remain unknown; no getter refresh after runtime fault. Physical commands **0**; production limits unchanged.

`final_end_to_end_readiness.py` is a command-incapable long audit with background image/JSONL logging. `summarize_long_readiness.py` performs offline distribution/context interpretation. Current Phase 12 unittest/regression suites: **192 PASS / 0 FAIL / 0 SKIP**, including 7 new long-audit tests; this is not a new DDS/process/hardware PASS claim. Training sample: 11 episodes/539 steps; its translation median/max 7.096/48.315 mm does not invalidate the configured 4 mm raw-action safety gate or establish checkpoint membership.

Historical offline/fake verdict (2026-10-06): **PRE_ROBOT_SOFTWARE_COMPLETE** (audited offline/fake scope), **HARDWARE_VALIDATION_PENDING**, physical rollout **NOT_AUTHORIZED**. Then-validation: 137 unittest/regression/DDS PASS, 13 fake-live process scenarios PASS, 7 generated-request/stage-intercept scenarios PASS. Default configs remain disabled. Hardware/GPU execution was pending in that report; it is not proof of real driver stop responsiveness or physical task success. See the current top section of FINAL_READINESS_REPORT.md and HARDWARE_DAY_CHECKLIST.md; older summaries below are history.

Latest follow-up: 131 unit/regression/DDS tests PASS plus 7 fake-live subprocess scenarios PASS. RobotState normalization, concurrency-safe logger, idempotent abort and actual processor metadata were added. This is still command-disabled: verified live servo/manual-auto sources and remaining E2E/model checks are documented in the latest FINAL_READINESS_REPORT section. Fake graph results are infrastructure tests, not model behavior findings.

Follow-up: background watchdog, interruptible gripper pulse, geometric phase detector, no-overlap scheduler and real isolated DDS integration tests are now added. Latest validation is 126 PASS. See the follow-up section of FINAL_READINESS_REPORT for exact remaining integration blockers; the older baseline below is retained as history and does not supersede the follow-up.

Date: 2026-10-06. This directory adds real ROS service request code, but no physical API was called during development. Production safety thresholds are unchanged.

## Implemented

- `real_sink.py`: four-factor authorization, lazy service clients, synchronous MoveLine, tool digital-output gripper pulses, MoveStop hold/stop, request/send/ACK/result timestamps, fake transport.
- `run_real_rollout.py`: canonical action → reused SafetyPipeline → authorization → sink → logging; corrected OFT target step and 0.2-second timing; recorded dry-run support.
- `live_observation.py`: camera/JointState/TCP subscribers, named joint ordering, approved FK fallback, model HTTP input, source image and model-input hashes. No unstable TCP getter calls.
- `pre_real_rollout_check.py`: fresh operator evidence requirements. Checklist readiness is not continuous hardware monitoring.
- `stage_gate.py`: preceding real PASS artifact required; dry-run cannot unlock motion.
- `integration_metrics.py`: descriptive trajectory/rejection/ACK metrics. Task success remains unknown without physical evidence.
- `configs/`: disabled model/protocol/service/polarity contracts.

## Verified source interfaces

| Operation | Service | Type | Meaning |
|---|---|---|---|
| Absolute base TCP pose | `/dsr01/motion/move_line` | `dsr_msgs2/srv/MoveLine` | pos mm/ZYZ degree; vel mm/s, degree/s; sync_type=0 |
| Gripper IO | `/dsr01/io/set_tool_digital_output` | `dsr_msgs2/srv/SetToolDigitalOutput` | output index 1..6, binary value; physical polarity pending |
| Hold | `/dsr01/motion/move_stop` | `dsr_msgs2/srv/MoveStop` | stop_mode=3 |
| Stop | same | same | stop_mode=0 |

Evidence: existing `openvla_doosan_runtime/doosan_bridge_node.py`, `doosan-robot2/dsr_msgs2/srv/`, and `dsr_controller2.cpp` movel callback. No joint command is introduced: the selected pipeline is TCP-only. Existing streaming, blend and joint APIs are deliberately not reused.

Service `success` is controller API return evidence, not independently observed physical completion. A hold service return is not proof of E-stop, safe standstill or protective stop. Command knowledge never becomes measured gripper state.

## Validation

Integration unit tests: 12 PASS / 0 FAIL / 0 SKIP. Existing Phase12 core: 100 PASS / 0 FAIL / 0 SKIP. Fake transport uses real service names but no DDS; generated request classes were also bound to fake clients without ROS initialization. This is not a fake ROS server discovery/executor test.

An initial test run found an import-name collision with the existing metrics module; renaming the new module resolved it. Euler singularity warning is expected; matrix-composition implementation is reused.

## Remaining software blockers — do not claim hardware-only readiness

1. Continuous hardware state monitoring and asynchronous watchdog must remain active while HTTP, MoveLine ACK and gripper pulse sleeps block. Current runner explicitly refuses non-dry motion with `continuous_hardware_watchdog_integration_pending`.
2. Gripper pulse interruption/de-energization policy needs a verified actuator contract; do not infer safe behavior from tool IO ACK.
3. Live phase progression currently comes from operator evidence; full-task phase/completion verification is not integrated. No task-success claim is possible.
4. Synchronous MoveLine transport cannot yet demonstrate a 5 Hz executed trajectory contract. Corrected K5 scheduling is implemented, not hardware cadence validated.
5. Real DDS fake-server/executor integration and model-specific preprocessing health checks need completion. Current tests do not establish these.

OFT early-close review remains required; the premature-close safety is unchanged. Current integration status: `PARTIAL_REAL_COMMAND_INTEGRATION_COMMAND_DISABLED`, not `HARDWARE_VALIDATION_ONLY_REMAINING`.

## OpenVLA full-policy live launcher — 2026-10-14 addition

The legacy status above is not authorization for the new launcher. Explicit
OpenVLA-only `run_live_full_policy.py` uses the persistent live episode engine
and `FULL_POLICY_ROLLOUT` Controller mode. Arm-only CLI remains arm-only.
Software wiring/fake-transport tests completed; this new gripper-inclusive live
launcher has not yet been physically executed.

Sequence: current REAL/AUTO/STANDBY, named-joint start pose and clean-window
readiness, camera/model/logger preflight, OFF/OFF initialization, normal OPEN,
fresh actual OPEN confirmation, bounded episode, sequential arm ACK/feedback/
settle then gripper state transition. Gripper hysteresis is <0.3 OPEN, >0.7
CLOSED, middle retain; no duplicate transition. Camera age is gated at inference
start, raw13mm and existing rotation/dynamic/workspace/JointState gates retained.

Normal OPEN: DO1 ON1.3s, OFF0.2s, ON0.5s, OFF. CLOSE: DO2 ON0.5s, OFF.
Fault: inhibit future commands, existing arm abort, both Tool outputs OFF with
ACK/readback confirmation, at most3 total pair attempts. Never retry ON, motion,
episode, Home or recovery. Electrical OFF policy is operator-approved; physical
jaw retention/release under faults is still not formally verified. Formal TCP
also remains supported-not-verified; require explicit engineering acceptance.

```bash
export ROS_LOCALHOST_ONLY=1
/usr/bin/python3 run_live_full_policy.py \
  --approval approval.json \
  --open-confirmation actual_open.json \
  --output real_trials/UNIQUE_FULL_POLICY_TRIAL
```

Default existing bounds:30s/150 actions. Explicit70s override requires both
`--max-duration-s 70` and `approved_max_duration_s:70` in approval; no unbounded
execution. Output directory must not exist.

Approval JSON must have execution_mode FULL_POLICY_ROLLOUT, operator_present,
workspace_clear, estop_accessible, robot_stationary, tcp_engineering_acceptance,
gripper_mapping_confirmed and explicit_one_episode all actually confirmed true,
and confirmed_wall_time fresh epoch seconds within60s. Never populate confirmations
from assumptions. Config mapping is enabled only in that approved session copy.

After OPEN ACK, the launcher waits up to60s for actual_open.json with observed_open
true, operator identity and confirmed_wall_time AFTER initialization ACK. It
rechecks approval freshness and start pose before arm dispatch. An old confirmation
file cannot start the episode. Ctrl+C/operator_abort.request uses software abort;
neither is a substitute for immediately accessible physical E-stop.

Normal horizon completion remains OPERATOR_RESULT_CONFIRMATION_PENDING, not
synthetic task SUCCESS/FAILURE. Record actual task outcome after episode end:

```bash
/usr/bin/python3 run_live_full_policy.py \
  --output real_trials/UNIQUE_FULL_POLICY_TRIAL \
  --finalize-outcome task_outcome.json
```

This is offline/no ROS. Outcome JSON requires operator, task_success boolean,
confirmed_wall_time after episode_ended_wall_time. INVALID/operator abort cannot
be promoted to SUCCESS. A valid completed physical trial is required. Hardware
state is queried at command boundaries, not represented as continuous independent
authority/servo/protection telemetry. Current safety confirmation remains required.
# 2026-10-30: stale_action critical-path update

The shared episode runner now performs three discarded fresh-frame model warm-up requests before
physical sink creation. Existing action age (1.2 s), 13 mm gate, camera/JointState watchdogs and full
settle remain unchanged. Exact model JPEG + mandatory JSONL are durable before dispatch; PNG and hash
work is deferred until the step returns. Deferred artifact rows join observations by `step_id`.
Controller rechecks the original inference-start age after logging; sink rechecks its deadline after
service preparation, immediately before async submission. No timestamp rebasing or gate relaxation.

Command-free live timing probe (use a NEW output directory):

```bash
source /opt/ros/humble/setup.bash
source /home/ubuntu/robot_ws/install/setup.bash
export ROS_LOCALHOST_ONLY=1
/usr/bin/python3 validate_action_latency.py --output real_trials/NEW_TIMESTAMP_latency_check --samples 5
```

This probe has no physical transport and only a read-only RobotState service client.
See `real_trials/20261030_stale_action_latency_fix_002/STALE_ACTION_FIX_REPORT.md` for changes,
regression results and live-validation limitations. Code/test PASS does not imply live timing/robot readiness PASS.
