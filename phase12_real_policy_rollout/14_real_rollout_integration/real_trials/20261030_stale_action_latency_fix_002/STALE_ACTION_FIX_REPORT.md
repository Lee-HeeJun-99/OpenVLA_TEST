# stale_action latency-path change — 2026-10-30

## Scope and status

- Code baseline: `07039966653ac26f55f2dd0390b4fdf0e1955d72`, branch `lhj-research`.
- Production arm-only and full-policy episode engines share the updated path.
- No physical motion, gripper, Hold or Stop commands were sent for this work.
- No safety limit changes, clipping/scaling, timestamp rebasing or automatic episode retries.
- Implementation and regression tests passed. **Live latency improvement is NOT YET VERIFIED.**

## Previously measured failure

20261015_full_policy_live_001: inference 1.128501 s, post-inference to Controller age check 0.746780 s,
total original inference-start age 1.875281 s > 1.2 s. That post-inference period included logging,
PNG compression/save, hashes, scalar state getter and 0.2 s consecutive settle. There were no per-operation
timers proving that PNG alone accounted for all of the excess. Earlier warm prediction was 0.457073 s.

## Changes

1. `rollout_latency.warm_up`: three fresh-frame prediction-only requests before physical sink construction.
   First request is explicitly recorded/discarded (not automatically claimed to be a server cold start).
   Both subsequent requests must leave room for the existing 0.2 s settle within 1.2 s.
   All three requests are excluded from episode attempts, outcomes and physical commands. No warm-up action reuse.
   Runtime sensor/model failures still block. Warm-up failure does not trigger motion/gripper/stop clients.
2. The exact already encoded JPEG sent to the model is written and fsynced before dispatch, together with
   mandatory raw prediction/current observation and Controller ACTION JSONL logs.
   Expensive PNG encoding and SHA hashes execute after Controller.step returns/raises, including rejection.
   `observations.jsonl` has a pre-command metadata row and a `DEFERRED_ARTIFACTS` row joined by step_id.
   PNG/hash failures abort and inhibit future commands. No async unbounded logger queue was introduced.
3. Fresh robot-state getter and the complete existing 0.2 s clean settle remain before dispatch.
   New PRE_DISPATCH_TIMING records inference, required evidence, state/settle and original action age.
4. Controller checks age after mandatory ACTION fsync, using the ORIGINAL inference-start timestamp.
   The physical sink checks that same deadline again after service preparation/discovery and immediately
   before actual submission. Expiry inhibits arm submission; emergency Hold/Stop and existing gripper
   electrical abort cleanup remain usable. No rebasing of timestamp to inference end.

## Thresholds unchanged

Action age 1.2 s; JointState source/receive 100 ms and existing freshness checks;
camera inference-start age 0.5 s and camera stream watchdog; raw translation 13 mm;
rotation, dynamic limiter, workspace and gripper contracts unchanged.

## Tests

Full ROS-sourced offline suites: integration 176, Phase12 safety 103, valid/invalid policy 30 — **309 PASS**.
Added coverage: warm-up/discard, slow warm rejection, duplicate fresh-frame rejection, sensor failure,
exact JPEG byte persistence, archive/deferred logger errors, stale camera and nonfinite action rejection,
mandatory logger delay expiry, service preparation delay expiry, unchanged original timestamp/1.2 s gate.
These tests use fake transports; they are not physical hardware observations.

An initial unsourced/PYTHONPATH-overwritten test run could not import rclpy. The corrected runs preserved
ROS PYTHONPATH and passed; the import error was not hidden or treated as a functional PASS.

## Live attempts (no physical transport)

- `20261030_stale_action_latency_fix_001/summary.json`: BLOCKED, model connection refused.
- Started existing `scripts/serve.sh vanilla 8766 0 127.0.0.1`; strict identity ready for step8130.
  Driver and ZED were not restarted by this work.
- This attempt (`..._002/summary.json`): FAIL at `fresh_camera_timeout` before any warm-up prediction completed.
  Measured would-dispatch samples: 0, physical commands: 0, transport_created: false.
- Follow-up current graph lookup reported `/zed/zed_node/rgb/color/rect/image` unknown and the previously
  observed ZED PID 67721 no longer existed. This establishes current source unavailability, not its cause.
  Camera/root-cause investigation or threshold relaxation was not performed as part of this fix.

Therefore no new dispatch latency numbers or physical readiness PASS are claimed.

## Next allowed validation

With the existing driver/ZED/model launched and publishing the expected image topic:

```bash
source /opt/ros/humble/setup.bash
source /home/ubuntu/robot_ws/install/setup.bash
export ROS_LOCALHOST_ONLY=1
cd /home/ubuntu/a0509_vla_linux_field_bundle_20260903/lhj/phase12_real_policy_rollout/14_real_rollout_integration
/usr/bin/python3 validate_action_latency.py \
  --output real_trials/NEW_TIMESTAMP_stale_action_latency_check --samples 5
```

This command creates only a scalar read-only RobotState client and image/JointState subscribers.
It never creates motion, IO, Hold/Stop or physical sink interfaces. PASS means only that the measured
prediction/logging/state/settle age samples passed the timing check, not full physical readiness.
Actual production dispatch is still subject to every existing gate and the two extra deadline checks.
