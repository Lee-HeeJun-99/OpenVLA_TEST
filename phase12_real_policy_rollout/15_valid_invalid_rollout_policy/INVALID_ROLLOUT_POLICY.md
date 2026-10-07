# SUCCESS / FAILURE / INVALID rollout policy

## Scope and safety

Infrastructure validity and VLA task outcome are separate. Terminal trial `status` is exactly **SUCCESS**, **FAILURE**, or **INVALID**. During preparation/active execution it is `null`, never a fourth terminal status. Readiness/stage PASS/FAIL artifacts remain a different schema; old observations and readiness reports are not relabeled as new trials.

No threshold tuning: JointState source/receive gaps must remain strictly below100 ms, latest age below500 ms, selected-camera age below0.5 s. Invalid names/values, non-increasing timestamps, input/HTTP/logger/runtime/watchdog/operator faults and hard safety rejection terminate the current trial as INVALID. Raw 4 mm/4 degree gates are unchanged. INVALID does not authorize motion or waive preflight, hardware/TCP contracts, approval, stage gates or emergency-stop requirements.

## Trial lifecycle

`WAIT_DISCOVERY → FIRST_FRESH_SAMPLE → consecutive 10 s CLEAN WARMUP → RUNTIME_READY → start trial`.

Readiness has a 120-second overall deadline. Pre-trial gaps, stale samples, invalid
values or timestamp errors are STARTUP_WARMUP_EVENTs and reset the clean timer.
Recovery starts a new clean timer, not a performance trial. A readiness timeout
is BLOCKED and excluded from task SUCCESS/FAILURE/INVALID counts. Only after
RUNTIME_READY are sensor faults terminal for the current trial; no in-trial reset.

Only the new trial's fresh readiness may start it. Startup/warm-up events are retained as pre-trial evidence and excluded from trial failure counts. Existing runtime faults stay latched. The same invalid trial can never reset or resume. `LiveObservation.rearm_trial_readiness()` creates a new readiness state on the existing node/subscribers, retains old events, seeds prior timestamp comparison and waits for a **new** fresh sample plus10 s. It does not clear a trial or sink, grant physical approval, or reset/move the robot. Call only after the previous worker/watchdog has finished.

At first invalid event: inhibit new model and gripper dispatch, latch statusINVALID/runtime_valid=false/task_success=null; record monotonic and wall invalid timestamps, exact reason/category, last valid observation, latest raw prediction and already-dispatched command count. Additional faults cannot change the first terminal outcome. Already-submitted motion is not assumed canceled or physically stopped. Existing validated emergency Hold/Stop/abort-output paths remain separate safety operations, not new policy actions.

ROS discovery/request preparation and ACK wait are outside the trial invalidation lock. Only actual async request submission is serialized with INVALID; a blocked prepare cannot later dispatch a model/gripper command after invalidation. ACK may resolve after INVALID and remains in-flight evidence, never task success.

## Outcome and provenance

- Valid runtime + explicit task-success evidence → SUCCESS, runtime_valid=true, task_success=true.
- Valid runtime + explicit task-failure evidence → FAILURE, runtime_valid=true, task_success=false, reasonFAILURE_TASK.
- Sensor/runtime/safety fault → INVALID, runtime_valid=false, task_success=null.
- Missing task-outcome evidence → INVALID_TASK_OUTCOME_UNVERIFIED, never inferred SUCCESS/FAILURE from ACK, protocol completion or task sequenceCOMPLETE. This is an unevaluable trial, not a VLA task failure.

The single live real runner always attaches a trial classifier. `--task-outcome-evidence` must bind its trial ID and evaluation scope and supply a boolean. For real task results, operator identity, `physical_task_evidence=true` and an observation time within that trial are required. Stage technical completion is not physical grasp success. A non-dry INVALID result cannot grant a next-stage PASS. Legacy standalone Controller unit fixtures can omit a classifier; production runner cannot.

## Saved evidence

Per attempt directory: `trial_summary.json`, `observations.jsonl`, `predictions.jsonl`, `runtime_events.json`, `invalid_event.json` (null for valid trials), plus existing controller/sensor/image logs. No trial directory is deleted. Nonfinite raw evidence is JSON-tagged, not silently dropped. Summary fields include the requested validity/outcome/counts/phase and Observation/Representation/Action Gap join IDs/scores; unavailable values remain null.

Outcome metadata is committed only after prospective artifacts can be persisted. Persistence failure before commit produces INVALID_LOGGER_ERROR rather than SUCCESS. If the filesystem cannot even preserve INVALID evidence, batch stops withARTIFACT_PERSISTENCE_FAILURE; it does not silently continue or pretend disk-full logging succeeded.

## Aggregation and research

`valid_trials = success_trials + failure_trials`.

`task_success_rate = success_trials / valid_trials`; `invalid_rate = invalid_trials / total_attempts`. A zero denominator yieldsnull, not0%. Duplicate trial IDs, nonterminal/inconsistent outcomes and mixed evaluation scopes are rejected. Invalid categories are counted separately. Main `performance_groups()` includes only task-scope SUCCESS/FAILURE; INVALID, prediction-only and simulation fixtures do not enter real policy performance groups. Invalid evidence remains available for runtime/sensor/ROS/DDS reliability analysis.

The task-success rate is conditional on valid execution. Especially when safety rejection itself causes INVALID, always report its invalid count/rate alongside success rate to avoid concealing unsafe-policy selection effects.

## Batch and physical recovery

`RolloutBatch(target_valid_rollouts=20,max_attempts=30)` supports new-attempt readiness and bounded retry. INVALID preserves data and consumes an attempt, but not a valid-trial target. SUCCESS and FAILURE both consume a valid target.

Current CLI is **recorded/synthetic prediction-only**, uses dry-run Controller and has no physical automation path. Physical batch requests explicitly fail closed with a validated-recovery/operator-reset requirement. No new Home/reset/recovery motion exists. On a real trial termination, operator reset through an already validated protocol and new preflight/approval/readiness are required before the next single trial. Existing TCP/hardware/manual blockers remain blockers; merely excluding INVALID from statistics does not resolve them.

## Reproduction

From this directory, with a new output path:

```bash
/usr/bin/python3 -m unittest discover -s tests -v
/usr/bin/python3 run_rollout_batch.py \
  --recorded-input fixtures/recorded_predictions.json \
  --output results/new_dry_run \
  --target-valid-rollouts 3 --max-attempts 7
```

The checked-in fixture is synthetic, NOT checkpoint output and NOT a new robot task experiment. Its true/false task outcomes are explicit test labels. Actual process result `results/recorded_dry_run_01`:7 attempts,3 valid,4 invalid,2SUCCESS/1FAILURE; conditional software-fixture success2/3, invalid4/7, physical commands0, transport calls0. It covers joint/camera/input/safety invalid reasons and retry. Never report these ratios as OpenVLA/OFT performance.

## Current next-stage gate

Policy implementation may PASS while real rollout readiness is BLOCKED. Previously unresolved current TCP/tool, independent hardware-state and manual safety evidence are not replaced by this policy. This task performs no real motion or real recovery. Next physical stage remains **BLOCKED** until fresh original gates and operator evidence pass.
