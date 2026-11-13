# Existing ServoJ path audit

This is a source audit only. No ROS graph or robot was contacted.

## Current path

| Item | Existing behavior | Evidence |
|---|---|---|
| trajectory | `SegmentGenerator` creates 100 Hz minimum-jerk Cartesian interpolation and rotation slerp, solves every IK sample using the preceding solution, and precomputes the entire segment. It lengthens D dynamically until candidate velocity/acceleration pass. | `controller/trajectory.py` |
| initial timing | Rate 100 Hz, initial D 0.5 s. The controller candidate limits are π rad/s for J1–J3, 2π rad/s for J4–J6; acceleration 3/6 rad/s². | `controller/controller_config.json`; `controller/config.py` |
| wire message | `ServojPublisher` maps canonical J1…J6 radians to degrees, QoS depth 1, mode 0. `time`, `vel`, and `acc` come from runtime settings and are currently `null`, so publication is refused. | legacy `runtime/servoj.py`; compatible `runtime/servoj/publisher.py`; `runtime/runtime_config.json` |
| driver side | The investigated subscriber is reliable, depth 20, and forwards fields to DRFL without application-side validation. | `DESIGN.md` driver audit |
| CLI/adapter | `run_v2.py` authorizes modes but has no physical mode adapter (`MODE_ADAPTER_NOT_BOUND_IN_CLI_FAIL_CLOSED`). `runtime/ros_transports.py` contains MoveStop and MoveJoint transports, not a ServoJ transport; the publisher is separate. | `runtime/run_v2.py`; `runtime/ros_transports.py` |
| stop | Existing generic runtime has `StopCoordinator`/MoveStop. The old ServoJ publisher only supports `inhibit`; it was not assembled with feedback monitoring and StopCoordinator into a policy loop. | `runtime/stop.py`; legacy `runtime/servoj.py` |
| gates | `servoj_time_param_verified=false`, `tracking_thresholds_calibrated=false`, `stop_servoj_verified=false`, `arm_stationary_verified=false`. Policy requires the first three (plus limits/topics/rate); `arm_stationary_verified` is not currently in the policy gate. | `runtime/gates.json`; `runtime/gate.py` |

## Difference from policy_movej

| Concern | policy_movej | old ServoJ path | new ServoJ Mode-A building block |
|---|---|---|---|
| process isolation | A: control/safety; B: image/inference | none assembled | same contract; stream thread belongs only to A |
| command execution | one asynchronous MoveJoint | publisher wrapper only | fixed-T, absolute-deadline 100 Hz thread |
| stop | StopCoordinator, inhibit, MoveStop, zero-speed confirmation | inhibit only in publisher | stream inhibition + stop callback on tracking/freshness/state fault |
| Enter/q abort | active during motion | absent | `OperatorAbort` contract; field adapter remains gated |
| action planning | one target IK + 20 joint-linear samples | Cartesian sample IK, but D auto-extension | fixed T; all samples accepted or rejected before sample zero |
| telemetry | system monitor, inference/step JSONL | absent | sent count, jitter p50/p99/max, max tracking error, T, precompute/move/settle totals |
| field readiness | proven MoveJ stop path | time/stop/tracking unknown | deliberately fail-closed until staged evidence exists |

## Disposition

- Reuse: kinematics and strict/fallback DLS behavior, `minimum_jerk`, `ToolTransform.tool0_real`, `world_left` action application, feedback/robot-state/stop/gate modules, process-B observation protocol, system monitor and post-run upload.
- Correct/complete: fixed-duration planner (no automatic D change), per-sample sigma/margin/dynamics checks, dedicated absolute-deadline sender, command-vs-measured joint tracking, A/B integration, Enter handling, ServoJ-specific telemetry and staged field adapters.
- Discard from policy execution: `run_v2.py`'s unbound physical dispatch and any assumption that publisher construction alone provides a safe runtime. Keep them only as fail-closed legacy scaffolding.
- Gate correction to review before field policy: whether `arm_stationary_verified` must be explicitly added to `policy` requirements. It is false and must not be inferred from the MoveJ test.
