# ServoJ staged field session

Hold, time exploration, and the ServoJ stop test have completed with reviewed field evidence. Policy A is enabled in this revision; recorded replay remains fail-closed. Do not proceed past a failed stage, loosen a limit, or replay an action after an interruption.

## Order and evidence

1. **Hold:** stream the measured current J1…J6 unchanged. Completed evidence; it never changes a gate automatically.
2. **Time exploration:** now enabled after reviewed hold evidence. J6 alone executes +2° and return for `time=0.10,0.05,0.02,0.01 s` in that order. It records candidates only; no gate is changed automatically and no trial is retried.
3. **Stop:** completed with selected `time=0.05 s`. During J6 +2° minimum-jerk motion, inhibit at measured 1.875°/s and call MoveStop `DR_QSTOP=1`.
4. **Recorded replay:** replay reviewed actions in settled Mode A and compare every arrived `tool0_real` pose with the MoveJ record.
5. **Policy A:** observation → inference → complete fixed-T precheck → segment → settle → new observation. No action repetition while waiting for a frame.
6. **Continuous B:** explicitly outside this session and outside the present design.

## Draft invocations

Hold, `time_explore`, `stop_test_servoj`, and policy A are enabled with fresh confirmation. Replay remains locked with `FIELD_ADAPTER_REVIEW_REQUIRED`.

```bash
/usr/bin/python3 tools/servoj_hold_test.py --enable-physical --output reports/live/<session>/servoj_hold_test/<time>
/usr/bin/python3 tools/servoj_time_explore.py --enable-physical --output reports/live/<session>/servoj_time_explore/<time>
/usr/bin/python3 tools/stop_test_servoj.py --enable-physical --send-rate-hz 100 --output reports/live/<session>/stop_test_servoj/<time>
/usr/bin/python3 tools/servoj_replay_actions.py --enable-physical --segment-T 0.6 --output reports/live/<session>/servoj_replay/<time>
/usr/bin/python3 tools/policy_servoj.py --camera-topic /zed/zed_node/rgb/color/rect/image --segment-T 1.0 --session reports/live/<session>
```

Paste only the command line, then type the requested `I_CONFIRM_V2_MODE:<mode>` phrase directly. Do not place a reusable confirmation in shell history.

## Enabled hold procedure

Before confirmation the screen must show: robot mode AUTONOMOUS, system REAL, state STANDBY, no alarm, fresh JointState, real-home error within ±2° for every named joint, ServoJ publishers `0`, and driver subscribers at least `1`. Any failed item must result in `sent_count=0`.

After typing `I_CONFIRM_V2_MODE:servoj_hold`, watch the robot, pendant state/alarm, and terminal continuously. The fixed initial measured J1…J6 target is sent for 10 s at 100 Hz using `vel=[10]*6 deg/s`, `acc=[20]*6 deg/s²`, `time=0.1 s`, mode 0. It then publishes nothing for 5 s, calls MoveStop mode 1 once, and observes for 3 s. Enter or `q`+Enter inhibits publication and calls MoveStop immediately.

The runtime abort threshold is any start-relative joint change over 0.05°, velocity over 0.005 rad/s, feedback stale, alarm, or state outside `{STANDBY,MOVING}`. Evidence passes only when every stage remains below 0.01° and 0.001 rad/s, alarm/SAFE count is zero, and send jitter p99 is below 2 ms. Review `hold_samples.jsonl`, `summary.json`, `system_metrics.jsonl`, and `system_monitor.log`; then a human may separately update `arm_stationary_verified`. The tool never edits `gates.json`.

Absolute-deadline loops never call sleep with a negative duration. If processing is at least one 10 ms period late, missed slots are counted and skipped; commands are never burst to catch up. Review `missed_deadline_count`, `max_lateness_ms`, and `deadline_by_stage`. `summary.json` also contains `move_stop.requested_monotonic`, completion time, ACK, and `stop_confirmation` based on ≤0.001 rad/s for 0.3 s. The hold monitor does not pass `--obs-pid` or subscribe to a camera; later observation tools supply those options normally.

Queue/time analysis is in `reports/servoj_hold_analysis.md`. Hold alone did not verify moving ServoJ timing; the later `20261113_162851` time-exploration evidence did, selecting 0.05 s.

## Enabled time exploration

Run only from real home after verifying the same preflight screen used by hold:

```bash
/usr/bin/python3 tools/servoj_time_explore.py --enable-physical --output reports/live/20261113_091859/servoj_time_explore/$(date +%Y%m%d_%H%M%S)
```

Paste the command only, then directly type `I_CONFIRM_V2_MODE:servoj_time_explore`. Each trial sends a 1.0 s minimum-jerk J6 +2° trajectory at 100 Hz, holds the last target by streaming for 1.0 s, observes without publication for 2.0 s, then performs the corresponding −2° return and another 2.0 s observation. J1–J5 remain fixed at the measured trial-start values. Message limits are 10°/s and 20°/s², mode 0.

Watch actual J6, pendant state/alarm and the terminal. Enter or `q` inhibits permanently and calls MoveStop. The same happens for J1–J5 motion over 0.05°, J6 command error over 1.0°, endpoint overshoot over 0.3°, J6 speed over 10°/s, stale feedback, alarm, or state outside `{STANDBY,MOVING}`. A failed trial skips every later time value.

Review `samples.jsonl`, `summary.json`, `system_metrics.jsonl`, and `system_monitor.log`. The per-trial table has this form:

| time (s) | result/reason | forward/return 50% lag (ms) | max tracking error (°) | overshoot (°) | post-send final error/additional motion (°) | settling time (s) | J1–J5 max change (°) | jitter p50/p99/max (ms) | missed |
|---:|---|---|---|---|---|---|---|---|---:|
| 0.10 | recorded | `forward.lag_50_percent_ms` / `return...` | leg fields | leg fields | leg fields | leg fields | six-axis record | trial field | trial field |

After every trial the full six-joint return error must be below ±0.01°. Evidence `20261113_162851` selected `time=0.05 s`: 0.02–0.10 s passed and 0.01 s is forbidden. The calibrated command-error threshold per joint is `2 × (0.020 + 0.6 × time) × segment_max_velocity + 0.1°`; for this J6 test it is 0.475°.

## Enabled ServoJ stop test

```bash
/usr/bin/python3 tools/stop_test_servoj.py --enable-physical --output reports/live/20261113_091859/stop_test_servoj/$(date +%Y%m%d_%H%M%S)
```

Type `I_CONFIRM_V2_MODE:stop_test_servoj` directly. J6 follows a +2°/1 s minimum-jerk command with `time=0.05 s`. At the first measured `|J6 velocity|≥1.875°/s`, the tool records the trigger, inhibits publication, and then calls MoveStop mode 1. The terminal must show this order. No trigger within 3 s is an invalid trial and still invokes MoveStop.

Pass requires successful ACK or a preceding controller SAFE state, 0.001 rad/s continuously for 0.3 s with confirmation within 1.0 s of the trigger, post-trigger travel below 0.3°, J1–J5 movement below 0.05°, and no alarm. After a passed stop, the already-verified MoveJoint path returns all joints to the measured start at 5°/s and 10°/s² and confirms arrival; ServoJ is never used for return. Review `trigger`, `move_stop`, `stop_confirmation`, `post_stop_travel_deg`, `events`, and `return_to_start` in `summary.json`. Gate updates remain manual.

Evidence `reports/live/20261113_091859/stop_test_servoj/20261113_164249/summary.json` passed and has been registered as `stop_servoj_verified=true`: trigger 1.95°/s, MoveStop ACK in 2.6 ms, stop confirmation in 0.33 s, 0.045° post-trigger travel, negligible J1–J5 motion, and successful MoveJoint return.

## Enabled policy ServoJ A

From manually established real home, paste only this one command and then type both confirmation phrases directly:

```bash
/usr/bin/python3 tools/policy_servoj.py --server http://127.0.0.1:8766 --camera-topic /zed/zed_node/rgb/color/rect/image --instruction "Pick up the blue cube." --scene-cube-color blue --segment-T 1.0 --session reports/live/20261113_091859
```

Type `I_CONFIRM_V2_MODE:policy_servoj`, then `TABLE_WATCH_BY_OPERATOR`. `--segment-T` accepts only `0.6` or `1.0`; 1.0 is the default. Optional controls match MoveJ: `--server-pid`, `--max-steps`, `--max-episode-time`, `--sigma-min`, `--sigma-min-reason`, and `--max-descent-mm`. Lowering sigma requires a recorded reason.

Every step must be stationary before requesting a new post-settlement frame. The tool never repeats an action while waiting. It precomputes the full Cartesian minimum-jerk segment and all IK samples before publication, holds the last target by continued 100 Hz publication until 0.001 rad/s for 0.3 s and ≤0.1° target error, and records `steps.jsonl`, `predictions.jsonl`, model-input images, `summary.json`, and system telemetry. Enter/q inhibits publication and calls MoveStop immediately. There is no automatic home return and gripper output remains record-only.

At every stage, Enter/q must inhibit further ServoJ samples immediately, call the reviewed MoveStop mode, log response time, continue feedback polling, and permanently inhibit further commands if zero-speed confirmation fails.

## Required policy telemetry

Output is `reports/live/<session>/policy_servoj/<timestamp>/`. `steps.jsonl` retains the policy_movej action/pose/arrival/timing fields and adds `servoj_sent_count`, `send_jitter_ms` (`p50/p99/max`), `missed_deadline_count`, `tracking_err_deg_max`, `tracking_threshold_deg`, and `segment_T_s`. The system monitor is mandatory but its failure is warning-only, as in policy_movej. `wandb_upload.py --all` discovers these folders through `summary.json` and records `control_mode`, segment T, and ServoJ time in run config.
