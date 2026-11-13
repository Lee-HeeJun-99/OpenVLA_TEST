# ServoJ staged field session

All commands below are future field-session commands. The present code was only tested offline. Do not proceed past a failed stage, do not loosen a limit, and do not replay an action after an interruption.

## Order and evidence

1. **Hold:** stream the measured current J1…J6 unchanged. Pass only if measured motion is zero, 100 Hz timing is acceptable, tracking is bounded, and stop remains available. This produces a candidate for `arm_stationary_verified`.
2. **Time exploration:** J6 +2° only, testing reviewed combinations of message `time` and send period. Record lag, overshoot, jitter and the controller response after stream end. This establishes `servoj_time_param_verified` and tracking candidates; no automatic retry.
3. **Stop:** during the same small ServoJ motion, stop publication then call MoveStop `DR_QSTOP=1`. Pass only with a nonzero-speed request, ACK or prior SAFE state, and 0.001 rad/s for 0.3 s. This produces `stop_servoj_verified` after human review.
4. **Recorded replay:** replay reviewed actions in settled Mode A and compare every arrived `tool0_real` pose with the MoveJ record.
5. **Policy A:** observation → inference → complete fixed-T precheck → segment → settle → new observation. No action repetition while waiting for a frame.
6. **Continuous B:** explicitly outside this session and outside the present design.

## Draft invocations

These drafts are intentionally locked with `FIELD_ADAPTER_REVIEW_REQUIRED`; they construct no ROS command object in the offline revision. When field adapters are reviewed, retain both `--enable-physical` and fresh confirmation, start `system_monitor.py`, and write `summary.json` on success, exception, or Enter/q abort.

```bash
/usr/bin/python3 tools/servoj_hold_test.py --enable-physical --duration 2 --send-rate-hz 100 --output reports/live/<session>/servoj_hold_test/<time>
/usr/bin/python3 tools/servoj_time_explore.py --enable-physical --send-rate-hz 100 --output reports/live/<session>/servoj_time_explore/<time>
/usr/bin/python3 tools/stop_test_servoj.py --enable-physical --send-rate-hz 100 --output reports/live/<session>/stop_test_servoj/<time>
/usr/bin/python3 tools/servoj_replay_actions.py --enable-physical --segment-T 0.6 --output reports/live/<session>/servoj_replay/<time>
/usr/bin/python3 tools/policy_servoj.py --enable-physical --segment-T 0.6 --output reports/live/<session>/policy_servoj/<time>
```

Paste only the command line, then type the requested `I_CONFIRM_V2_MODE:<mode>` phrase directly. Do not place a reusable confirmation in shell history.

At every stage, Enter/q must inhibit further ServoJ samples immediately, call the reviewed MoveStop mode, log response time, continue feedback polling, and permanently inhibit further commands if zero-speed confirmation fails.

## Required policy telemetry

Output is `reports/live/<session>/policy_servoj/<timestamp>/`. `steps.jsonl` retains the policy_movej action/pose/arrival/timing fields and adds `servoj_sent_count`, `send_jitter_ms` (`p50/p99/max`), `tracking_err_deg_max`, and `segment_T_s`. The system monitor is mandatory but its failure is warning-only, as in policy_movej.
