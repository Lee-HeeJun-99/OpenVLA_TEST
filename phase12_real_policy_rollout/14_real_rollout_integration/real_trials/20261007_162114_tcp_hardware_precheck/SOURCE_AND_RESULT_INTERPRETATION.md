# Current TCP/hardware read-only precheck

Starting branch `lhj-research`, HEAD `946361fac0101a9b1404b5e8a6c0e28d76daa52f`. Driver PID3061204 unchanged.

## Direct observations

FIRST_FRESH_SAMPLE after10.84108 s, fixed10 s warm-up, same persistent subscriber retained across all5 getter calls and10 s post-call observation. Runtime1527 samples over15.27603 s (~99.895 Hz), max sourcegap12.2607 ms, max receivegap27.3745 ms, runtime>=100 ms events0, latched faultnone. Post-observation final receive age4.674 ms. No warm-up gap observations in this trial.

| Getter (exactly1 request each) | Result | Observed client latency |
|---|---|---:|
| `/dsr01/tcp/get_current_tcp` | success=true, info="" |11.620 ms|
| `/dsr01/tool/get_current_tool` | success=true, info="" |14.774 ms|
| `/dsr01/system/get_robot_mode` |1 = ROBOT_MODE_AUTONOMOUS|14.716 ms|
| `/dsr01/system/get_robot_state` |1 = STATE_STANDBY|10.244 ms|
| `/dsr01/system/get_robot_system` |0 = ROBOT_SYSTEM_REAL|10.212 ms|

These latencies include Python client observation/polling; not controller-internal durations.

## Source and meaning

`dsr_controller2.cpp:1938` calls `Drfl->get_tcp()` and sets success=true unconditionally; `:1967` similarly calls `get_tool()`. srv definitions specify string info as name. Empty string does NOT prove zero offset, no active TCP, physical tool absence, or valid controller configuration. Thus active names UNKNOWN and TCP_UNKNOWN. Exact-name6D binding search cannot proceed with an empty name. Historical Tool_v1 is not reused.

Mode/state/system callbacks at326/347/339 call the corresponding DRFL getters, with no visible setter/motion operation. Returned enum definitions come from actual dsr_msgs2 system srv files, not invented mappings. The values are DRIVER_REPORTED via the DRFL getter boundary; freshness/internal cache semantics of vendor library remain unverified. One successful query is not a continuous authority/connection/stop-state source.

`get_current_posx` was NOT called due known feedback-loss history. `get_current_pose` was NOT called: visible callback dereferences the vendor returned pointer without a null check and vendor live stability is unverified. No verified Cartesian topic/tool offset was established. FK flange is not current actual TCP.

Authority, servo enabled, dedicated protective-stop/E-stop status, and continuous affirmative controller connection remain UNKNOWN. STANDBY is recorded but not promoted into those separate safety signals.

## Log comparison

Baseline file offsets and endpoint/type snapshot are in tcp_tool_getter_precheck.json. ROSout yielded0 messages; no-event evidence is not affirmative safety evidence. Actual driver/launch file offsets were read after the trial: only the3 expected hardware getter callback lines were appended; no motion/setter/disconnect/error lines in that examined delta. See driver_file_log_delta.txt. TCP/tool logging is commented in the source.

## Readiness

- JointState current trial runtime: PASS.
- Camera passive subscriber:1600 frames. Selected-frame freshness/OpenVLA live Shadow PASS belongs to the preceding trial, NOT a new30 s camera/inference validation in this precheck.
- TCP name/offset/current pose: NOT_VERIFIED.
- Mode AUTO / state STANDBY / system REAL: DRIVER_REPORTED snapshot only.
- Gripper output/pulse/model gripper application: DISABLED; physical gripper state UNKNOWN.
- Operator presence/workspace clear/E-stop reachability: MANUAL_CONFIRMATION_REQUIRED.
- Minimum-motion: MINIMUM_MOTION_BLOCKED; next stage BLOCKED.
- All physical pose/gripper/Home/trajectory/Hold/Stop/rollout commands:0. No getters retried.

## Executed commands and code

After sourcing `/opt/ros/humble/setup.bash` and `/home/ubuntu/robot_ws/install/setup.bash`:

```bash
/home/ubuntu/a0509_vla_linux_field_bundle_20260903/environment/a6000_ubuntu22_py310/bin/python tcp_hardware_precheck.py
/usr/bin/python3 -m unittest discover -s tests -v
/usr/bin/python3 -m py_compile tcp_hardware_precheck.py
```

Added tcp_hardware_precheck.py (allowlisted getter-only, single persistent subscriber) and tests/test_tcp_hardware_precheck.py. Integration-directory tests51 PASS/0 FAIL/0 SKIP. Physical hardware validation and whole-repository testing are not claimed.
