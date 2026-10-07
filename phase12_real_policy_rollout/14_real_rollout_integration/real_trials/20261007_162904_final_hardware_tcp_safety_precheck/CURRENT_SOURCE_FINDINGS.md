# Final source/evidence interpretation — 2026-10-07

Starting HEAD58281968dc4636905ec1cbb4e1d619f6367648f7 / branch lhj-research. Driver/controller not restarted. Physical command0, getter requests0 in this trial.

## JointState: current runtime FAIL

FIRST_FRESH_SAMPLE delay10.6298 s; fixed10 s warm-up; runtime start monotonic2246727.952253882. The initial transition was READY but is not an overall PASS. Thereafter receive timeout latched RUNTIME_FAULT. First observed post-warm-up gap: receive2246728.542680400 →2246728.740839304,198.159 ms; associated source gap19.994 ms. Next source jump190.259 ms. Later runtime receive3.066333/3.067845 s and source3.060001/3.060006 s events were observed after readiness. All remain preserved in jointstate_runtime_events.json (phase RUNTIME_FAULT means latched runtime fault, NOT startup).

Final latest receive age2.921 s; post-precheck FAIL. No window reset and no threshold relaxation. CLI graph inspection/extra passive TF subscriptions coincided with this integrated observation, so the data do NOT prove that the driver alone caused these gaps. Publisher source/host scheduling root cause is not established. No getter was called, so these current gaps cannot be attributed to a getter request from this trial.

## TCP, tool, Cartesian sources

- Historical verified-config candidate says Tool_v1/zero offset; current active-name binding NOT proved. Never adopted.
- GetCurrentTcp/GetCurrentTool source1938/1967: DRFL name string only; previous empty responses remain UNKNOWN; not repeated.
- GetCurrentPose source370 dereferences vendor pointer without checking null: UNSAFE_OR_NULL_RISK, not called.
- GetCurrentPosx source983 does contain a null check, but vendor network wait has known prior feedback-loss correlation: excluded for that stability risk, not falsely claimed to lack a null check.
- GetCurrentToolFlangePosx source1045 directly copies g_stDrState.fCurrentToolPosx. No vendor call/null dereference in visible callback. READ_ONLY_BUT_STABILITY_UNVERIFIED: cache has no per-response freshness proof, and srv states flange/zero-TCP semantics, not active TCP. Request has ref but visible callback ignores it. Not called.
- RobotState.msg has current_posx/current_tool_posx/access_control/disconnected/safety input fields. RobotStateRt.msg explicitly documents actual_tcp_position and flange as base-frame mm/EulerZYZ degrees. Schema existence is not a live stream. Current direct graph did not expose fresh RobotState/RobotStateRt/actual_tcp_position samples; hardware_raw_sources messages empty.
- TF observed link_5→link_6, not a verified active controller TCP frame. SO101 and ZED pose frames are not Doosan TCP.
- Optional RT publication in dsr_controller2.cpp146 reads read_data_rt(), checks pointer then publishes configured keys. Current graph did not show enabled actual_tcp_position RT output. No RT start/connect/parameter changes attempted.

## Hardware state/connection/access control

dsr_hardware2/src/dsr_hw_interface2.cpp140–181 handles connection and static startup get_control_access/is_standby via monitoring callbacks. Startup logs/initial grant do not provide current affirmative freshness.

dsr_controller2.cpp3022/3071 stores g_stDrState.nRobotState/nAccessControl and g_bHasControlAuthority. These callbacks can reset stops, servo-on, change mode or request access in existing driver logic; this audit does not invoke them or assume they are passive queries. Error/disconnection publishers at2373/2374 and callbacks publish events; silence cannot prove connected/protection clear.

Dedicated current servo status is not established. STANDBY does not imply servo=true. Brake/motor-current fields are not silently mapped to servo readiness. LastAlarm getter is not called because vendor pointer/live stability is unverified and the current JointState gate has faulted; zero alarm would not alone prove stop-clear.

Mode=AUTO/state=STANDBY/system=REAL from prior same-session trial20261007_162114 are timestamped DRIVER_REPORTED snapshots, not fresh current readiness evidence. Planned refreshes were all inhibited by runtime fault. Authority/servo/protective/E-stop/connection/gripper state remain UNKNOWN. Gripper output/pulse and model gripper application remain disabled; output state is not measured physical gripper position.

## Current camera/model

Camera1280×720 BGRA8 fresh snapshot, actual RGB/JPEG→OpenVLA sample inference; selected-frame age0.439384 s <0.5 s. MODEL_HEALTH_PASS current strict identity/processor, step8130/K1/action_dim7/proprio=false. This is a short current sample check, not a new30 s continuous Shadow.

## Final gate and tests

MINIMUM_MOTION_BLOCKED. Missing current clean JointState, verified TCP/tool transform/current Cartesian pose, affirmative hardware safety sources, and manual operator/workspace/E-stop confirmations. Next stage BLOCKED; no minimum-motion executed.

Added final_hardware_tcp_precheck.py and3 tests. Integration-directory unittest54 PASS/0 FAIL/0 SKIP; compile PASS. Whole-repository and physical validation are not claimed.

Executed after sourcing ROS Humble and robot_ws/install: bundle Python final_hardware_tcp_precheck.py, /usr/bin/python3 -m unittest discover -s tests -v, py_compile. ROS topic/service CLI and direct graph snapshots preserved in ros_graph_snapshot.json; no CLI result promoted to current controller state.
