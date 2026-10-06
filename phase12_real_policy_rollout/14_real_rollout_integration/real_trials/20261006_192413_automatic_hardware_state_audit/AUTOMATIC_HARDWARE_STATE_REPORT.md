# Automatic hardware state audit

JointState: FAIL, 0 messages, 0.000 Hz, max source gap None, receive gap None.

Camera: PASS, 741 frames. This passive check does not validate inference selected-frame age.

TCP: TCP_FLANGE_ONLY; active name/tool/offset/current controller Cartesian pose UNKNOWN.

Robot mode/servo/protection/emergency/authority remain UNKNOWN: no fresh complete state signal verified. Event-topic silence does not prove safety.

Manual items: MANUAL_CONFIRMATION_REQUIRED (operator, workspace clear, E-stop access).

Minimum-motion: BLOCKED. Physical commands: 0; getter requests: 0; publisher/client/action capability: none.

See raw_observation.json for current endpoints, direct discovery, messages and driver rosout collected during this window. Historical source/config information is not current hardware evidence.

## Supplemental evidence

- Branch `lhj-research`, HEAD `d9287a00b7e976275516465a8c0ff93cf0efe8e2`; initial worktree clean. Real runtime and safety configs unchanged.
- Driver PID 3017586 remained present, CPU 203%. Process existence does not prove controller connection or feedback health.
- CLI node discovery omitted controller-manager/controller/broadcaster names; direct service discovery subsequently found `/dsr01/tcp/get_current_tcp` and `/dsr01/tool/get_current_tool`. Discovery methods differ; this is not proof of process crash.
- JointState publisher offered RELIABLE/TRANSIENT_LOCAL. Audit subscriber requested BEST_EFFORT/VOLATILE, a compatible combination, but received zero samples. This does not by itself distinguish driver failure, discovery delay or transport loss.
- Camera: 741 frames / 20 seconds, receive rate 37.034 Hz, latest age 0.0203 s, max source gap 0.133363 s, max receive gap 0.134503 s; 1280×720 `bgra8`. Selected-frame inference freshness was NOT tested.
- Current `/dsr01/robot_state` and `/doosan/current_pose` publisher counts were zero. Error/disconnection endpoints existed but no messages arrived. These are event channels, not a continuous affirmative safety state.
- Installed `dsr_controller2.yaml:35` has `use_rt_topic_pub: false`; conditional RT-field publisher code is at `dsr_controller2.cpp:120–153`. No config/parameter changes were made to enable it.
- Driver cache holds monitor/access-control information (`OnMonitoringStateCB`, `OnMonitoringAccessControlCB`), but cache presence in C++ is not a ROS read interface or fresh current evidence.
- Name getters wrap `_get_tcp(_rbtCtrl)` / `_get_tool(_rbtCtrl)` in vendor code, whose actual body was not found in available source. Visible callbacks contain no setters, but transport/blocking behavior is unverified. Their hardcoded `success=true` cannot validate name freshness or offset validity. No getter meets SAFE_READ_ONLY live-stability criteria for this audit; both were skipped, especially with JointState gate failure.
- TCP database interface found in source is creation/deletion/selection (state-changing), not a verified read API returning active name plus 6D offset. Runtime config search supplied no verified current active-name/offset binding. Historical Tool_v1 values were not reused.
- `TCP_FLANGE_ONLY` describes the available URDF/FK source capability, NOT a fresh flange measurement: this run received no JointState and therefore computed no fresh Cartesian pose. Current Cartesian value is NOT_FOUND.
- Driver log tail contained historical mode/movej callbacks and older Skip-dt warnings; no new driver rosout appeared in this observation window. Historical external commands are not commands issued by this audit and were not relabelled as current events.
- First invocation with Conda Python 3.14 failed before ROS initialization (Humble ABI mismatch). Rerun with `/usr/bin/python3` succeeded. Python compile check PASS; measurement execution PASS; hardware readiness remains FAIL/BLOCKED.

Commands executed: Git branch/HEAD/status, ROS node/topic/service/action lists, passive subscriber script, source `rg`/`sed`, process `ps`, log `tail`, Python compile check. No parameter/getter/motion service call, action call, publisher, restart, tool/mode/servo change occurred.

All physical command categories (robot, motion service/action, gripper, Home, trajectory, Hold/Stop, real rollout): **0**.
