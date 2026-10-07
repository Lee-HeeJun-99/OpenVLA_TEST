# Source/hardware audit and interpretation

## Baseline / preservation

Branch lhj-research; starting HEAD1fae6eb1556de7357e4136a1c108f20b7c5debbc; initial worktree clean. Driver PID3061204 / launch3061188 retained throughout. RViz was already off. Camera launch3062081 and model3062167 were normally stopped for A. B camera/model C restored as3066471(container)/3066655(server); after the measurement process ended those child processes exited, so verified standalone launch commands restored camera launch3075939/container3075971 and model3076021. Driver PID unchanged. No other command-capable nodes were launched.

ROS_DOMAIN_ID/RMW_IMPLEMENTATION unset in sourced shell; ROS_LOCALHOST_ONLY=0; observer runtime library reports rmw_fastrtps_cpp. UNSET is preserved rather than claiming an explicitly configured domain. Domain0 is the ROS default, not independent controller evidence.

## A-D and perturbation evidence

| Run | Runtime | Result | Count | Hz | Source max ms | Receive max ms | Source>=100 ms | Receive>=100 ms |
|---|---:|---|---:|---:|---:|---:|---:|---:|
| A minimal |60 s|PASS|6000|100.001|11.242|28.598|0|0|
| B camera |60 s|FAIL|1116|18.594|3069.994|3072.744|17|17|
| C camera+prediction |60 s|PASS JointState|6000|100.001|11.728|19.132|0|0|
| D integrated |60 s|FAIL|3610|60.160|3060.022|3071.338|9|9|

Counts are adjacent-sample threshold violations, not unique physical stall incident counts. Startup/warm-up preserved separately; runtime windows never reset. FIRST_FRESH delay A11.120/B0.991/C0.009/D0.001 s: B-D reuse the existing DDS participant, so these are new-condition fresh sample delays, not new-participant discovery times. Warm-up event counts A3/B13/C0/D0.

Same-process feature-off30 s results: without TF PASS, tf_static PASS, rosout PASS (receive maximum92.016 ms), dynamic PASS, CLI PASS, hash PASS, HTTP PASS. Initial without-services condition recorded receive113.743 ms/FAIL, BUT dynamic discovery and CLI still queried service graph; its feature-isolation interpretation is INVALID. Raw measurement remains preserved. Software flag separation was corrected and tested. Corrected follow-up used a NEW experimental participant; strict health first failed because the child model server had ended; that failed attempt is preserved in../20261007_165024_complete_rollout_readiness_audit. After independent server/camera restoration, ../20261007_165128_complete_rollout_readiness_audit had60 s discovery timeout and no runtime window. It is NOT a successful60 s corrected service-off comparison.

Seven shorter off conditions passing does not isolate one culprit. Sequential order, condition-transition DDS settling, durations and shared participant history are confounders. A/C PASS disproves persistent total publisher failure in those intervals, not intermittent failures. Higher load C (max load1=10.766) PASS while B (4.710) FAIL rules out a simple load threshold explanation. D max load10.068; A3.496.

All sampled interface RX/TX drop/error counter deltas0 in A-D and perturbation runs; this does not exclude packet loss elsewhere. Driver end-of-run ps %CPU~202, memory0.3%, threads39 across primary conditions. ps %CPU is a lifetime average, not100 ms instantaneous driver CPU. ZED~108–112% lifetime CPU/3.2% memory; model snapshots~143%→95% lifetime CPU/6.6–6.8% memory. Raw host1 s network/load records retained. No perf/packet trace or publisher instrumentation was performed.

Final JointState classification JOINTSTATE_STILL_INCONCLUSIVE, runtime safety reliability NOT globally approved. Observer/discovery sensitivity remains plausible but not proven as sole cause.

## Camera/model: current trial, not historical PASS

C147 predictions/60 s with zero nonfinite/model-schema failures. Camera age maximum0.519079 s,1 threshold failure; cold-start BEFORE C runtime0.655821 s also retained separately (inference0.642014 s). Cold sample was not silently used to pass the runtime camera gate.

Translation median3.208621 mm/max4.613488 mm;4 actions exceed production4 mm step. Rotation median1.360025 degrees/max3.739539 degrees. Inference median0.391445 s; close candidates>=0.7:0. Thus current GPU identity/processor/inference PASS, action sanity FAIL and selected-frame camera gate FAIL. No threshold tuning. JSONL logs actual raw predictions, JointState phase/ages/gaps, command=false/null values. This diagnostic logger does not claim physically validated SafetyPipeline acceptance.

## TCP and Cartesian sources

Current active TCP/tool names remain UNKNOWN; prior empty getter strings not re-requested and not interpreted as zero offset. Fresh config search of doosan source/install, legacy runtime, bundle configs/runtime_state found only historical Tool_v1/zero-offset operator fields and TCP/tool creation API examples. No current controller active-name binding or read-only configured-TCP-list source established. These are CANDIDATES_ONLY, not active configuration.

RobotState schema has current_posx/current_tool_posx/access_control/disconnected. RobotStateRt schema actual_tcp_position is documented base-frame mm/EulerZYZ degrees. Current direct graphs contained error/disconnection/command-stream/IO topics but no live RobotState/RobotStateRt/actual_tcp_position publication. Dynamic dsr subscribers recorded0 actual hardware state messages; no schema promoted to live data. SO101 TCP and ZED pose topics are unrelated. TF does not prove active controller tool/TCP.

GetCurrentToolFlangePosx callback1045 copies g_stDrState.fCurrentToolPosx; OnMonitoringDataExCB2920/2940 updates it from _fActualPos[1] and also updates dSyncTime2957. The source comment says100 ms callback, NOT a measured current update rate. Getter response has pos/success, no cache freshness/sequence/timestamp; ref request is not used in visible copy. RobotState stream that could expose sync_time is unavailable. Therefore NOT READ_ONLY_CACHE_VERIFIED; no call. Its flange/zero-TCP semantics cannot replace active TCP.

GetCurrentPose370 vendor pointer is dereferenced without null check. GetCurrentPosx983 contains null check but retains known vendor wait/feedback-loss risk. GetLastAlarm388 repeatedly dereferences get_last_alarm() without null check; classify UNSAFE_OR_NULL_RISK, not called. Last alarm/error silence cannot prove stop clear.

## Hardware

OnMonitoringAccessControlCB3071 caches nAccessControl and g_bHasControlAuthority; OnMonitoringStateCB3022 caches robot state. Existing callbacks can send access/stop-reset/servo/mode operations; audit does not invoke them. Hardware startup connection/authority locals and historic initial-grant logs are not fresh continuous affirmative signals. No validated current servo power source found. STANDBY never converted to servo=true/protection clear/authority=true.

Previous same-driver-session mode1=AUTO/state1=STANDBY/system0=REAL are timestamped DRIVER_REPORTED observations from trial20261007_162114 only. Current experiment calls NO getters and does not make those old values current safety evidence. Connection/authority/servo/protective/E-stop/motion/gripper measured state remain UNKNOWN. Control-box IO is a bitfield/cache publisher, not verified gripper position feedback or E-stop accessibility. Gripper physical output/pulse/model gripper application are completely absent/disabled in this validator.

Operator presence, physically clear workspace and reachable physical E-stop are MANUAL_CONFIRMATION_REQUIRED.

## Remaining and final verdict

MINIMUM_MOTION_BLOCKED_MULTIPLE; next stage BLOCKED. Remaining are NOT hardware-only: current JointState runtime/discovery uncertainty, camera end-to-end age exceedance, action step exceedances, TCP/tool/current pose binding, independent hardware safety evidence and manual confirmation. Additional matched/repeated observer tests or publisher instrumentation may be needed; no causal resolution is claimed.

Physical robot/motion/gripper/Home/trajectory/Hold/Stop/real-rollout commands0. Getter requests0. Production configs unchanged.

## Code and tests

Added complete_readiness_comparison.py plus5 tests; corrected service flag prevents direct dynamic service query and CLI service list when off. Integration-directory unittest59 PASS/0 FAIL/0 SKIP; compile/diff checks PASS. Not the full-repository or physical test suite.

Executed after sourcing ROS Humble and robot_ws/install: bundle Python complete_readiness_comparison.py; same script --followup-without-services (two attempts, preserved); system Python unittest discover -s tests -v. Normal camera restore ros2 launch zed_wrapper zed_camera.launch.py camera_model:=zed2i; normal model restore ./scripts/serve.sh vanilla 8766 0 127.0.0.1 in bundle. Only exact verified camera/model PIDs were SIGINTed; no driver restart.
