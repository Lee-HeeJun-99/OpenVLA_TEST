# Current-session physical rollout precheck

Verdict: BLOCKED. No physical trial started; physical command count is 0.

Driver and ZED process environments both have ROS_LOCALHOST_ONLY=1. No process was stopped or restarted. Network, ROS_DOMAIN_ID, rt_host, production safety limits and runtime watchdog thresholds are unchanged.

JointState current-session gate: PASS. Initial runtime: 30.006 s, 3,001 samples; maximum source/receive gaps 12.46/35.75 ms, events >=100 ms: 0. A second persistent session performed the three reviewed scalar read-only getters once each, followed by 10 seconds of monitoring: 40.132 s runtime, 4,014 samples; maximum source/receive gaps 12.82/24.42 ms, events >=100 ms: 0. No historical 3-second gap is used as a current blocker.

Current camera sample: PASS (1280x720, bgra8). Selected-frame inference freshness is not validated in this session because OpenVLA health at 127.0.0.1:8766 returned connection refused. No GPU server was stopped or restarted.

Current scalar hardware responses: AUTO (mode=1), STANDBY (state=1), REAL (system=0). These responses do not establish servo, authority or independent stop-state clearance.

Active TCP/tool and offset remain UNKNOWN. Current graph exposes no RobotState/RobotStateRt TCP topic. Existing empty-name getters were not repeated. Unsafe/current-pose getters and cache getter without freshness evidence were not called. FK flange is not actual TCP and zero offset is not assumed.

Required before motion: verified active TCP/offset/current pose with base reference and units; hardware authority/servo/protection evidence; actual operator, workspace, E-stop access and stationary confirmations; current OpenVLA model health; remaining command/hold readiness checks. Minimum-motion protocol disables gripper and uses deterministic base X +0.5 mm, but no request or command was generated.

Minimum-motion: NOT_EXECUTED. Short-horizon: NOT_STARTED. Task outcome and E-stop/physical motion observations are null, not fabricated SUCCESS/FAILURE. No physical trial means the model health preflight failure is not counted as INVALID task performance.

Tests: 11 existing JointState startup/runtime tests and 7 gap/environment tests PASS. Next allowed stage: BLOCKED pending the listed evidence.
