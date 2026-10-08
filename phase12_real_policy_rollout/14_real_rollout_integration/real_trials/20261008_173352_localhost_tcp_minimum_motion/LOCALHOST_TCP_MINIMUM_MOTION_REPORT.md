# Localhost-fixed TCP / minimum-motion current-session preflight

Result: MINIMUM_MOTION_NOT_EXECUTED. Next: BLOCKED. Physical commands: 0. No command transport, setter, gripper, Hold/Stop or model physical command was executed. Driver/ZED processes were not restarted.

Branch lhj-research, HEAD 66628544f98b2728a1b6fd70a90f73058128558f. Actual driver/ZED/passive collector process environments: ROS_LOCALHOST_ONLY=1. DDS root-cause investigation was not repeated and historical 3-second gaps are not current blockers.

The same JointState subscriber completed consecutive 10-second clean readiness, a 30-second current-session window, three reviewed scalar getter calls (one each), and a 10-second post-getter watch. Runtime coverage including post-watch: 40.148 s, 4,016 samples. Max source gap 12.809 ms; max receive gap 21.356 ms; >=100 ms events, invalid samples, duplicates and regressions: 0. JOINTSTATE_CURRENT_SESSION_PASS.

Current mode/state/system getter responses are AUTO/STANDBY/REAL. They do not independently establish authority, servo or protective/emergency stop clearance. Those values remain UNKNOWN.

TCP source investigation: RobotState/RobotStateRt definitions exist but no current live topics were discovered. No RT topics were discovered; installed controller config has use_rt_topic_pub=false. No read exposure for current active TCP six-dimensional offset/binding was verified in the audited API/config. GetCurrentTcp/GetCurrentTool empty-string responses were not repeated or interpreted as zero offset. GetCurrentPose pointer null risk, GetCurrentPosx prior stability risk and flange-cache freshness limitations prevent their use as verified active TCP evidence.

Current passive TF captured 14 frame edges. The robot chain terminates at link_6, with no tool/TCP child. TF-derived flange is not active TCP. The offset and current TCP remain unverified: TCP_FLANGE_ONLY. No target pose was generated.

Existing command contract is absolute BASE (ref=0), mm / ZYZ degrees; minimum protocol is base X +0.5 mm, zero rotation delta, no gripper, maximum one action. It was not executed because verified TCP and actual manual/hardware preflight evidence are missing. No alternative joint/flange command path was created, and no threshold was changed.

Operator presence, physical workspace clearance, E-stop accessibility and stationary confirmation were requested together; no actual confirmation has been received. Current OpenVLA health additionally reports connection refused, which must be resolved before subsequent model rollout but is not a claim that deterministic motion requires model output.

Required next evidence: active TCP offset/current pose and matching units/base frame from a verified controller source or pendant, plus actual manual confirmations and authority/servo/stop-state evidence. No automatic short-horizon follows this blocked precheck. Motion ACK, observed delta and E-stop outcome are N/A, not fabricated physical results.
