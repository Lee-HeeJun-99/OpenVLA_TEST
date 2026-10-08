# SAFE_OFF recovery / start-position / single prediction preflight

Final: BLOCKED. Controller recovery is not confirmed: current RobotState=3, SAFE_OFF. No software recovery/mode/servo/safety setters, reset motion, model physical commands, gripper or physical Hold/Stop calls were issued.

Current read-only responses (each getter called once): RobotSystem=0 REAL; RobotMode=1 AUTO; RobotState=3 SAFE_OFF. Exact response timestamps and latencies are in hardware_state_after_safeoff.json. Operator was requested to use the established pendant/controller recovery procedure, without tool/TCP changes, jog or arbitrary motion. No recovery-complete input or pendant TCP evidence was received.

JointState used the existing FIRST_FRESH_SAMPLE -> consecutive 10 clean seconds -> RUNTIME_READY state machine in one process. Runtime fault=none, events>=100 ms=0. A 10-second post-getter watch measured 1,010 samples, max source/receive gap 12.024/14.866 ms. DDS/root-cause investigation was not repeated; thresholds remain unchanged.

JointState was reordered by joint names. Current joint degrees: [-182.69436437236462, -1.0601918880880168, -40.97306856666672, 1.2424241099083073, -113.40407599744456, 1.454241296118508]. Exact target and per-joint errors are in jointstate_start_pose_check.json. Maximum absolute error 0.141594 degrees <= existing 0.5-degree tolerance: START_POSE_POSITION_MATCH=true. Reset was deliberately skipped; this is not physical joint-command or Cartesian TCP path validation.

Current OpenVLA health/processor identity PASS: ready, vanilla_s1_balanced_step8130, openvla_token, chunk_size=1, action_dim=7, requires_proprio=false. Existing server remains running. A prediction-only one-request path is prepared in the same subscriber process, but it was NOT_EXECUTED because current hardware is not STANDBY. No synthetic raw action or inference latency is reported.

TCP/action contract remains unverified. Flange is not assumed to be TCP; no zero offset is applied. No new pose getter was used. After normal pendant recovery, verified active TCP/offset/current pose is still required for physical OpenVLA Cartesian actions.

Short horizon NOT_STARTED; prediction count=0, command count=0. Preflight block is not a task FAILURE or INVALID trial. E-stop use and unexpected physical motion are N/A because no rollout executed. Next: BLOCKED until normal hardware recovery and verified TCP evidence. Tests: 11 startup/runtime, 7 joint reset guard, 5 single-prediction hardware gate tests PASS.
