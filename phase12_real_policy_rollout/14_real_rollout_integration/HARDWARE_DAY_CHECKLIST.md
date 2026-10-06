# Hardware-day checklist

No physical command was executed during software preparation. Default configurations remain disabled. Check each item on the actual hardware; UNKNOWN is not PASS.

1. Connect the controller/robot; confirm approved local operator and immediate physical E-stop access.
2. Confirm E-stop, protective stop, alarms, Hold/stop response versus physically observed standstill. Service ACK alone is not safe standstill.
3. Confirm actual servo, MANUAL/AUTO and authority sources. Driver has GetRobotMode and optional RT robot_mode/state; do not repeat previously unstable getters. No dedicated servo-enabled stream was identified: **HARDWARE_SOURCE_NOT_AVAILABLE_IN_CURRENT_SOFTWARE_INTERFACE**. An explicitly documented operator-confirmed policy is supported, expires after 60 seconds, and never becomes measured feedback.
4. Verify sustained JointState and measured TCP or approved FK/tool transform. Configure verified topic names/types; do not launch command-capable bridge merely to obtain TCP.
5. Test physical gripper polarity once under separate approval; approve abort output value and pulse timing. Command knowledge remains distinct from measured state.
6. Measure workspace, grasp/pregrasp pose and lift threshold; confirm configured velocities/accelerations against actual robot limits. Production limits are not automatically relaxed.
7. Run the actual GPU model verifier for OpenVLA8130 and visionOFT28560 separately. **GPU_RUNTIME_VALIDATION_PENDING** until those executions pass. OFT early-close behavior review requires explicit approval; software readiness does not approve the checkpoint's physical use.
8. Run live prediction-only dry-run. All delivered/executed commands must remain null and command-issued false. Inspect stale inputs, hashes, clock domains and geometric phase labels.
9. Grant fresh explicit stage approval, then minimum-motion. Verify one translation-only command and physical orientation/units, Hold readiness and its result artifact.
10. Grant separate approval for short-horizon; verify attainable cadence/no-overlap and real feedback. Target 5 Hz is not guaranteed achievable by synchronous MoveLine.
11. Grant separate approval for full rollout. Record object grasp/lift outcome independently: sequence completion is not physical task success.

All items are live hardware, GPU-runtime or operator approval checks, not instructions to execute them now. Abort or any UNKNOWN preserves fail-closed behavior. No automatic progression from dry-run artifacts is permitted.
