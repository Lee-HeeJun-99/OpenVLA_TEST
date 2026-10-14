# Gripper / Stop staged validation

Previous ARM_ONLY_SHORT_HORIZON_PASS results frozen unchanged. Fresh REAL/AUTO/STANDBY and JointState runtime readiness PASS; operator confirmed current gripper OPEN and four physical safety conditions.

Historical real collection code uses tool output2 for CLOSE, active1/inactive0,1second pulse. One controlled CLOSE logical operation was executed: output2=1,1second, output2=0. Both IO service calls returned success=true, ACK latencies5.125ms and5.085ms. Arm commands0, physical Stop/Hold0, logical gripper operations1, physical IO service requests2. Automatic repeats disabled.

Operator explicitly reported: "계속 open 상태야". Actual state OPEN, not requested CLOSED. Result **GRIPPER_MAPPING_FAILED**. ACK success is not physical jaw feedback. No claim of inverted polarity: this shows only that the tested output2/1second command failed to close under current setup. It does not prove whether wiring/channel, tool power configuration, driver output path, pulse width, pneumatic supply or another hardware condition is responsible.

Production config remains polarity_confirmed=false, abort_value=null. No unverified value was promoted, no longer pulse or opposite channel was automatically attempted. Deactivation output2=0 was performed; jaw failsafe behavior remains unverified.

Stop-path source audit and guarded small-motion test script are prepared, but no physical Stop test was launched after this failed stage. Multi-step readiness **BLOCKED**. Additional arm/motion/gripper/Stop commands0. Next: read-only IO/power/wiring/interface inspection, then a separately approved controlled gripper retest based on evidence. Prior arm PASS is not invalidated by this separate gripper test.
