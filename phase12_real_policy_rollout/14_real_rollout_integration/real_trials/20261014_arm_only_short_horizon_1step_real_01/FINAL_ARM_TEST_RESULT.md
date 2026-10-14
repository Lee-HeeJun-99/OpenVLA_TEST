# ARM_ONLY_SHORT_HORIZON_PASS

Operator explicitly confirmed only the expected small motion, no gripper movement, no unexpected motion and no E-stop usage. Recorded2026-10-14 08:57:45UTC. This supersedes the pending operator-confirmation status in the earlier technical report; raw command/feedback logs remain unchanged.

Physical Cartesian arm command1, gripper0, physical Hold/Stop0. Prediction1. Existing synchronous MoveLine ACK success,556ms. Requested limited translation norm0.800mm; measured-joint FK flange displacement0.806688mm, difference0.007924mm; direction cosine0.999986. Requested rotation0.642879deg; observed0.641841deg. Runtime JointState source/receive >=100ms events0; max12.043/15.126ms, final velocities0. Post-motion STANDBY.

Final result ARM_ONLY_SHORT_HORIZON_PASS based on ACK, independent JointState/FK movement and actual operator confirmation. Evaluation SYSTEM_INTEGRATION_TEST, task_success=null; no task success-rate contribution. Formal TCP classification remains TCP_ZERO_OFFSET_SUPPORTED_NOT_VERIFIED; scoped engineering acceptance is not formal TCP verification. Gripper mapping and physical Stop/abort path were not physically validated by this test. No additional motion, automatic retry, batch, home or recovery performed. Next ANALYZE_ARM_TEST, not automatic full-policy rollout.
