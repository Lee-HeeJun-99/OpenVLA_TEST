# A0509 start-pose / short-horizon execution attempt

Final: BLOCKED. Physical joint/model/gripper/Stop commands: 0. No physical trial started.

User confirmed operator presence, immediate E-stop access, workspace clear, stationary robot, authority, servo and stop clearance in the current conversation, and explicitly requested execution. The existing align.py model was corrected from m1013 to a0509 and the exact ROLL_OUT_START_JOINT_DEG constant was added. The original align.py entry point was not launched because it also calls set_robot_mode. A guarded single-reset wrapper reuses the existing installed DSR_ROBOT2.movej API; no alternative joint/Cartesian transport, Home or gripper path was introduced. Seven guard/name-order unit tests PASS.

The guarded physical execution attempt reached fresh JointState readiness with 10 consecutive clean seconds and no runtime events. It then failed the pre-motion hardware state check, before importing/dispatching the movej command API. Result START_POSE_NOT_EXECUTED; zero motion/stop requests and no leaked command worker. No automatic reset retry took place.

The initial failed guard did not persist its scalar value; a separately identified read-only diagnostic after the block returned robot_state=3 (SAFE_OFF), success=true, timestamp 1791450190.3669925. This is recorded as post-block current evidence, not invented as the original guard value. Future guard failures now persist their raw hardware responses. Controller evidence takes precedence over conflicting manual servo/stop assumptions. No servo/mode setter was called to clear SAFE_OFF.

JointState array ordering is joint_1,joint_2,joint_4,joint_5,joint_3,joint_6, so all comparisons reorder by names. The last preflight sample is within the existing 0.5-degree home tolerance of the exact target; this is only position evidence, NOT physical joint-command validation or verified active-TCP Cartesian validation. Before/after physical delta, ACK, E-stop and unexpected-motion results are N/A.

The existing OpenVLA GPU serve.sh was restored on localhost:8766, GPU0. Current strict health/processor identity validation PASS: vanilla_s1_balanced_step8130, openvla_token, chunk_size=1, action_dim=7, requires_proprio=false. The server is left running. No sample prediction or short-horizon follows the failed hardware guard.

TCP/action contract remains TCP_FLANGE_ONLY and unverified. No active offset is guessed. Safe next action is inspect the pendant/controller SAFE_OFF condition through the established hardware procedure; retry requires valid current hardware state and fresh authorization. OpenVLA Cartesian rollout additionally requires verified active TCP/current pose and existing production safety/stage gates. No action clipping/scaling, threshold relaxation, automatic Home/recovery or physical batch rollout was performed.
