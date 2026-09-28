# Real runtime inventory

Audited root: `/home/ubuntu/robot_ws/src/openvla_doosan_runtime`; ROS 2 Humble, `ament_python`, package version 0.1.0. Git repository `/home/ubuntu/robot_ws`, HEAD `86eaa9632d651eb907332334d02f32c1461850d7` (2026-08-04). The working tree is not pristine: at minimum the action adapter, bridge and OpenVLA inference are modified, while OFT/replay/teleop/servo-test nodes are untracked. Results describe the working files, not HEAD alone.

## Files and roles

- `camera_adapter_node.py`: ZED Image relay preserving header; reusable concept, live node not run here.
- `openvla_inference_node.py`: OpenVLA model, image/instruction/enable subscriptions, raw-action publisher; existing closed-loop publisher makes it reference-only for Shadow replay.
- `openvla_oft_inference_node.py`: OFT extension, JointState and gripper-command proxy inputs, checkpoint/model setup; reuse model loading logic with offline adapter only.
- `action_adapter_node.py`: raw 7-D delta to absolute Doosan pose, clipping/workspace/gripper hysteresis; authoritative unit mapping, unsafe to execute remotely.
- `doosan_bridge_node.py`: robot services, stream publishers, pose polling, filtering, gripper IO and stop; robot-control entry point and strictly do-not-run.
- `episode_manager_node.py`: episode services and enable/instruction/emergency publishers; unsafe to execute remotely.
- `dataset_episode_replay_node.py`: replay publishers; untracked and do-not-run even though it offers dry-run.
- `steamvr_teleop_node.py`, `servol_stream_test_node.py`: direct command generators; do-not-run.
- `common.py`: array validation/message helper; reusable validation ideas.
- `scripts/analyze_vla_bag.py`, `visualize_vla_tracking.py`: offline analysis/logger consumers; reusable offline.
- `launch/runtime.launch.py`: starts camera, inference, action adapter, bridge, manager; closed-loop, do-not-run.
- `launch/dataset_replay.launch.py`: starts bridge plus replay; do-not-run, including remote dry-run.
- `launch/steamvr_teleop.launch.py`: command path; do-not-run.
- `config/runtime.yaml`, `runtime_oft.yaml`: actual topic/unit/safety parameters; static source of truth pending local confirmation.

## Planner/logger assessment

No distinct motion-planner node, planner raw-action topic, comprehensive synchronized logger, measured gripper feedback, or robot home routine was found in this package. The replay node is a trajectory command generator, not planner GT. Existing status strings/bag analysis are not the required integrated logger. Phase 10 therefore wraps observations without changing runtime source. Direct modification should be deferred until local operator review establishes a real planner interface and timestamped feedback contract.

## Reuse classification

- Reuse as reference: unit conversion, workspace bounds, model preprocess/load logic, bag analyzers.
- Reuse with offline wrapper: observed camera/state/target/stream schemas.
- Do not execute: all launch files and nodes capable of publishing `/vla/target_pose`, `/vla/enable`, `/vla/emergency_stop`, stream messages, motion/IO services.
- Missing/blocking: planner identity/action, measured gripper state, timestamped EE pose/executed command, hold acknowledgement, physical E-stop state, home/gripper-home verification, joint-limit source.
