# Planner Reference Interface

Status: `PARTIALLY_COMPLETED`

## Recorded reference

`RecordedPlannerEpisode` reads an existing `metadata.json` and `steps_with_actions.jsonl` without publishing. It preserves pose provenance. Episode 4 is usable for pipeline smoke tests, but its pose source is `planned_commanded_pose`, not measured feedback.

## Live reference

`LivePlannerInterface.execute()` is deliberately blocked. Real integration must expose, with common IDs/timestamps:

- planner target/waypoint and phase;
- planner raw delta action;
- safety/postprocessed command;
- command actually delivered to the robot;
- pre/post measured joint, TCP and gripper states;
- hold, timeout, communication and E-stop state.

Only the planner may control the robot in Shadow Mode. OpenVLA/OFT outputs must have no route to a command publisher.

## Existing external reference

The external ROS workspace contains `collect_episode001_from_align_home.py`, `single_robot_simple.py`, and `dataset_episode_replay_node.py`. They are useful references but are dirty/untracked relative to their Git repository and therefore are not production dependencies of Phase 10.

