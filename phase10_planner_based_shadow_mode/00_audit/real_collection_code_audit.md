# Real collection code audit

Audited working files at robot workspace HEAD `86eaa9632d651eb907332334d02f32c1461850d7`: `single_robot_simple.py` is modified and `collect_episode001_from_align_home.py` is untracked. Neither was changed.

`collect_episode001_from_align_home.py` is safe only without `--execute`: it resolves layout/reference data, prints the proposed HOME→alignment→descent→close→lift sequence, and exits before ROS initialization or output replacement. With `--execute`, it can back up/replace the episode directory, initialize ROS, set autonomous/full-speed mode, actuate the tool IO gripper, move to HOME, issue absolute base-frame MoveJ/MoveL service or H2R action commands, record, postprocess actions, rebuild dataset indices, and optionally run a Sim replay. It must not be executed remotely.

Layout JSON supplies `episode_id` and `real_layout_mm.orange`; that ID changes the output directory. Otherwise the explicit/default episode path determines the ID. Reference route poses come from metadata or phase endpoints in reference `steps.jsonl`; target XY and Z offsets may adjust them. The TCP mode uses three fixed task-space waypoints. The joint-delta mode offsets every reference joint sample from a fixed Real HOME. Neither performs online obstacle avoidance, IK replanning, or environment-conditioned path repair.

`single_robot_simple.py` generates a 6×6 grid, uses a fixed unrecorded start joint pose, and constructs alignment/grasp/lift absolute base poses with a constant orientation. Waypoints are executed by blocking MoveLine; the recorder independently linearly interpolates start and target poses over the observed segment duration at nominal 5 Hz. Defaults are 200 mm/s, 200 deg/s, 200 mm/s², and 200 deg/s². Holds surround alignment, grasp, close and lift. Tool digital outputs pulse index 1 for open and 2 for close; saved gripper semantics are commanded `0=open, 1=closed`. No measured gripper feedback is recorded.

Images, JointState, commanded/planned TCP pose, phase, image/source/episode timestamps, commanded gripper state, and age/error counters are logged. With the default planned-pose setting, measured robot pose is not the action source. `process_episode` computes adjacent commanded-pose actions. Episode 4 confirms 45 planned samples, zero feedback samples and `pose_source=planned_commanded_pose`.

Exceptions mark failure metadata and stop the recording thread; they do not turn the commanded path into measured ground truth. Command timestamps are not stored at the Doosan driver execution boundary.
