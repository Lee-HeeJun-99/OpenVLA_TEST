# Reference trajectory definition

Classification: `SCRIPTED_REFERENCE_TRAJECTORY`.

1. The task-space route is calculated before motion from start/alignment/grasp/lift poses.
2. Current robot state supplies the starting TCP and arrival checks, but no online replanning occurs.
3. No obstacle model or IK-driven path modification is present.
4. The robot receives blocking absolute base-frame MoveLine waypoints; the logger reconstructs linear interpolation over actual segment duration.
5. A per-frame reference command is available as the interpolated `tcp_pose`, not as acknowledged driver execution.
6. Adjacent recorded poses produce frame-level Reference Commands with phase and gripper command.
7. The same mechanism can be configured again, but reproducibility of physical execution requires local controller/hardware validation and does not imply identical measured motion.

Required terminology: `Reference Command`, `scripted reference trajectory`, or `reference controller`. Do not call it Planner GT, motion-planned ground truth, or measured action.
