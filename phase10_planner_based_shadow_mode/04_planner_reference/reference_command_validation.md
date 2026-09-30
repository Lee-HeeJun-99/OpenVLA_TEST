# Reference command validation

The extractor is ROS-free and reads `metadata.json` plus `steps.jsonl`. Episode 4 yields 45 records and preserves `pose_source=planned_commanded_pose`, `measured_feedback_available=false`, and `measured_action=null`. Translation is the adjacent saved commanded-pose difference in metres. Rotation exactly reproduces the collector's RPY→quaternion→relative-rotvec convention; this validates dataset compatibility, not the robot controller's native Euler convention. The final record is a zero pose delta with the final commanded gripper state, matching the source postprocessor.

Classification: `SCRIPTED_REFERENCE_TRAJECTORY`. These actions are Reference Commands, not Planner GT or measured ground truth.
