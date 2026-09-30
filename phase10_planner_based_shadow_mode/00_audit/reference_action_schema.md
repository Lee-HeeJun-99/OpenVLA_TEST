# Reference action schema

Canonical order is `[dx_m, dy_m, dz_m, dRotVecX_rad, dRotVecY_rad, dRotVecZ_rad, gripper_closedness]`. Translation is the adjacent difference between recorded commanded poses. Rotation follows the collector's saved RPY→quaternion→relative-rotvec convention and is labelled with that assumption. Gripper is the following sample's commanded closedness. The final sample has zero pose delta and final gripper command.

Every row retains both poses, phase, source and receive timestamps, base reference, `pose_source`, validity, and explicit absence of measured feedback. See `04_planner_reference/reference_command_schema.json`.
