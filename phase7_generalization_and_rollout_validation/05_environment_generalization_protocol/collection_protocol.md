# Phase7 Environment Generalization Collection Protocol

Status: `DATA_REQUIRED`

Collect single-factor conditions before composite conditions.

Object positions:
- left/center/right x near/middle/far = 9 positions

Lighting:
- low / normal / high
- Record lux if possible.

Camera:
- baseline / shift_left / shift_right / shift_up / shift_down / pitch_change / yaw_change
- Record measured extrinsic or pixel offset.

Keep robot home, instruction, object identity, gripper initial state, frame rate, and task protocol fixed where possible.
