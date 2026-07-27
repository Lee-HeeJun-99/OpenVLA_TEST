# OpenVLA + Doosan A0509 runtime

## Nodes

- `camera_adapter`: ZED image -> `/vla/image_rgb`
- `openvla_inference`: image + instruction -> `/vla/raw_action`
- `action_adapter`: delta action + current pose -> safe absolute target
- `doosan_bridge`: Doosan services, pose polling, motion and gripper execution
- `episode_manager`: start/stop, maximum step count and timeout

The pipeline permits only one inference and one robot command at a time.

## 1. Workspace

This package is already placed under:

```text
~/robot_ws/src/openvla_doosan_runtime
```

The current workspace also contains the Doosan Humble package:

```text
~/robot_ws/src/doosan-robot2
```

## 2. Dependencies and build

```bash
cd ~/robot_ws
source /opt/ros/humble/setup.bash
rosdep install -r --from-paths src --ignore-src --rosdistro humble -y
colcon build --symlink-install --packages-up-to dsr_bringup2 openvla_doosan_runtime
source install/setup.bash
```

OpenVLA Python dependencies must be available in the Python environment used by
ROS2: `torch`, `transformers`, `timm`, `tokenizers`, `Pillow`, and optionally
`flash-attn`.

If OpenVLA is installed only in the `openvla` conda environment, launch the
runtime from that environment and expose the local repo:

```bash
conda activate openvla
cd ~/robot_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
export PYTHONPATH=~/robot_ws/src/openvla:$PYTHONPATH
```

The launch file runs `openvla_inference` through `$CONDA_PREFIX/bin/python3`
when conda is active. To force a specific interpreter:

```bash
ros2 launch openvla_doosan_runtime runtime.launch.py \
  inference_python:=/home/ubuntu/miniconda3/envs/openvla/bin/python3
```

## 3. Edit configuration

The default config is:

```text
~/robot_ws/src/openvla_doosan_runtime/config/runtime.yaml
```

It currently points to the latest local HF-format checkpoint:

```text
~/robot_ws/src/openvla/runs/openvla-7b+doosan_a0509+b16+lr-0.0005+lora-r32+dropout-0.0--image_aug
```

Runtime defaults:

- `unnorm_key`: `doosan_a0509`
- `center_crop_scale`: `0.9` because this checkpoint used image augmentation
- inference log includes model-input image stamp/hash and consecutive action
  repeat count
- action unit: translation `m -> mm`, rotation `rad -> deg`
- action type: delta TCP action
- per-step translation clamp: `3 mm`
- rotation execution: disabled by default
- repeated raw actions: rejected after 5 identical consecutive predictions
- gripper: Tool DO pulse, `open=DO[1] x2`, `close=DO[2] x1`

Verify these values before real motion:

1. `model_path`
2. `unnorm_key`
3. camera topic and preprocessing
4. action unit: m/mm and rad/degree
5. action type: delta or absolute
6. axis signs and coordinate frame
7. workspace limits
8. gripper Tool DO index and pulse wiring

The supplied `action_adapter` assumes:

```text
[dx_m, dy_m, dz_m, dRx_rad, dRy_rad, dRz_rad, gripper]
0 = close, 1 = open
```

Rotation execution is disabled by default.

## 4. Start Doosan and ZED

Example Doosan real-mode launch:

```bash
ros2 launch dsr_bringup2 dsr_bringup2_rviz.launch.py \
  mode:=real \
  host:=192.168.0.110 \
  port:=12345 \
  model:=a0509 \
  name:=dsr01
```

Start the ZED wrapper separately:

```bash
ros2 launch zed_wrapper zed_camera.launch.py camera_model:=zed2i
```

Then check:

```bash
ros2 topic list | grep zed
ros2 service list | grep dsr01
ros2 topic hz /zed/zed_node/rgb/color/rect/image
```

## 5. Start runtime

```bash
cd ~/robot_ws
conda activate openvla
source /opt/ros/humble/setup.bash
source install/setup.bash
export PYTHONPATH=~/robot_ws/src/openvla:$PYTHONPATH

ros2 launch openvla_doosan_runtime runtime.launch.py
```

Wait until `openvla_inference` logs `OpenVLA loaded`.

## 6. Dry checks before enabling motion

```bash
ros2 topic hz /vla/image_rgb
ros2 topic echo /doosan/current_pose
ros2 topic echo /vla/raw_action
ros2 topic echo /vla/target_pose
ros2 topic echo /doosan/status
ros2 topic echo /vla/model_status
```

For the first test, keep the controller in a safe condition and verify that the
target changes by no more than the configured `max_translation_step_mm`.

If the policy is collapsed, the inference node will log lines like:

```text
repeated_action_detected:count=3,...
```

and `action_adapter` will stop the episode once
`raw_action_repeat_count` exceeds `repeated_action_limit`. If image hashes keep
changing while the action repeats, the problem is model/data behavior rather
than a frozen camera topic.

## 7. Start and stop episode

Start:

```bash
ros2 service call /vla/set_episode std_srvs/srv/SetBool "{data: true}"
```

Normal stop:

```bash
ros2 service call /vla/set_episode std_srvs/srv/SetBool "{data: false}"
```

Emergency motion stop:

```bash
ros2 service call /vla/stop std_srvs/srv/Trigger "{}"
```

## Important driver checks

This package targets the official Doosan `humble` branch and uses:

```text
/dsr01/aux_control/get_current_posx
/dsr01/motion/move_line
/dsr01/motion/move_stop
/dsr01/io/set_tool_digital_output
```

Confirm the names on the installed driver:

```bash
ros2 service list | grep -E "current_posx|move_line|move_stop|set_tool_digital_output"
ros2 interface show dsr_msgs2/srv/MoveLine
ros2 interface show dsr_msgs2/srv/GetCurrentPosx
ros2 interface show dsr_msgs2/srv/SetToolDigitalOutput
ros2 interface show dsr_msgs2/srv/MoveStop
```

## Checkpoint switch

A non-image-augmentation checkpoint also exists:

```text
~/robot_ws/src/openvla/runs/openvla-7b+doosan_a0509+b8+lr-0.0005+lora-r32+dropout-0.0
```

If you switch `model_path` to that directory, set:

```yaml
center_crop_scale: 1.0
```
