# OpenVLA + Doosan A0509 runtime

## Nodes

- `camera_adapter`: ZED image -> `/vla/image_rgb`
- `openvla_inference`: image + instruction -> `/vla/raw_action`
- `action_adapter`: delta action + current pose -> safe absolute target
- `doosan_bridge`: Doosan services, pose polling, motion and gripper execution
- `episode_manager`: start/stop, maximum step count and timeout

The pipeline permits only one inference and one robot command at a time.

## 1. Place package

```bash
mkdir -p ~/ros2_ws/src
cp -r openvla_doosan_runtime ~/ros2_ws/src/
```

The official Doosan Humble package must also exist in the same workspace:

```bash
cd ~/ros2_ws/src
git clone -b humble https://github.com/DoosanRobotics/doosan-robot2.git
```

## 2. Dependencies and build

```bash
cd ~/ros2_ws
rosdep install -r --from-paths src --ignore-src --rosdistro humble -y
colcon build --symlink-install
source install/setup.bash
```

OpenVLA Python dependencies must be installed in the Python environment used by
ROS2. In particular: `torch`, `transformers`, `timm`, `tokenizers`, `Pillow`,
and optionally `flash-attn`.

## 3. Edit configuration

Edit:

```text
config/runtime.yaml
```

The following values must be verified before real motion:

1. `model_path`
2. `unnorm_key`
3. camera topic and preprocessing
4. action unit: m/mm and rad/degree
5. action type: delta or absolute
6. axis signs and coordinate frame
7. workspace limits
8. gripper digital output index and ON/OFF wiring

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
  host:=192.168.137.100 \
  port:=12345 \
  model:=a0509 \
  name:=dsr01
```

Start the ZED wrapper separately, then check:

```bash
ros2 topic list | grep zed
ros2 service list | grep dsr01
```

## 5. Start runtime

```bash
source ~/ros2_ws/install/setup.bash

ros2 launch openvla_doosan_runtime runtime.launch.py \
  config:=$(ros2 pkg prefix openvla_doosan_runtime)/share/openvla_doosan_runtime/config/runtime.yaml
```

## 6. Dry checks before enabling motion

```bash
ros2 topic echo /doosan/current_pose
ros2 topic echo /vla/raw_action
ros2 topic echo /vla/target_pose
ros2 topic echo /doosan/status
```

For the first test, keep the controller in a safe condition and verify that the
target changes by no more than 5 mm per action.

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
/dsr01/io/set_ctrl_box_digital_output
```

Confirm the names on the installed driver:

```bash
ros2 service list | grep -E "current_posx|move_line|move_stop|digital_output"
ros2 interface show dsr_msgs2/srv/MoveLine
ros2 interface show dsr_msgs2/srv/GetCurrentPosx
ros2 interface show dsr_msgs2/srv/SetCtrlBoxDigitalOutput
ros2 interface show dsr_msgs2/srv/MoveStop
```
