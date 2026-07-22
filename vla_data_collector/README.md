# A0509 + ZED 2i OpenVLA Data Collector

## Fixed conventions

- Robot: Doosan A0509
- ROS 2: Humble
- Camera: ZED 2i, eye-in-hand
- Gripper: one controller digital-output pin
- Gripper command:
  - `0`: close
  - `1`: open
- Action:
  `[dx, dy, dz, droll, dpitch, dyaw, gripper]`
- Position unit: meter
- Rotation unit: radian

## 1. Copy package

```bash
cp -r vla_data_collector ~/ros2_ws/src/
cd ~/ros2_ws
colcon build --packages-select vla_data_collector --symlink-install
source install/setup.bash
```

Dependencies:

```bash
sudo apt update
sudo apt install -y ros-humble-cv-bridge python3-opencv python3-numpy
```

## 2. Start A0509

Replace the IP with the actual controller IP.

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash

ros2 launch dsr_bringup2 dsr_bringup2_rviz.launch.py \
  mode:=real \
  host:=192.168.137.100 \
  port:=12345 \
  model:=a0509 \
  name:=dsr01
```

## 3. Start ZED 2i

```bash
ros2 launch zed_wrapper zed_camera.launch.py camera_model:=zed2i
```

Find the image topic:

```bash
ros2 topic list | grep -E 'rgb|left|image'
ros2 topic hz /zed/zed_node/rgb/color/rect/image
```

If the actual topic differs, change `config/collector.yaml`.

## 4. Confirm the gripper output pin

The code assumes a single controller digital output:

```text
DO pin = 0 -> close
DO pin = 1 -> open
```

Edit:

```yaml
gripper_output_index: 1
```

to the actual output number.

Test only after confirming the wiring and clearing the robot workspace.

## 5. Record waypoints

Enable hand guiding using the approved Doosan procedure.

```bash
mkdir -p ~/ros2_ws/waypoints
cd ~/ros2_ws

ros2 run vla_data_collector waypoint_recorder \
  --ros-args \
  -p output_path:=waypoints/cube_pick.json
```

Record:

1. home
2. approach
3. pre_grasp
4. grasp
5. lift

## 6. Replay and collect one episode

Before running:

- switch out of hand-guiding mode
- clear the workspace
- use low robot speed
- verify the emergency stop
- place the cube in the taught location

```bash
cd ~/ros2_ws

ros2 run vla_data_collector dataset_recorder \
  --ros-args \
  --params-file src/vla_data_collector/config/collector.yaml \
  -p episode_id:=episode_000001
```

## 7. Generate OpenVLA delta actions

```bash
ros2 run vla_data_collector make_actions \
  raw_dataset/episode_000001
```

## 8. Validate

```bash
ros2 run vla_data_collector validate_episode \
  raw_dataset/episode_000001
```

## Output

```text
raw_dataset/episode_000001/
├── metadata.json
├── steps.jsonl
├── steps_with_actions.jsonl
└── images/
    ├── 000000.jpg
    ├── 000001.jpg
    └── ...
```

## Important limitation

The replay trajectory is tied to the taught cube position. To train a vision
policy that generalizes, collect many successful episodes with different cube
positions, lighting, backgrounds, and approach trajectories. A fixed waypoint
replay alone does not provide sufficient action diversity for robust OpenVLA
fine-tuning.
