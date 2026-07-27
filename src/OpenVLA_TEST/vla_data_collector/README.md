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

## 6. Hand-guide and collect one episode

Use this mode for OpenVLA fine-tuning. It does not replay waypoints. You guide
the robot by hand while the recorder samples camera frames and TCP pose at the
same fixed rate.

Before running:

- enable hand guiding using the approved Doosan procedure
- verify the emergency stop
- keep the cube visible in the ZED image
- move continuously; avoid pauses during recording
- use the keyboard controls below for the gripper

```bash
cd ~/ros2_ws

ros2 run vla_data_collector handguide_recorder \
  --ros-args \
  --params-file src/vla_data_collector/config/collector.yaml
```

During recording:

```text
o : open gripper
c : close gripper
s : stop and mark success
f : stop and mark failure
q : abort recording, keep files marked failure
```

The recorder defaults to:

- 10 Hz sampling
- latest ZED RGB image + current Doosan TCP pose
- skip near-stationary samples smaller than 1 mm and 0.005 rad
- automatically write `steps_with_actions.jsonl` on stop

For this task, a good episode should include:

1. approach the cube from a visible starting pose
2. align the gripper around the cube
3. press `c` to close the gripper
4. lift the cube clearly off the table
5. press `s` only after a successful lift

Vary the cube position and approach slightly across episodes.

## 7. Replay and collect one episode

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

Do not use this replay mode for the main fine-tuning dataset unless you are
debugging the pipeline.

## 8. Generate OpenVLA delta actions

```bash
ros2 run vla_data_collector make_actions \
  raw_dataset/episode_000001
```

The hand-guide recorder runs this automatically when
`generate_actions_on_stop: true`.

## 9. Validate

```bash
ros2 run vla_data_collector validate_episode \
  raw_dataset/episode_000001
```

## 10. Resample and Convert to RLDS

If episodes were collected at mixed rates, first create a 10 Hz raw dataset
copy. The original raw dataset is left untouched; the output uses symlinks to
the original image folders.

```bash
python3 -m vla_data_collector.resample_dataset \
  ~/robot_ws/raw_dataset \
  ~/robot_ws/raw_dataset_resampled_10hz \
  --target-hz 10 \
  --max-gap-sec 0.5
```

After collecting episodes, validate the RLDS source data:

```bash
python3 -m vla_data_collector.check_rlds_source \
  ~/robot_ws/raw_dataset_resampled_10hz
```

Then build the TFDS/RLDS dataset:

```bash
cd ~/robot_ws/src/OpenVLA_TEST/rlds/doosan_a0509

python build_dataset.py \
  --raw-dataset-dir ~/robot_ws/raw_dataset_resampled_10hz \
  --data-dir ~/tensorflow_datasets \
  --overwrite
```

See `../rlds/README.md` for the OpenVLA registration snippets.

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
