# Doosan A0509 RLDS Dataset

This TensorFlow Datasets builder converts recorded Doosan A0509 episodes into
an RLDS-compatible dataset for OpenVLA fine-tuning.

## Raw Input

Expected input directory:

```text
raw_dataset/
  episode_000001/
    metadata.json
    steps_with_actions.jsonl
    images/
      000000.jpg
      000001.jpg
      ...
```

Each step is converted as:

```text
image                  -> steps/observation/image
tcp_pose[6]            -> steps/observation/EEF_state
gripper                -> steps/observation/gripper_state
tcp_pose + gripper     -> steps/observation/state
action[7]              -> steps/action
metadata.instruction   -> steps/language_instruction
```

Units in the RLDS dataset:

```text
position: meter
rotation: radian
action: [dx, dy, dz, droll, dpitch, dyaw, gripper]
gripper: 0=closed, 1=open
```

## Build

Run from this directory:

```bash
cd ~/robot_ws/src/OpenVLA_TEST/rlds/doosan_a0509
export PYTHONPATH=~/robot_ws/src/OpenVLA_TEST/vla_data_collector:$PYTHONPATH

python build_dataset.py \
  --raw-dataset-dir ~/robot_ws/raw_dataset \
  --data-dir ~/tensorflow_datasets \
  --overwrite
```

Output:

```text
~/tensorflow_datasets/doosan_a0509/1.0.0/
```

Optional validation split:

```bash
python build_dataset.py \
  --raw-dataset-dir ~/robot_ws/raw_dataset \
  --data-dir ~/tensorflow_datasets \
  --val-ratio 0.1 \
  --overwrite
```

By default, only successful episodes are converted. To include failed episodes:

```bash
python build_dataset.py \
  --raw-dataset-dir ~/robot_ws/raw_dataset \
  --data-dir ~/tensorflow_datasets \
  --include-failures \
  --overwrite
```

Keep failures excluded for the first OpenVLA fine-tuning runs unless the
training objective explicitly uses failed demonstrations.
