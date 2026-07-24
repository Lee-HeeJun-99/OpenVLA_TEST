# RLDS Conversion Pipeline

This directory contains the conversion path from collected Doosan A0509 raw
episodes to a TensorFlow Datasets / RLDS dataset for OpenVLA fine-tuning.

## Directory Layout

```text
rlds/
  README.md
  requirements.txt
  doosan_a0509/
    __init__.py
    doosan_a0509_dataset_builder.py
    README.md
    CITATIONS.bib
  openvla_registration/
    doosan_a0509.py
```

## 1. Validate Raw Episodes

Run this after collecting episodes and generating actions:

```bash
cd ~/robot_ws
source install/setup.bash

python3 -m vla_data_collector.check_rlds_source raw_dataset
```

Expected output includes:

```text
Convertible episodes : <N>
Total steps          : <M>
Dataset units        : position=meter, rotation=radian
Action shape         : (7,)
```

## 2. Install RLDS Conversion Dependencies

Use a separate Python/conda environment from the ROS runtime when possible.

```bash
pip install -r ~/robot_ws/src/OpenVLA_TEST/rlds/requirements.txt
```

If the conversion environment is not using the ROS workspace Python path, add
the collector package manually:

```bash
export PYTHONPATH=~/robot_ws/src/OpenVLA_TEST/vla_data_collector:$PYTHONPATH
```

## 3. Build TFDS/RLDS

```bash
cd ~/robot_ws/src/OpenVLA_TEST/rlds/doosan_a0509

python build_dataset.py \
  --raw-dataset-dir ~/robot_ws/raw_dataset \
  --data-dir ~/tensorflow_datasets \
  --overwrite
```

The generated dataset should appear at:

```text
~/tensorflow_datasets/doosan_a0509/1.0.0/
```

The `tfds build` CLI can also be used, but recent TFDS versions import
`apache_beam` during CLI startup. The local `build_dataset.py` path avoids that
extra CLI dependency for this generator-based dataset.

## 4. Register Dataset In OpenVLA

Use `openvla_registration/doosan_a0509.py` as the source of snippets for:

```text
prismatic/vla/datasets/rlds/oxe/configs.py
prismatic/vla/datasets/rlds/oxe/transforms.py
prismatic/vla/datasets/rlds/oxe/mixtures.py
```

Then use `doosan_a0509_single` as the dataset mixture name for fine-tuning.

## Data Contract

RLDS step fields:

```text
observation/image          uint8[H, W, 3], RGB
observation/EEF_state      float32[6], [x, y, z, roll, pitch, yaw]
observation/gripper_state  float32[1], 0=closed, 1=open
observation/state          float32[7], EEF_state + gripper_state
action                     float32[7], delta EEF pose + gripper target
language_instruction       string
language_embedding         float32[512], zero placeholder
```

The language embedding is included for RLDS schema compatibility. OpenVLA uses
the text `language_instruction` for training.
