# OpenVLA Doosan A0509 Problem Record

작성일: 2026-07-28  
작업 위치: `/home/ubuntu/robot_ws/src`  
목적: 학습된 OpenVLA 모델로 Doosan A0509를 구동할 때 로봇이 제대로 움직이지 않는 원인이 로봇/런타임/좌표계 문제인지, 학습/모델 출력 문제인지 객관적으로 판단하기 위한 파일 구조, 데이터 포맷, 현재 관측값, 평가 지표를 기록한다.

## 1. 현재 결론 요약

현재까지의 객관 증거는 **로봇 하드웨어나 Doosan motion service 고장보다는 모델 출력 collapse 또는 학습/전처리/데이터 분포 문제 가능성이 더 높다**.

근거:

- `vla_debug_dryrun` rosbag은 `doosan_bridge.dry_run=true`로 기록되었다. 이 모드에서는 실제 `move_line`과 gripper digital output을 호출하지 않는다.
- dry-run bag에서 `/vla/raw_action` 12개가 모두 완전히 같은 값이었다.
- 같은 구간에서 `/vla/image_rgb`는 4083개 기록되었고, 분석 스크립트가 샘플링한 이미지 hash 100개는 모두 달랐다.
- `/doosan/current_pose`는 거의 고정되어 있었고 dry-run 상태였으므로, 로봇 motion 결과가 모델 입력을 교란한 상황도 아니었다.
- 따라서 “카메라 이미지가 멈춰서 같은 action이 나왔다”는 가설은 약하다.

반복된 action:

```text
[3.239264112769348e-05,
 4.9954392563764105e-05,
 -0.00014253266217808397,
 0.001078210179419531,
 0.00034518010059698956,
 0.0012115914716438886,
 0.996078431372549]
```

해석:

- 앞 3개는 translation delta이고 데이터셋 기준 단위는 meter이다.
- 다음 3개는 rotation delta이고 데이터셋 기준 단위는 radian이다.
- 마지막 값은 gripper target이고 `0=close`, `1=open`이다.
- gripper가 거의 항상 `0.996`으로 open 쪽에 붙어 있다.
- 현재 runtime의 `translation_scale_to_mm=30000.0`을 적용하면 이 action은 대략 `[0.972 mm, 1.499 mm, -4.276 mm]`가 되고, debug config에서는 `max_translation_step_mm=3.0` 때문에 Z가 `-3.0 mm`로 clamp된다.
- 실제 motion config는 안전을 위해 `max_translation_step_mm=1.0`, `repeated_action_limit=2`로 낮춰 두었다.

## 2. Workspace 구조

핵심 디렉터리:

```text
/home/ubuntu/robot_ws/src/
  OpenVLA_TEST/                # Doosan/ZED 데이터 수집, raw dataset 검증, RLDS/TFDS 변환 코드
  openvla/                     # OpenVLA 원본/수정 repo, LoRA fine-tuning, HF checkpoint
  openvla_doosan_runtime/      # ROS2 runtime: camera -> model -> action -> Doosan bridge
  tensorflow_datasets/         # TFDS/RLDS 산출물 일부
  doosan-robot2/               # Doosan ROS2 driver/vendor dependency
  zed-ros2-wrapper/            # ZED ROS2 wrapper/vendor dependency
  build/                       # colcon build 산출물
  install/                     # colcon install 산출물
  log/                         # colcon build log
```

Workspace 외부 관련 디렉터리:

```text
/home/ubuntu/robot_ws/raw_dataset/                 # 원본 수집 episode
/home/ubuntu/robot_ws/raw_dataset_resampled_10hz/  # 10 Hz resampled episode
/home/ubuntu/robot_ws/tensorflow_datasets/         # 다른 TFDS 출력 root
/home/ubuntu/.ros/log/                             # ROS runtime log
```

파일 수 기준:

- `openvla_doosan_runtime`: 28 files
- `OpenVLA_TEST`: 66 files
- `openvla`: 272 files
- `doosan-robot2` + `zed-ros2-wrapper`: 1441 files

이 문서는 직접 수정/운영하는 OpenVLA-Doosan 관련 파일은 파일 단위로 기록하고, `doosan-robot2`, `zed-ros2-wrapper`, `build`, `install`, `log`, `__pycache__`, safetensors weight shard, mesh 파일 등 외부/생성/대용량 산출물은 역할 단위로 묶어 기록한다.

## 3. 전체 파이프라인

```text
ZED RGB topic
  /zed/zed_node/rgb/color/rect/image
    -> camera_adapter
  /vla/image_rgb
    -> openvla_inference
  /vla/raw_action: Float64MultiArray[7]
    -> action_adapter
  /vla/target_pose: Float64MultiArray[6]
  /vla/gripper_open: Bool
    -> doosan_bridge
  /dsr01/motion/move_line
  /dsr01/io/set_tool_digital_output
```

Episode 제어:

```text
/vla/set_episode std_srvs/srv/SetBool
  true  -> /vla/enable true, /vla/instruction publish
  false -> /vla/enable false

/vla/stop std_srvs/srv/Trigger
  -> /vla/emergency_stop true
  -> /dsr01/motion/move_stop
```

한 step 흐름:

1. `episode_manager`가 `/vla/enable=true` publish.
2. `doosan_bridge`가 현재 pose를 `/doosan/current_pose`로 publish.
3. `doosan_bridge`가 enabled + not busy + pose 있음이면 `/vla/robot_ready=true`.
4. `openvla_inference`가 최신 `/vla/image_rgb`와 instruction으로 action 예측.
5. `openvla_inference`가 `/vla/raw_action` publish.
6. `action_adapter`가 raw action을 target pose로 변환.
7. `doosan_bridge`가 target pose를 실제 `move_line`으로 실행.
8. 완료 시 `/vla/step_done=true`.
9. 다음 inference는 `/vla/robot_ready=true`가 다시 나와야 진행.

## 4. 핵심 데이터 계약

### 4.1 Raw dataset step JSONL

파일:

```text
/home/ubuntu/robot_ws/raw_dataset_resampled_10hz/episode_*/steps_with_actions.jsonl
```

각 line은 JSON object 한 개이다.

예시:

```json
{
  "step_index": 0,
  "timestamp": 0.0,
  "source_timestamp": 3.2344743389985524,
  "source_step_index": 1,
  "resample_run_index": 1,
  "image_timestamp": 1785126071.3390844,
  "image_age_sec": 0.010466462990734726,
  "image": "images/000001.jpg",
  "tcp_pose": [-0.00826671600341797, 0.30026638793945315, 0.63117041015625, 0.2534647921411511, 3.1065478504893314, 1.8214293464594529],
  "gripper": 1,
  "action": [3.0163464543875307e-06, -3.9089733036234975e-05, 3.293993722763844e-05, 0.0030092468950897455, -2.39985511143459e-05, 0.003015708178281784, 1.0]
}
```

필드:

| 필드 | 타입 | 단위/의미 |
|---|---:|---|
| `step_index` | int | episode 내부 step index |
| `timestamp` | float | resampled timeline seconds |
| `source_timestamp` | float | 원본 기록 timestamp |
| `source_step_index` | int | 원본 step index |
| `resample_run_index` | int | timestamp gap 기준으로 분리된 run index |
| `image_timestamp` | float | ROS image header time |
| `image_age_sec` | float | 기록 시점에서 이미지 age |
| `image` | string | episode 폴더 기준 상대 image path |
| `tcp_pose` | float[6] | `[x,y,z,rx,ry,rz]`, meter/radian |
| `gripper` | int | `0=closed`, `1=open` |
| `action` | float[7] | `[dx,dy,dz,dRx,dRy,dRz,gripper]`, meter/radian + absolute gripper |

### 4.2 Raw episode metadata

파일:

```text
/home/ubuntu/robot_ws/raw_dataset_resampled_10hz/episode_*/metadata.json
```

주요 필드:

| 필드 | 의미 |
|---|---|
| `episode_id` | episode folder id |
| `instruction` | OpenVLA language instruction. 현재 전부 `pick up the cube` |
| `task` | task id. 현재 `cube_pick` |
| `robot` | `Doosan A0509` |
| `camera` | `Stereolabs ZED 2i` |
| `camera_mount` | `eye_in_hand_vertical` |
| `coordinate_frame` | `base` |
| `position_unit` | dataset pose/action position unit. `meter` |
| `rotation_unit` | dataset pose/action rotation unit. `radian` |
| `robot_control_position_unit` | Doosan motion command position unit. `millimeter` |
| `robot_control_rotation_unit` | Doosan motion command rotation unit. `degree` |
| `pose_source` | `service` or `topic` |
| `record_frequency_hz` | target recording/resample frequency |
| `num_steps` | step count |
| `success` | successful demo 여부 |
| `resampled` | 10 Hz resampling 여부 |

### 4.3 TFDS/RLDS dataset structure

파일:

```text
/home/ubuntu/robot_ws/src/tensorflow_datasets/doosan_a0509/1.0.0/features.json
/home/ubuntu/robot_ws/src/tensorflow_datasets/doosan_a0509/1.0.0/dataset_info.json
```

Feature schema:

```text
episode_metadata:
  file_path: text
  episode_id: text
  success: bool
  num_steps: int32

steps:
  observation:
    image: uint8 image, shape=(H, W, 3), jpeg
    EEF_state: float32[6], [x,y,z,roll,pitch,yaw], meter/radian
    gripper_state: float32[1], 0=closed, 1=open
    state: float32[7], EEF_state + gripper_state
  action: float32[7], [dx,dy,dz,droll,dpitch,dyaw,gripper]
  discount: float32, fixed 1.0
  reward: float32, final successful step만 1.0
  is_first: bool
  is_last: bool
  is_terminal: bool
  language_instruction: text
  language_embedding: float32[512], zero placeholder
```

현재 확인된 TFDS 산출물:

```text
/home/ubuntu/robot_ws/src/tensorflow_datasets/doosan_a0509/1.0.0/
  dataset_info.json
  features.json
  dataset_statistics_229351d8dc607c47fd18b77f8031f4c17133c6a6510c65ea225497961f8d696c.json
  doosan_a0509-train.tfrecord-00000-of-00002
  doosan_a0509-train.tfrecord-00001-of-00002
```

주의:

- `/home/ubuntu/robot_ws/tensorflow_datasets/doosan_a0509/1.0.0/`에도 16 shard TFDS 출력이 있다.
- 학습에 어떤 `data_root_dir`를 썼는지 재현하려면 training command 또는 wandb config가 필요하다.
- 현재 모델의 `dataset_statistics.json`은 `num_transitions=9255`, `num_trajectories=59`로, `/home/ubuntu/robot_ws/raw_dataset_resampled_10hz`의 step 수와 일치한다.

### 4.4 Model checkpoint structure

현재 runtime이 가리키는 모델:

```text
/home/ubuntu/robot_ws/src/openvla/runs/47a0ec7fc4ec123775a391911046cf33cf9ed83f+doosan_a0509+b8+lr-0.0005+lora-r32+dropout-0.0--handguide_10hz_resampled--image_aug
```

필수 파일:

| 파일 | 역할 |
|---|---|
| `config.json` | HF/OpenVLA model config |
| `generation_config.json` | HF generation config |
| `model.safetensors.index.json` | shard index |
| `model-00001-of-00004.safetensors` ... `model-00004-of-00004.safetensors` | 병합된 model weight shard |
| `tokenizer.json` | HF tokenizer |
| `tokenizer.model` | sentencepiece tokenizer model |
| `tokenizer_config.json` | tokenizer config |
| `special_tokens_map.json` | special token mapping |
| `added_tokens.json` | added token metadata |
| `preprocessor_config.json` | image processor config |
| `processor_config.json` | processor wrapper config |
| `configuration_prismatic.py` | trust_remote_code config class |
| `modeling_prismatic.py` | trust_remote_code model class |
| `processing_prismatic.py` | trust_remote_code processor class |
| `dataset_statistics.json` | action unnormalization statistics |

`dataset_statistics.json` 중 action stats:

```text
mean = [-0.00006018, 0.00071357, -0.00043344, -0.00099963, -0.00953748, -0.00158374, 0.75245816]
std  = [0.00248056, 0.00308142, 0.00594148, 0.15441146, 0.03343018, 0.15440337, 0.43158486]
min  = [-0.01285561, -0.01871649, -0.01956766, -3.13224483, -0.84882247, -3.12820315, 0.0]
max  = [0.01032591, 0.01542921, 0.02625024, 3.09569860, 0.80992520, 3.09615874, 1.0]
q01  = [-0.00804276, -0.00880261, -0.01134807, -0.13079219, -0.09373100, -0.12045910, 0.0]
q99  = [0.00661321, 0.00823440, 0.01766118, 0.12299613, 0.02532322, 0.11728823, 1.0]
mask = [true, true, true, true, true, true, false]
num_transitions = 9255
num_trajectories = 59
```

해석:

- action normalization은 앞 6개 action에 적용된다.
- gripper는 `mask=false`이므로 normalized/unnormalized 연속값이 아니라 token 값 그대로 또는 별도 처리에 가깝다.
- dataset gripper 평균이 약 `0.752`라 open class가 많다.

### 4.5 ROS message contract

| Topic | Type | Publisher | Subscriber | 의미 |
|---|---|---|---|---|
| `/zed/zed_node/rgb/color/rect/image` | `sensor_msgs/msg/Image` | ZED wrapper | `camera_adapter` | 원본 RGB/BGR image |
| `/vla/image_rgb` | `sensor_msgs/msg/Image` | `camera_adapter` | `openvla_inference` | OpenVLA 입력 image, `rgb8` |
| `/vla/instruction` | `std_msgs/msg/String` | `episode_manager` 또는 사용자 | `openvla_inference` | 자연어 instruction |
| `/vla/enable` | `std_msgs/msg/Bool` | `episode_manager` | inference/action/bridge | episode 실행 여부 |
| `/vla/robot_ready` | `std_msgs/msg/Bool` | `doosan_bridge` | `openvla_inference` | 다음 action 요청 가능 여부 |
| `/vla/raw_action` | `std_msgs/msg/Float64MultiArray` | `openvla_inference` | `action_adapter` | model action, length 7 |
| `/doosan/current_pose` | `std_msgs/msg/Float64MultiArray` | `doosan_bridge` | `action_adapter` | Doosan current pose, length 6, mm/deg |
| `/vla/target_pose` | `std_msgs/msg/Float64MultiArray` | `action_adapter` | `doosan_bridge` | Doosan absolute target pose, length 6, mm/deg |
| `/vla/gripper_open` | `std_msgs/msg/Bool` | `action_adapter` | `doosan_bridge` | true=open, false=close |
| `/vla/action_valid` | `std_msgs/msg/Bool` | `action_adapter` | `episode_manager` | action reject 여부 |
| `/vla/action_status` | `std_msgs/msg/String` | `action_adapter` | logs/bag | action conversion 상태 |
| `/vla/model_status` | `std_msgs/msg/String` | `openvla_inference` | logs/bag | `inference_ok` or error |
| `/vla/step_done` | `std_msgs/msg/Bool` | `doosan_bridge` | `episode_manager` | robot step 완료 여부 |
| `/doosan/status` | `std_msgs/msg/String` | `doosan_bridge` | logs/bag | move/gripper/dry-run 상태 |
| `/vla/emergency_stop` | `std_msgs/msg/Bool` | `episode_manager` | `doosan_bridge` | emergency stop |

Doosan services:

```text
/dsr01/aux_control/get_current_posx
/dsr01/motion/move_line
/dsr01/motion/move_stop
/dsr01/io/set_tool_digital_output
```

## 5. 파일별 역할

### 5.1 `openvla_doosan_runtime/`

이 패키지는 ROS2 runtime이다. 학습은 하지 않고, 저장된 HF checkpoint를 load해서 robot command로 변환한다.

| 파일 | 역할 | 문제 판단에서 보는 포인트 |
|---|---|---|
| `README.md` | 실행 순서, safety check, dry-run bag 절차 문서 | 실제 재현 절차 기준 |
| `package.xml` | ROS2 package manifest | dependency 선언 확인 |
| `setup.py` | console script entry point 등록 | launch가 실행하는 node 이름 확인 |
| `setup.cfg` | package install config | ROS2 Python package metadata |
| `resource/openvla_doosan_runtime` | ament package resource marker | package discovery |
| `launch/runtime.launch.py` | 5개 node launch: camera, inference, action, bridge, episode | `config`, `inference_python` 인자 확인 |
| `config/runtime.yaml` | 실제 motion용 runtime 파라미터 | 모델 경로, 단위 변환, safety clamp |
| `config/runtime_debug_bag.yaml` | dry-run bag 진단용 config | `dry_run=true`, `max_steps=12`, 실제 robot motion 없음 |
| `openvla_doosan_runtime/__init__.py` | Python package marker | 기능 없음 |
| `openvla_doosan_runtime/common.py` | `ACTION_DIM=7`, `POSE_DIM=6`, array validation | NaN/shape 오류 차단 |
| `openvla_doosan_runtime/camera_adapter_node.py` | ZED image를 OpenVLA용 `/vla/image_rgb`로 변환 | encoding, resize, crop, flip 문제 확인 |
| `openvla_doosan_runtime/openvla_inference_node.py` | HF OpenVLA load, image+instruction -> action | model path, unnorm_key, image_hash, repeat count |
| `openvla_doosan_runtime/action_adapter_node.py` | raw action -> Doosan absolute target pose | scale, axis sign, clamp, workspace, repeated action reject |
| `openvla_doosan_runtime/doosan_bridge_node.py` | Doosan service bridge, pose polling, move_line, gripper | robot service 성공/실패, dry_run 분리 |
| `openvla_doosan_runtime/episode_manager_node.py` | episode start/stop, max_steps/max_duration watchdog | enable 흐름, episode 실패/정지 판정 |
| `scripts/analyze_vla_bag.py` | rosbag 분석 도구 | raw action 반복, image hash, pose range 객관 지표 산출 |
| `bags/vla_action_repeat_20260728_01/*` | 2026-07-28 기록 bag | episode off라 raw action 0개 |
| `bags/vla_debug_dryrun/*` | 2026-07-28 dry-run 기록 bag | raw action 12개 모두 동일 |

생성 파일:

- `__pycache__/*.pyc`: Python bytecode cache. 기능 판단 대상 아님.
- `bags/*.db3`, `*.db3.zstd`, `metadata.yaml`: rosbag 산출물. 분석/증거 자료.

#### `runtime.yaml` 주요 파라미터

```yaml
openvla_inference:
  model_path: /home/ubuntu/robot_ws/src/openvla/runs/47a0...--handguide_10hz_resampled--image_aug
  unnorm_key: doosan_a0509
  device: cuda:0
  center_crop_scale: 0.9
  do_sample: false

action_adapter:
  translation_scale_to_mm: 30000.0
  rotation_scale_to_deg: 57.29577951308232
  max_translation_step_mm: 1.0
  apply_rotation: false
  reject_repeated_actions: true
  repeated_action_limit: 2
  axis_sign: [1.0, 1.0, 1.0, 1.0, 1.0, 1.0]

doosan_bridge:
  dry_run: false
```

중요 주의:

- 데이터셋 정의상 position은 meter이고 Doosan command는 millimeter이다.
- 수학적으로 meter -> millimeter 변환은 `1000.0`이다.
- 현재 `translation_scale_to_mm=30000.0`은 30배 gain이다.
- 이 값은 action 반복 문제의 원인은 아니지만, 작은 모델 출력이 반복될 때 target step이 쉽게 clamp되어 누적 직선 이동을 만들 수 있다.
- 따라서 원인 분리 시 `translation_scale_to_mm=1000.0` 실험도 별도 지표로 수행해야 한다.

#### `runtime_debug_bag.yaml` 주요 차이

```yaml
doosan_bridge:
  dry_run: true

episode_manager:
  max_steps: 12
  max_duration_sec: 20.0

action_adapter:
  repeated_action_limit: 20
  max_translation_step_mm: 3.0
```

의미:

- robot service는 pose polling만 사용한다.
- `move_line`과 gripper output은 호출하지 않는다.
- 모델이 같은 action을 내는지 로봇 motion 없이 확인한다.

### 5.2 `OpenVLA_TEST/vla_data_collector/`

이 패키지는 데이터 수집과 raw dataset 생성용 ROS2/Python package이다.

| 파일 | 역할 | 데이터/포맷 |
|---|---|---|
| `README.md` | 수집 package 사용 설명 | waypoint/record/build 절차 |
| `package.xml` | ROS2 package manifest | dependency 선언 |
| `setup.py` | console script 등록 | recorder CLI/node entry |
| `setup.cfg` | package install config | metadata |
| `resource/vla_data_collector` | ament marker | package discovery |
| `config/collector.yaml` | waypoint/dataset/handguide recorder 설정 | dataset_root, image_topic, gripper DO index |
| `vla_data_collector/__init__.py` | package marker | 기능 없음 |
| `vla_data_collector/doosan_interface.py` | Doosan service wrapper | `get_current_posx`, `move_line`, gripper DO |
| `vla_data_collector/gripper_interface.py` | digital gripper command helper | open/close pulse |
| `vla_data_collector/waypoint_recorder.py` | 현재 pose를 waypoint JSON으로 저장 | `waypoints/cube_pick.json` |
| `vla_data_collector/dataset_recorder.py` | waypoint replay 중 image/pose/gripper 기록 | `raw_dataset/episode_*/steps.jsonl` |
| `vla_data_collector/handguide_recorder.py` | hand-guided demo를 10 Hz 내외로 기록 | continuous raw dataset |
| `vla_data_collector/make_actions.py` | 인접 step pose 차이로 action 생성 | `steps_with_actions.jsonl` |
| `vla_data_collector/resample_dataset.py` | raw episode를 fixed Hz로 resample | `raw_dataset_resampled_10hz` |
| `vla_data_collector/rlds_utils.py` | raw episode 읽기/검증/분할/helper | TFDS builder에서 재사용 |
| `vla_data_collector/check_rlds_source.py` | RLDS 변환 전 source summary | episode/step/action shape 확인 |
| `vla_data_collector/validate_episode.py` | episode 단위 raw file 검증 | image, timestamp, pose/action shape |

핵심 변환:

- `dataset_recorder.py`, `handguide_recorder.py`
  - Doosan service pose: mm/degree
  - dataset pose: meter/radian
  - 변환:
    - position `* 0.001`
    - rotation `* pi / 180`
- `make_actions.py`
  - `delta_position = next_pose[:3] - current_pose[:3]`
  - `delta_rotation = wrap_to_pi(next_pose[3:] - current_pose[3:])`
  - action = `[delta_position, delta_rotation, following_gripper]`
- 따라서 학습 action은 기본적으로 **dataset 좌표계의 delta action**이다.

### 5.3 `OpenVLA_TEST/rlds/`

이 디렉터리는 raw dataset을 OpenVLA가 읽는 TFDS/RLDS 형식으로 변환한다.

| 파일 | 역할 |
|---|---|
| `README.md` | RLDS 변환 설명 |
| `requirements.txt` | 변환용 Python dependency |
| `doosan_a0509/README.md` | Doosan RLDS builder 설명 |
| `doosan_a0509/CITATIONS.bib` | dataset citation |
| `doosan_a0509/__init__.py` | builder package marker |
| `doosan_a0509/build_dataset.py` | raw dataset -> TFDS 실행 script |
| `doosan_a0509/doosan_a0509_dataset_builder.py` | TFDS `GeneratorBasedBuilder`; feature schema 정의 |
| `openvla_registration/doosan_a0509.py` | OpenVLA repo에 반영할 config/transform snippet |

OpenVLA 등록 상태:

- `openvla/prismatic/vla/datasets/rlds/oxe/configs.py`에 `doosan_a0509` 추가됨.
- `state_encoding = StateEncoding.POS_EULER`
- `action_encoding = ActionEncoding.EEF_POS`
- image key: `image`
- state keys: `EEF_state`, `gripper_state`
- `openvla/prismatic/vla/datasets/rlds/oxe/transforms.py`에 `doosan_a0509_dataset_transform`이 등록되어 있고 현재 transform은 identity이다.

### 5.4 `openvla/`

OpenVLA 원본 repo와 fine-tuning code, local checkpoint를 포함한다.

문제 분석과 직접 관련 있는 파일:

| 파일/디렉터리 | 역할 |
|---|---|
| `README.md` | OpenVLA 공식 사용/학습 설명 |
| `pyproject.toml` | package/dependency metadata |
| `requirements-min.txt` | 최소 dependency |
| `vla-scripts/finetune.py` | LoRA fine-tuning script |
| `vla-scripts/train.py` | full training script |
| `vla-scripts/deploy.py` | REST inference server |
| `scripts/generate.py` | generation playground |
| `scripts/preprocess.py` | preprocessing utility |
| `scripts/pretrain.py` | pretraining script |
| `test_server.py` | REST server `/act` 테스트 client |
| `prismatic/vla/action_tokenizer.py` | continuous action <-> token discretization |
| `prismatic/vla/datasets/` | RLDS loading/batch transform |
| `prismatic/vla/datasets/rlds/oxe/configs.py` | dataset observation/action key mapping |
| `prismatic/vla/datasets/rlds/oxe/transforms.py` | dataset transform registry |
| `prismatic/extern/hf/modeling_prismatic.py` | HF OpenVLA model implementation, `predict_action` |
| `runs/<checkpoint>/` | 병합된 HF-format fine-tuned model |
| `adapter-tmp/<run>/` | LoRA adapter temporary weights |
| `wandb/` | offline run logs |

`finetune.py` 핵심 설정:

```text
vla_path: openvla/openvla-7b
dataset_name: doosan_a0509
run_root_dir: runs
adapter_tmp_dir: adapter-tmp
batch_size: 16 default, 실제 run name은 b8
max_steps: 200000 default
save_steps: 5000 default
learning_rate: 5e-4
image_aug: true
use_lora: true
lora_rank: 32
lora_dropout: 0.0
do_sample during inference: false
```

학습 산출물:

```text
openvla/runs/47a0...+doosan_a0509+b8+lr-0.0005+lora-r32+dropout-0.0--handguide_10hz_resampled--image_aug/
```

임시 adapter:

```text
openvla/adapter-tmp/openvla-7b+doosan_a0509+b16+lr-0.0005+lora-r32+dropout-0.0--image_aug/
openvla/adapter-tmp/openvla-7b+doosan_a0509+b8+lr-0.0005+lora-r32+dropout-0.0/
```

주의:

- 현재 runtime 모델은 `runs/47a0...b8...handguide_10hz_resampled--image_aug`이다.
- 이전에 존재하던 `b16` 경로는 모델 파일이 없거나 경로가 달라 runtime load 오류를 만들었다.
- 모델 directory에 safetensors shard가 모두 있어야 `AutoModelForVision2Seq.from_pretrained`가 local path로 load된다.

### 5.5 `tensorflow_datasets/`

`/home/ubuntu/robot_ws/src/tensorflow_datasets/doosan_a0509/1.0.0`:

| 파일 | 역할 |
|---|---|
| `features.json` | TFDS feature schema |
| `dataset_info.json` | TFDS dataset metadata |
| `dataset_statistics_*.json` | dataset statistics cache |
| `doosan_a0509-train.tfrecord-00000-of-00002` | train shard |
| `doosan_a0509-train.tfrecord-00001-of-00002` | train shard |

`/home/ubuntu/robot_ws/tensorflow_datasets/doosan_a0509/1.0.0`:

- 16개 train shard가 있는 별도 TFDS root.
- 파일 크기 총량이 훨씬 크다.
- 학습 재현 시 `--data_root_dir`가 어느 root였는지 반드시 확인해야 한다.

### 5.6 `raw_dataset`와 `raw_dataset_resampled_10hz`

현재 데이터 수:

```text
/home/ubuntu/robot_ws/raw_dataset
  episodes: 59
  successes: 59
  steps_with_actions: 7986
  jpg_images: 7986
  instruction set: {"pick up the cube": 59}

/home/ubuntu/robot_ws/raw_dataset_resampled_10hz
  episodes: 59
  successes: 59
  steps_with_actions: 9255
  jpg_images: 7986
  instruction set: {"pick up the cube": 59}
```

데이터 분포:

```text
actions: 9255
xyz mean: [-0.00006018, 0.00071357, -0.00043344]
xyz std:  [0.00248057, 0.00308143, 0.00594148]
xyz q01:  [-0.00804276, -0.00880261, -0.01134807]
xyz q50:  [0.00001717, 0.00047192, -0.00097854]
xyz q99:  [0.00661321, 0.00823440, 0.01766118]
norm <= 0.001m: 1346 / 9255 = 14.54%
norm <= 0.003m: 3135 / 9255 = 33.87%
gripper open count: 6964
gripper close count: 2291
gripper open ratio: 75.25%
```

의미:

- 데이터에 작은 motion이 많지만 전부 정지 action은 아니다.
- gripper open class가 많아 모델이 open에 치우칠 가능성이 있다.
- instruction이 하나뿐이라 언어 조건으로 상황을 세분화하지 못한다.
- 성공 episode만 포함되어 실패/복구 다양성이 없다.

### 5.7 `doosan-robot2/`

Doosan ROS2 driver/vendor dependency이다. 직접 수정 대상이 아니라 runtime에서 service API를 사용한다.

중요 하위 구성:

| 디렉터리 | 역할 |
|---|---|
| `dsr_bringup2/` | real/sim bringup launch |
| `dsr_msgs2/` | Doosan service/message/action 정의 |
| `dsr_controller2/` | controller bridge |
| `dsr_hardware2/` | hardware interface |
| `dsr_description2/` | URDF, meshes, robot descriptions |
| `dsr_moveit2/` | MoveIt configs |
| `dsr_example2/` | examples |
| `dsr_gazebo2/`, `dsr_mujoco/` | simulation |

문제 판단에서 확인할 항목:

```bash
ros2 service list | grep -E "current_posx|move_line|move_stop|set_tool_digital_output"
ros2 interface show dsr_msgs2/srv/MoveLine
ros2 interface show dsr_msgs2/srv/GetCurrentPosx
ros2 interface show dsr_msgs2/srv/SetToolDigitalOutput
ros2 interface show dsr_msgs2/srv/MoveStop
```

robot-side failure로 볼 수 있는 근거:

- `/dsr01/aux_control/get_current_posx`가 응답하지 않음
- `/doosan/current_pose`가 NaN, shape 오류, 또는 실제 motion과 불일치
- `/dsr01/motion/move_line`이 실패 응답
- 같은 target pose를 수동으로 보냈을 때 로봇이 일관되게 다른 방향으로 움직임
- gripper DO wiring이 metadata와 다름

### 5.8 `zed-ros2-wrapper/`

ZED camera ROS2 wrapper/vendor dependency이다.

문제 판단에서 확인할 항목:

```bash
ros2 topic hz /zed/zed_node/rgb/color/rect/image
ros2 topic hz /vla/image_rgb
ros2 topic echo /zed/zed_node/status/health
```

camera-side failure로 볼 수 있는 근거:

- `/zed/.../image`가 publish되지 않음
- `/vla/image_rgb` image hash가 변하지 않음
- image stamp가 증가하지 않음
- camera_adapter encoding 변환 실패 log 발생

현재 dry-run bag에서는 image hash가 계속 바뀌므로 camera freeze 가능성은 낮다.

### 5.9 `build/`, `install/`, `log/`

| 디렉터리 | 역할 |
|---|---|
| `build/` | `colcon build` 중간 산출물 |
| `install/` | ROS2 package install/symlink-install 산출물 |
| `log/` | colcon build log |

주의:

- `install/openvla_doosan_runtime/share/openvla_doosan_runtime/config/runtime.yaml`은 symlink chain을 통해 source config를 가리킨다.
- 새 config 파일을 package share에서 사용하려면 `colcon build --symlink-install --packages-select openvla_doosan_runtime`를 다시 실행해야 한다.
- 현재 `runtime_debug_bag.yaml`은 빌드 후 install share에 symlink로 들어간 상태이다.

## 6. 현재 rosbag 증거

### 6.1 `vla_action_repeat_20260728_01`

경로:

```text
/home/ubuntu/robot_ws/src/openvla_doosan_runtime/bags/vla_action_repeat_20260728_01
```

요약:

```text
duration: 19.18s
messages: 1407
/vla/image_rgb: 1023
/doosan/current_pose: 192
/vla/robot_ready: 192
/vla/raw_action: 0
```

판단:

- episode가 꺼져 있던 구간이라 raw action이 없다.
- image와 current pose 기록은 정상.
- 모델 반복 여부 판단에는 사용 불가.

### 6.2 `vla_debug_dryrun`

경로:

```text
/home/ubuntu/robot_ws/src/openvla_doosan_runtime/bags/vla_debug_dryrun
```

기록 조건:

- `runtime_debug_bag.yaml`
- `doosan_bridge.dry_run=true`
- 실제 robot motion 없음
- episode max steps 12

rosbag info:

```text
duration: 82.36s
messages: 5907
/vla/raw_action: 12
/vla/model_status: 12
/vla/action_valid: 12
/vla/target_pose: 12
/vla/image_rgb: 4083
/doosan/current_pose: 801
/doosan/status: 37
/rosout: 108
/vla/enable: 2
/vla/instruction: 1
```

분석 결과:

```text
raw_action:
  count: 12
  unique_exact: 1
  longest_repeat_epsilon_1e-12: 12
  common x12:
    [3.239264112769348e-05,
     4.9954392563764105e-05,
     -0.00014253266217808397,
     0.001078210179419531,
     0.00034518010059698956,
     0.0012115914716438886,
     0.996078431372549]

image_rgb:
  hashed: 100
  unique_hashes: 100

current_pose:
  count: 801
  xyz_min: [10.226394653320312, 230.0408172607422, 581.5751342773438]
  xyz_max: [10.22643756866455, 230.04168701171875, 581.5755615234375]

target_pose:
  count: 12
  xyz_min == xyz_max:
    [11.198216802495356, 231.54031878863168, 578.5751342773438]
```

판단:

- image input은 변하고 있다.
- robot pose는 dry-run이라 거의 변하지 않는다.
- 그럼에도 raw action은 12개 모두 동일하다.
- 따라서 반복 action은 robot motion feedback 문제가 아니라 model inference output 문제로 분리된다.

## 7. 객관 평가 지표

### 7.1 모델/학습 문제 판정 지표

다음 조건이 만족되면 모델/학습/전처리 문제 가능성이 높다.

| 지표 | 기준 | 현재 값 | 판정 |
|---|---:|---:|---|
| dry-run raw action unique count | 12 step 중 2개 이상이어야 정상 기대 | 1 | 비정상 |
| longest repeated action | 3 이상이면 collapse 의심 | 12 | 비정상 |
| image unique hash | 100 sample 중 충분히 변해야 함 | 100/100 unique | camera freeze 아님 |
| model status | inference error 없어야 함 | 12 x `inference_ok` | load/inference는 성공 |
| gripper output 다양성 | task phase에 따라 변해야 함 | 12 x `0.996` | open 편향 |
| repeated_action_detected | 없어야 함 | count 3~12 발생 | 비정상 |

현재 결과는 모델/학습 문제 쪽이다.

가능 원인:

1. 학습 데이터가 적고 task/instruction이 하나뿐이라 모델이 평균적 open/near-stationary action으로 수렴.
2. gripper open 비율이 75.25%라 gripper가 open으로 치우침.
3. image augmentation/center crop 또는 camera view가 학습과 runtime에서 불일치.
4. action discretization 후 특정 token 조합이 반복됨.
5. 학습 step 부족 또는 overfit/underfit.
6. `unnorm_key`는 맞지만 dataset statistics 또는 TFDS root가 학습 의도와 다를 가능성.
7. action unit/gain 설정(`translation_scale_to_mm=30000`)이 모델 출력 평가를 왜곡할 수 있음. 단, raw action 반복 자체의 원인은 아님.

### 7.2 로봇/런타임 문제 판정 지표

다음 조건이면 robot/runtime 문제 가능성이 높다.

| 지표 | robot/runtime 문제 근거 |
|---|---|
| raw action이 다양하지만 target pose가 항상 같은 값 | `action_adapter` 변환 문제 |
| target pose가 맞는데 current pose가 반대로 움직임 | axis sign, Doosan frame, robot command 문제 |
| target pose가 맞는데 `move_line` 실패 | Doosan service/driver 문제 |
| current pose가 motion 후 갱신되지 않음 | pose polling/service 문제 |
| image hash가 변하지 않음 | camera_adapter/ZED 문제 |
| dry-run에서는 action 다양, real motion에서만 반복 | robot feedback 또는 scene dynamics 문제 |

현재 dry-run 결과는 이 조건과 반대이다.

### 7.3 좌표계/단위 문제 판정 지표

확인해야 할 변환:

```text
dataset tcp_pose:
  meter/radian, base frame

model raw_action:
  [dx,dy,dz,dRx,dRy,dRz,gripper]
  meter/radian + gripper

action_adapter:
  action[:3] * axis_sign[:3] * translation_scale_to_mm
  action[3:] * axis_sign[3:] * rotation_scale_to_deg
  clip by max_translation_step_mm/max_rotation_step_deg
  target = current_pose + delta

Doosan current/target pose:
  millimeter/degree
```

검증 기준:

- `translation_scale_to_mm=1000.0`이면 dataset 정의와 일치.
- 현재 `30000.0`은 실험적 gain으로 보이며, action이 쉽게 clamp된다.
- `axis_sign[2]`는 과거 `-1.0`에서 `1.0`으로 수정했다. 수집 action은 `next_pose-current_pose`였으므로 일반적으로 Z 부호 반전이 필요하지 않다.
- `apply_rotation=false`이므로 rotation action은 현재 robot command에 반영되지 않는다.

좌표계 검증 command:

```bash
ros2 topic echo /doosan/current_pose
ros2 topic echo /vla/raw_action
ros2 topic echo /vla/target_pose
ros2 topic echo /vla/action_status
```

수동 계산:

```text
expected_delta_mm = clip(raw_action[:3] * axis_sign[:3] * translation_scale_to_mm,
                         -max_translation_step_mm,
                         +max_translation_step_mm)
expected_target_xyz = current_pose[:3] + expected_delta_mm
```

bag의 `/vla/target_pose`가 이 값과 다르면 `action_adapter` 문제이다.

## 8. 재현/진단 절차

### 8.1 일반 runtime 실행

```bash
conda activate openvla
cd ~/robot_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
export PYTHONPATH=~/robot_ws/src/openvla:$PYTHONPATH

ros2 launch openvla_doosan_runtime runtime.launch.py \
  inference_python:=$CONDA_PREFIX/bin/python3
```

### 8.2 dry-run runtime 실행

실제 robot motion 없이 모델 output만 본다.

```bash
conda activate openvla
cd ~/robot_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
export PYTHONPATH=~/robot_ws/src/openvla:$PYTHONPATH

ros2 launch openvla_doosan_runtime runtime.launch.py \
  config:=/home/ubuntu/robot_ws/install/openvla_doosan_runtime/share/openvla_doosan_runtime/config/runtime_debug_bag.yaml \
  inference_python:=$CONDA_PREFIX/bin/python3
```

### 8.3 dry-run bag 기록

```bash
ros2 bag record --compression-mode file --compression-format zstd \
  -o ~/robot_ws/src/openvla_doosan_runtime/bags/vla_debug_dryrun_new \
  /vla/image_rgb /vla/raw_action /vla/target_pose /doosan/current_pose \
  /vla/model_status /vla/action_status /vla/action_valid /vla/robot_ready \
  /vla/episode_status /vla/instruction /vla/enable /doosan/status /rosout
```

다른 터미널:

```bash
ros2 service call /vla/set_episode std_srvs/srv/SetBool "{data: true}"
```

### 8.4 bag 분석

```bash
python3 ~/robot_ws/src/openvla_doosan_runtime/scripts/analyze_vla_bag.py \
  ~/robot_ws/src/openvla_doosan_runtime/bags/vla_debug_dryrun_new
```

좋은 결과의 기대값:

```text
raw_action count >= 10
unique_exact > 1
longest_repeat_epsilon_1e-12 < 3
image unique hashes > 80% of sampled hashes
inference_error 없음
action_rejected 없음 또는 반복 action 방어에 의한 reject만 있음
```

나쁜 결과:

```text
raw_action unique_exact == 1
longest_repeat >= 3
image hashes are unique
```

이 경우 모델/학습/전처리 문제로 분류한다.

## 9. 개선 실험 제안

### 9.1 먼저 로봇 없이 할 실험

1. `translation_scale_to_mm=1000.0`인 debug config를 추가해 target delta가 dataset unit과 일치하는지 확인.
2. `center_crop_scale=1.0`과 `0.9`를 dry-run bag으로 비교.
3. `do_sample=true`, 낮은 `temperature`로 action 다양성이 살아나는지 offline에서만 확인. 실제 motion에는 바로 쓰지 않는다.
4. raw dataset image와 runtime `/vla/image_rgb`를 side-by-side로 저장해 camera orientation/crop 차이를 확인.
5. 같은 image에 대해 `openvla/vla-scripts/deploy.py` REST server와 ROS inference node output이 같은지 비교.
6. 학습 checkpoint별 output 비교. 예: `b8`, 다른 save step, base openvla.

### 9.2 학습 데이터/모델 쪽 실험

1. episode 수와 scene 다양성 증가.
2. instruction 다양화. 현재 모든 episode instruction이 `pick up the cube` 하나뿐이다.
3. gripper close/open phase 균형 확인. 현재 open 75.25%, close 24.75%.
4. 정지/near-zero action 비율 감소. 현재 `norm <= 0.003m`가 33.87%.
5. success-only 데이터만 쓰는 것이 좋은지 검토. 실패/복구가 없으면 모델이 중간 상태 대응을 못할 수 있다.
6. train/val split을 만들어 action accuracy 외에 held-out rollout-like image inference 평가.
7. 학습 후 offline evaluation set에서 `unique_action_ratio`, `longest_repeat`, `action_norm_distribution`, `gripper_open_ratio`를 계산.

### 9.3 로봇 쪽 실험

1. OpenVLA 없이 고정 delta command로 `action_adapter`와 `doosan_bridge`를 검증.
2. `/doosan/current_pose`가 실제 teach pendant 좌표와 일치하는지 확인.
3. X/Y/Z 각각 +1mm target을 보냈을 때 실제 이동 방향이 맞는지 확인.
4. gripper DO index 1/2가 실제 open/close와 일치하는지 확인.
5. workspace limit이 실제 cell과 맞는지 확인.

## 10. 현재 안전 설정

실제 motion config:

```text
max_translation_step_mm = 1.0
repeated_action_limit = 2
reject_repeated_actions = true
apply_rotation = false
axis_sign = [1, 1, 1, 1, 1, 1]
dry_run = false
```

의미:

- 같은 raw action이 3번째 나오면 episode가 실패 처리된다.
- 1 step translation은 axis별 최대 1mm로 제한된다.
- rotation은 모델이 예측해도 실제 target에는 반영하지 않는다.
- gripper command는 threshold 0.5 기준으로 open/close bool로 변환된다.

## 11. 추적해야 할 결정 사항

아직 확정이 필요한 항목:

1. 학습 command 원본:
   - `--data_root_dir`
   - `--dataset_name`
   - `--batch_size`
   - `--max_steps`
   - `--save_steps`
   - `--image_aug`
   - `--run_id_note`
2. TFDS root가 `/home/ubuntu/robot_ws/src/tensorflow_datasets`였는지 `/home/ubuntu/robot_ws/tensorflow_datasets`였는지.
3. `translation_scale_to_mm=30000.0`을 의도한 이유.
4. runtime camera image가 학습 image와 같은 방향/색상/crop인지.
5. 학습 checkpoint가 충분한 step에서 저장된 것인지.
6. 현재 모델의 offline validation action diversity.

## 12. 판정 기준 최종안

다음 3개 실험을 모두 기록하면 원인 분리가 가능하다.

### A. dry-run model-only bag

- 조건: `dry_run=true`, robot motion 없음.
- 성공 기준: image hash가 변할 때 raw action도 상황에 따라 변한다.
- 실패 기준: image hash가 변하는데 raw action이 같은 값으로 3회 이상 반복.
- 현재 결과: 실패. 모델/학습/전처리 쪽.

### B. fixed synthetic action robot test

- 조건: OpenVLA inference를 끄고 작은 known delta를 `/vla/raw_action` 또는 `/vla/target_pose`에 직접 publish.
- 성공 기준: target pose 계산과 실제 robot 이동 방향/크기가 일치.
- 실패 기준: target은 맞는데 robot이 반대로 가거나 service 실패.
- 현재 결과: 아직 별도 기록 없음.

### C. offline dataset/replay evaluation

- 조건: raw dataset image 또는 held-out image를 모델에 넣고 action distribution 측정.
- 성공 기준: action norm, sign, gripper phase가 label distribution과 유사.
- 실패 기준: 대부분 같은 token/action, gripper open 고정, action norm collapse.
- 현재 결과: ROS dry-run 기준으로 collapse 관측.

## 13. 보관된 명령

빌드:

```bash
cd ~/robot_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select openvla_doosan_runtime
source install/setup.bash
```

토픽 확인:

```bash
ros2 topic list
ros2 topic hz /vla/image_rgb
ros2 topic echo /vla/raw_action
ros2 topic echo /vla/target_pose
ros2 topic echo /doosan/current_pose
ros2 topic echo /vla/model_status
ros2 topic echo /vla/action_status
```

episode 시작/정지:

```bash
ros2 service call /vla/set_episode std_srvs/srv/SetBool "{data: true}"
ros2 service call /vla/set_episode std_srvs/srv/SetBool "{data: false}"
ros2 service call /vla/stop std_srvs/srv/Trigger "{}"
```

현재 action_adapter live safety parameter:

```bash
ros2 param get /action_adapter repeated_action_limit
ros2 param get /action_adapter max_translation_step_mm
ros2 param get /action_adapter axis_sign
```

현재 확인값:

```text
repeated_action_limit = 2
max_translation_step_mm = 1.0
axis_sign = [1, 1, 1, 1, 1, 1]
```

