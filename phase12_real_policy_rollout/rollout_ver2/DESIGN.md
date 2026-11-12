# rollout_ver2 설계 및 확인 현황

## 문서 규칙과 안전 범위

- **[확인]** 이 디렉터리의 구현과 검증은 오프라인 전용이다. ROS 노드를 실행하거나 로봇 명령을 발행하지 않는다. **출처:** 작업 지시; `kinematics/solver.py`, `kinematics/validate_offline.py`의 import/실행 경로 점검.
- **[확인]** 내부 운동학 단위는 rad/m이며 action을 clamp하거나 수정하지 않는다. 검사 실패는 예외/정지 사유로 다룬다. **출처:** 작업 지시; `kinematics/solver.py`.
- **[가정]** 아래에서 작업 지시가 제공한 기존 조사 결과는 원 출처를 다시 재현하지 않은 경우에도 해당 조사 결과가 정확하다고 본다. **출처:** 작업 지시(2026-11-12).

## 1. 설계 결정

- **[확인]** 팔 제어 인터페이스는 `ServojStream(pos[6] deg, vel, acc, time, mode=0/DR_SERVO_OVERRIDE)`와 호스트 IK이며 100 Hz 스트리밍을 사용한다. **출처:** 작업 지시의 확정 설계; `dsr_msgs2/msg/ServojStream` 및 드라이버 사전 조사.
- **[미확인]** `time/vel/acc` 값은 저속 실기 시험으로 결정해야 한다. 현재 값은 정하지 않는다. **출처:** 작업 지시; 실기 시험 미수행.
- **[확인]** 실행은 시간 스케일링 모드 A(동기)다: 관측 → 추론 → `D=0.5 s` 구간 실행(최소 jerk, 양 끝 속도/가속도 0) → 정착 확인 → 반복. 한계 초과 시 경로/action을 바꾸지 않고 D를 늘린다. **출처:** 작업 지시의 확정 설계.
- **[확인]** 명령 자세를 적분 기준으로 삼아 `p_t=p_ref+Δp`, `R_t=Exp(ω)R_ref`(world 표현, 왼쪽 곱셈)로 계산한다. 모델 gripper 값은 sim에서는 연속 absolute이고, 실기 I/O 계층에서만 `<0.5=OPEN`, `>=0.5=CLOSE`로 이진화한다. **출처:** matched15 evaluator 조사; 작업 지시(2026-11-13).
- **[확인]** 실제 자세는 추종 오차, 관절 마진, 정착 확인의 감시 전용이다. 명령 누적 기준으로 되먹임하지 않는다. **출처:** 작업 지시; v1 positive-feedback 분석.
- **[사용자 보고·사진 미확인]** 현재 pendant는 일반/감소 모두 Default: J1 ±360, J2 ±95, J3 ±135, J4 ±360, J5 ±135, J6 ±360°, controller tolerance ±3°다. `pendant_limits_recorded`는 사진 전까지 false다. **출처:** 사용자 현장 보고(2026-11-13); `controller/pendant_limits.json`.
- **[가정 후보]** host margin은 관절별 5°이며 host stop은 J1 ±355, J2 ±90, J3 ±130, J4 ±355, J5 ±130, J6 ±355°다. 위반 사유는 `JOINT_LIMIT_MARGIN:J<n>`이고 clamp하지 않는다. 보호 정지 예상 지점은 pendant 바깥 방향으로 tolerance 3°를 더해 함께 기록한다. **출처:** 작업 지시; `controller/joint_limits.py`.
- **[확인(구현)]** 설정 loader는 host stop이 pendant 안쪽인지, pendant가 두산 설정 가능 최대치(J1/J2/J4/J5/J6 ±360, J3 ±160°) 이내인지, 출처 날짜/evidence 경로가 있는지 검사한다. **출처:** 사용자 제공 매뉴얼 값; `controller/joint_limits.py`.
- **[확인(구현)]** Default가 아닌 경우 `extension_reason`, `pendant_limits_recorded`, `extended_range_clearance_checked`의 실제 evidence 파일이 모두 없으면 실행을 거부한다. **출처:** `controller/joint_limits.py`, `runtime/executor.py`.
- **[확인]** 끝 토큰 연속 감시는 arm 6차원(`Δp,ω`)에만 적용한다. `endpoint_policy∈{log_only,stop}`이며 기본은 `log_only`다. gripper는 endpoint 감시에서 제외한다. **출처:** 작업 지시(2026-11-13); `controller/precheck.py`, `controller/controller_config.json`.
- **[가정]** N의 후보는 **3회**다. 이는 구현 한계나 승인된 안전값이 아니라 운영 검토용 후보이며 실기 전에 확정해야 한다. **출처:** 이번 설계 제안; 단발 양자화 노이즈와 지속 포화 구분 목적.
- **[확인(sim parity)]** matched15 gripper는 raw action[6]의 연속 absolute `[0,1]` target이며 full travel 0.6 s의 slew로 매 physics step 갱신되고 팔 joint target과 같은 step에 병행 적용된다. v2 offline controller도 debounce/binary 변환 없이 이 규약을 구현한다. **출처:** `vla_validation.py:1124-1181`; `controller/gripper.py`.
- **[확인]** 실기 장치는 주강로보테크 JRT JEGH-3520이며 기존 코드는 flange Tool DO 서비스를 사용한다. 최신 실기 명령 후보는 OPEN=DO1 이중 timed pulse, CLOSE=DO2 단일 timed pulse이고 상태 변화 때만 명령한다. **[미확인]** CLOSE 물리 완료, DI 완료 신호, I/O 포트/극성의 현장 재확인. **출처:** 사용자 현장 정보; `reports/gripper_control_audit.md`.
- **[통제 변수]** sim은 연속 absolute target, 0.6 s slew, 팔과 동시 동작이지만 실기는 thresholded binary timed pulse다. 첫 세션 `gripper_test` 동안 팔은 정지하며 `policy`에서는 gripper를 기록만 한다. 이 차이는 실기 성능 해석에서 별도 통제 변수다. **출처:** matched15 evaluator; 작업 지시; `runtime/gripper_io.py`.
- **[확인]** action 회전 규약은 `rotation_frame=world_left`, 즉 world 좌표의 `Δp`와 `R'=Exp(ω)R`이다. sim matched15 evaluator가 `target_position=current_position+action[:3]` 및 `quaternion_multiply(delta,current)`를 사용한다. 기준 pose는 Lula `tool0`의 world pose다. 비교/회귀용 `tool_right` 설정은 유지한다. **출처:** `isaac_sim/a0509_control/vla_validation.py:119-151,1023-1060`; `lula_planner.py`; `reports/controller_replay.md`.
- **[확인]** 프레임을 `tool0_sim=T_link6·[xyz 0,0,0.01,yaw -90°]`, `tool0_real=T_link6·[xyz 0,0,0.01,yaw +90°]`로 구분한다. 실기 명령·피드백·로그는 `tool0_real`, sim 비교는 `tool0_real↔tool0_sim`을 사용한다. **출처:** sim FK 검증; 현장 육안 확인(2026-11-13); `controller/geometry.py`.
- **[확인]** offline controller에는 파일 전용 `DryRunPublisher`가 있고, 별도 runtime 계층에는 기본 비활성 `ServojPublisher`가 있다. 후자는 explicit enable, 모드 gate, 확인된 time/vel/acc가 모두 없으면 발행을 거부한다. `.msg` 정의대로 pos/vel/acc는 각각 6원소이며 pos는 deg, mode=0, QoS depth 1이다. **출처:** `controller/publisher.py`; `runtime/servoj.py`; 읽기 전용 `dsr_msgs2/msg/ServojStream.msg`.
- **[확인(구현)]** 관절 한계는 `pendant_limits.json`에서만 로드한다. 현재 값은 사용자 보고 Default지만 사진 미확인이므로 실행 gate는 닫혀 있다. 속도/가속도/workspace는 계속 후보다. **출처:** `controller/config.py`, `controller/pendant_limits.json`.
- **[가정]** IK 허용 오차 후보는 위치 0.05 mm, 회전 0.005°다. solver는 먼저 수치 정밀도 해를 시도하고 실패할 때만 이 후보로 재판정한다. **출처:** 작업 지시의 예시 후보; `controller/trajectory.py`.
- **[확인(offline branch)]** matched15를 episode 첫 pose=`sim_home` seed, 이후=직전 해 seed로 다시 복원하면 J5 전부 음수이고 J6가 연속 음수 branch다. 다만 J6는 `[-288.773,-178.292]°`로 Isaac225 envelope보다 넓고, 1 mm/0.1° 겹침 pose는 없어 paired joint 오차는 확인하지 못했다. **출처:** `reports/matched15_branch_and_ik_timing.md`.
- **[가정 후보]** matched15 branch 연속 σmin p0/p1/p5/p50은 `0.100446/0.104796/0.110880/0.156459`, Isaac225는 `0.109456/0.109456/0.109456/0.177201`이다. 두 p0 중 낮은 값의 약 80%인 `0.0803`을 새 후보로 사용한다. 과거 wrong-branch 0.08 및 Isaac-only 0.0875 근거는 폐기한다. **출처:** `reports/matched15_branch_reconstruction.json`.
- **[확인(구조)]** `SegmentGenerator.generate()`는 전체 pose·IK·속도·가속도·관절 margin·특이점을 사전 계산/검사한 뒤에만 `Segment`를 반환한다. 실패 시 발행 가능한 segment가 생성되지 않는다. offline full-sequence 진단 replay만 명시적으로 final preflight를 비활성화한다. **출처:** `controller/trajectory.py`; `reports/matched15_branch_and_ik_timing.md`.
- **[확인(구현)]** `sim_joint_envelope`는 Isaac225 실제 관절각과 matched15 sim-home-seed 연속 branch IK 복원 관절각을 모두 실기 J6 기준으로 바꾼 뒤 관절별 min/max 합집합으로 저장한다. 출처별 범위도 설정과 분석 결과에 함께 보존한다. 매 sample의 이탈 관절/초과량을 기록하지만 `log_only`이며 정지 조건이 아니다. **출처:** `controller/controller_config.json`, `controller/precheck.py`, `runtime/monitor.py`, `reports/joint_limits_analysis.json`.
- **[확인]** action과 IK 결과는 clamp하지 않는다. step/workspace/끝 토큰/관절 margin/IK 불연속 위반은 정지 사유이며, 동역학 후보 초과는 동일 경로의 D만 늘린다. **출처:** `controller/precheck.py`, `controller/trajectory.py`.
- **[확인]** gripper endpoint 제외 뒤 190404 최초 endpoint streak 정지는 step 2가 아니라 step 25의 `rx min`이다. gripper binary 상태 변화는 step 117/168/169/170에 별도 기록된다. **출처:** `reports/controller_replay.json`.
- **[확인]** `gripper_mount_offset_J6_deg=180°`다. 실기 그리퍼는 sim 대비 link_6 z축으로 180° 회전 장착되어 `J6_real=J6_sim+180°`일 때 같은 물리 그리퍼 방향이다. **출처:** 현장 육안 확인(2026-11-13).
- **[확인]** sim home은 `[-182.535617,-1.058471,-40.983990,1.253979,-113.505675,-178.500246]°`, real home은 J6에 180°를 더한 `[-182.535617,-1.058471,-40.983990,1.253979,-113.505675,1.499754]°`다. 자동 home 이동은 하지 않는다. **출처:** 읽기 전용 sim config; 현장 확인; `reports/home_alignment.md`.
- **[가정 후보]** 시작 전 각 관절이 real home의 ±2° 이내인지 직접 검사한다. wrap이나 alternate branch는 허용하지 않고 `START_NOT_REAL_HOME:J<n>`으로 차단한다. **출처:** `controller/precheck.py`, `controller/controller_config.json`.

## 2. 시스템

- **[확인]** 커널 `6.8.0-138-generic`, PREEMPT_RT 아님, rtprio limit 0. **출처:** 작업 지시의 시스템 조사.
- **[확인]** CPU i9-10900(10C/20T), RAM 31 GiB, GPU RTX 3090 24 GB, NVIDIA driver 580.178.04. **출처:** 작업 지시의 시스템 조사.
- **[확인]** 100 Hz 유휴 더미 루프 jitter는 p50 10.00 ms, p99 11.92 ms, max 12.78 ms다. **출처:** 작업 지시의 측정 결과.
- **[미확인]** 추론과 동시에 실행할 때의 jitter. **출처:** 측정 없음.
- **[확인]** `/usr/bin/python3` 3.10.12에서 rclpy, numpy 1.26.4, scipy 1.15.3, PyKDL 1.5.1, torch 2.10.0+cu128을 사용할 수 있다. **출처:** 작업 지시; 이번 오프라인 import 및 테스트.
- **[확인]** conda base(Python 3.14)는 rclpy를 사용할 수 없다. conda를 사용하지 않는다. **출처:** 작업 지시의 환경 조사.
- **[확인]** PyKDL, kdl_parser, moveit_kinematics가 있고 pinocchio, ikpy는 없다. 다만 `/usr/bin/python3`에서 Python 모듈 `kdl_parser_py`는 import되지 않아 이번 코드는 URDF XML을 읽어 PyKDL chain을 직접 구성한다. **출처:** 작업 지시; 2026-11-12 import 검사; `kinematics/solver.py`.
- **[확인]** Docker 미설치로 두산 에뮬레이터 virtual mode를 사용할 수 없다. **출처:** 작업 지시의 시스템 조사.

## 3. ROS2 / 드라이버

- **[확인]** ROS2 humble, 기본 RMW, `ROS_DOMAIN_ID=71`, `ROS_LOCALHOST_ONLY=1`. **출처:** 작업 지시의 ROS 환경 조사.
- **[확인]** 드라이버는 `~/robot_ws/src/doosan-robot2`, 팀 수정 fork commit `86eaa963`. **출처:** 작업 지시.
- **[확인]** package 버전: dsr_bringup2/common2/description2/msgs2/gazebo2/visualservoing 1.1.0, dsr_controller2 2.33.0, dsr_moveit_config_* 2.0.7, dsr_mujoco 0.1.0, dsr_hardware2/dsr_realtime_control/dsr_tests 0.0.0. **출처:** 작업 지시의 package 조사.
- **[확인]** hardware interface `dsr_hardware2/DRHWInterface`, namespace `dsr01`, 설정 update_rate 100 Hz. **출처:** 작업 지시; 읽기 전용 `dsr_controller2/config/dsr_controller2.yaml` 조사.
- **[확인]** launch에서 `dsr_update_rate.yaml` 없음 경고가 관찰되었다. **출처:** 작업 지시의 launch 기록.
- **[미확인]** 실기에서 실제 적용되는 update rate. **출처:** 위 경고; 실기 재확인 미수행.
- **[확인]** controller `192.168.0.110:12345`, `rt_host=192.168.137.50`; RT 제어는 사용하지 않는다. **출처:** 작업 지시의 설정 조사.
- **[확인]** `servoj_stream` subscriber QoS depth 20, 기본 reliable이며 callback은 별도 검사 없이 `Drfl->servoj(pos,vel,acc,time,mode)`를 호출한다. **출처:** 작업 지시의 driver source 조사.
- **[확인(source)]** 드라이버는 `motion/move_stop`(`dsr_msgs2/srv/MoveStop`)과 `motion/move_joint`(`dsr_msgs2/srv/MoveJoint`) 서비스를 만든다. namespace를 적용한 설정 이름은 `/dsr01/motion/move_stop`, `/dsr01/motion/move_joint`이며 현장 graph에서 전체 이름을 재확인한다. MoveStop mode는 0 QSTOP_STO, 1 QSTOP, 2 SLOW, 3 HOLD다. **출처:** 읽기 전용 `dsr_controller2.cpp:745-748,2407,2420`; `MoveStop.srv:6-11`; `MoveJoint.srv:6-15`.
- **[확인]** RobotState 주요 필드는 `current_posj`, `current_velj`, `target_posj`, `current_posx`, `current_tool_posx`, `rotation_matrix`, `f_current_pos_world[0 tool,1 flange]`, `actual_bk`, `joint_err`, `task_err`, `sync_time`. **출처:** 작업 지시의 message 조사.
- **[확인]** robot_state: 0 INIT, 1 STANDBY, 2 MOVING, 3 SAFE_OFF, 5 SAFE_STOP, 6 EMERGENCY_STOP, 9 RECOVERY, 10 SAFE_STOP2, 11 SAFE_OFF2, 15 NOT_READY. **출처:** 작업 지시의 driver 조사.
- **[확인]** servo 중에도 robot_state가 주로 1이므로 이를 정지 증거로 쓰지 않는다. **출처:** 작업 지시의 실기 로그 분석.
- **[확인]** Ikin/Fkin 서비스 정의가 존재한다. **출처:** 작업 지시의 source 조사.
- **[미확인]** Ikin/Fkin `.srv` 실제 경로, controller firmware/pendant software 버전, 실기 topic 이름/QoS/RobotState 주기. **출처:** 작업 지시; 이번 작업은 하드웨어 미접속.

## 4. 네트워크

- **[확인]** 유선 `enp3s0`, `enp6s0`은 NO-CARRIER/IP 없음(로봇 OFF), Wi-Fi는 192.168.1.98. **출처:** 작업 지시의 host 조사.
- **[확인(현장 구성)]** PC와 로봇은 직접 연결되어 있고 real-mode launch 준비가 되어 있어 별도 IP 설정 절차는 필요 없다. **[미확인]** 연결 NIC/host IP와 ping latency/jitter는 첫 read-only 검사에서 기록한다. **출처:** 사용자 현장 정보(2026-11-13); 실기 접속은 이번 작업에서 금지.

## 5. 로봇 안전 설정

- **[부분 확인]** Robot Limits 관절각은 사용자 보고 Default로 채웠지만 화면 사진이 없다. 관절 속도, TCP 속도·힘, 충돌 민감도와 실제 tool(TCP)는 미확인이다. **출처:** 사용자 보고; `controller/pendant_limits.json`.
- **[확인]** 과거 J5 약 -108°→-138° 뒤 robot_state 10(SAFE_STOP2) 사고가 있었다. **출처:** 작업 지시의 과거 실기 기록.

## 6. 모델 / 추론 서버

- **[확인]** `vanilla_s1_balanced_step8130`은 OpenVLA-7B + LoRA r32, `openvla_token` 256 bins, K=1, action_dim=7, binary gripper, proprio 없음. **출처:** 작업 지시의 model/config 조사.
- **[확인]** 학습 데이터는 `openvla_a0509_legacy_midheight_corrective_speed1p712696_native5hz_300_v1`, key `a0509_sim_cube_pick`, train 12998/val 1604, trajectories 270, corrective_fraction 0.544, image augmentation 사용, sim-only다. **출처:** 작업 지시의 manifest/statistics 조사.
- **[확인]** q01/q99를 `[-1,1]`로 정규화한다. 통계 원본은 `models/vanilla_s1_balanced_step8130/dataset_statistics.json`. **출처:** 작업 지시.
- **[확인]** bin center 최대는 `1-1/255`; 끝 토큰 역정규화 값을 계산할 수 있다(예: rz max 0.064914, gripper min 0.001961). **출처:** 작업 지시의 tokenizer/statistics 계산.
- **[확인]** matched15 sim 평가 15 episode 성공률은 1.0. **출처:** 작업 지시의 평가 결과.
- **[확인]** `scripts/serve.sh vanilla <port>` → `runtime/vanilla-support/serve_adapter.py`; 별도 venv `environment/a6000_ubuntu22_py310`; JPEG base64와 instruction을 받는 HTTP `/predict`; port 8766 사용 이력이 있다. **출처:** 작업 지시의 server 조사.
- **[확인]** matched15 경로의 client는 Isaac camera RGB uint8을 JPEG quality 92로 보내며, server는 bottom crop 0.0, center crop 없음, processor `resize-naive` 224×224를 사용한다. **출처:** `isaac_sim/a0509_control/vla_client.py:45-78`; `runtime/vanilla-support/serve_adapter.py:108-176`; model `training_config.json`, `processor/preprocessor_config.json`; matched15 evidence의 `server.crop_bottom_fraction=0.0`.
- **[확인]** 기존 실기 rollout client는 RGB이지만 JPEG quality 95를 사용해 matched15의 quality 92와 다르다. vanilla server는 center crop을 추가하지 않는다. v2 `controller/observation.py`는 RGB→JPEG q92/optimize false→server RGB decode→resize-naive dual-backbone tensor를 재현하며 reference와 bit-identical tensor를 확인했다. **출처:** 읽기 전용 `../14_real_rollout_integration/rollout_latency.py`; `controller/test_observation.py`.
- **[확인]** matched15 지시문은 layout target에 따라 정확히 `Pick up the red/yellow/blue cube.`였고 15 episode에 반복됐다. **출처:** `validation/evidence/vanilla/success_profile_s1_balanced_step8130_matched15_k1_seed9406_15ep.json`의 results.
- **[확인]** 측정 inference 주기는 2.1–2.4 Hz(약 0.42–0.48 s latency). **출처:** 작업 지시의 측정 결과.

## 7. 학습 데이터 수집(sim)

- **[확인]** Isaac Sim + Lula planner, physics_dt 1/60, motion_speed_scale 2.5, EE frame `tool0`, joint-position target control, 5 Hz 기록, 1280×720 image, idle frame filtering 사용. **출처:** 작업 지시의 collector/config 조사.
- **[가정]** OFT kit `episode_recorder.py`/`preprocessing.py` 기준 action은 `Δp=p_{t+1}-p_t`(world,m), `ω=rotvec(q_{t+1}q_t^-1)`(world,left multiply), gripper absolute target, alignment `observation_t→observation_{t+1}`다. legacy dataset은 아직 확인하지 못했다. **출처:** 작업 지시.
- **[확인]** 이전 보고서의 1,734 sample은 sim이 아니라 `single_robot_simple.py`가 만든 실기 portable dataset이며 pose도 feedback이 아닌 `planned_commanded_pose`였다. 이를 sim FK 검증 결과로 사용한 것은 데이터 분류 오류다. **출처:** `raw_dataset_oft/dataset_manifest.json`, episode `metadata.json`의 `coordinate_frame=base`, `feedback_pose_samples=0`; `reports/kinematics_validation_2.md`.
- **[확인]** 실제 Isaac replay 225 sample에서 `T_rec=T_world_base·FK_base_link_link6(q)·T_link6_tool0`를 동시 적합하면 `R_world_base≈I`, `p_world_base≈[-0.000389,-0.000257,0.823526] m`, tool offset `[0,0,0.01] m`, yaw `-90.000210°`이며 p99 residual은 위치 `4.90e-13 mm`, 회전 `4.23e-14°`다. sim `tool0`는 `link_6`와 동일하지 않다. **출처:** `reports/kinematics_validation_2.json`; sim URDF `validation/a0509_project/urdf/a0509.urdf`.
- **[확인]** 해당 runtime/scene에서 sim world와 robot `base_link` 축 회전은 항등(`R_world_base≈I`)이다. OpenVLA action의 `Δp`와 left-multiply `ω`는 기록된 world frame이고, 따라서 이 scene에서는 base_link 축과 회전상 동일하다. **출처:** `preprocessing.py`; `lula_planner.py`; 양쪽 변환 적합 결과.
- **[미확인]** sim camera pose/FOV와 real camera 일치, idle-filter threshold. **출처:** 비교/설정 확인 미수행.

## 8. rollout v1 교훈

- **[확인]** 실제 자세를 목표 기준으로 쓰면서 stream을 내부 명령에서 이어가 회전 positive feedback이 발생했다. v2는 명령 자세 기준을 단일화한다. **출처:** 작업 지시의 v1 분석.
- **[확인]** 속도 적분형 생성기는 129/208 구간 overshoot를 냈다. **출처:** 작업 지시의 v1 로그 분석.
- **[확인]** servol 20 Hz, `time=0.7`은 매 cycle 목표의 약 40%만 추종했다. **출처:** 작업 지시의 v1 측정.
- **[확인]** ZYZ C값 ±180° 통과와 발산 시작이 겹쳤다. **[미확인]** 인과관계. v2는 servoj로 Euler-angle 경로를 제거한다. **출처:** 작업 지시의 v1 로그 분석.
- **[확인]** 대기 중 robot_state 미갱신으로 stale 판정이 발생했으므로 robot_state 단독 정지 증거를 금지한다. **출처:** 작업 지시.
- **[확인]** rz 최대 token 반복은 관찰된 분포 이탈 신호다. **출처:** 작업 지시의 v1 prediction log.
- **[확인]** 구 생성기 정상 구간 추종 오차는 위치 p99 7.0 mm, 회전 p99 2.3°. **[가정]** 10 mm/3°는 임시 threshold 후보일 뿐 승인된 안전 한계가 아니다. **출처:** 작업 지시.

## 9. v2 실기 연동 계층(현재 기본 비활성)

- **[확인(구현)]** `RobotFeedback`는 RobotState/JointState 수신 host monotonic 시각, FK `tool0_real`, 관절 속도, robot_state와 brake state를 제공하고 JSONL에 frame 이름·위치·회전행렬을 기록한다. 0.1 s stale은 **[후보]**이며 실제 topic/QoS는 **[미확인]**이다. **출처:** `runtime/feedback.py`, `runtime/runtime_config.json`.
- **[가정 후보]** 매 sample 감시값은 추종 위치 10 mm/회전 3°, zero-velocity 유지 0.3 s다. 승인 한계가 아니며 첫 세션 evidence로 보정 전 policy 실행에 사용할 수 없다. **출처:** 작업 지시; `runtime/monitor.py`, `runtime/stop.py`.
- **[가정 후보·미확인]** 두 정지 시험의 MoveStop mode는 설정 하나로 고정한 `DR_QSTOP=1`이다. 이는 driver 일반 stop 기본값과 일치하는 후보일 뿐이며 현장 검토가 필요하다. `stop_test_movej`와 `stop_test_servoj`가 같은 값을 사용한다. **출처:** 읽기 전용 `dsr_robot.h:196`; `DSR_ROBOT2.py:272-279`; `runtime/runtime_config.json`, `runtime/stop.py`.
- **[확인(구현)]** 정지 순서는 command publisher inhibit → 설정된 MoveStop 요청 → 계속 수신한 JointState의 zero-velocity 구간 확인이다. ACK 실패는 SAFE_STOP/OFF 계열이 먼저 관찰된 경우만 허용하며, 확인 실패 뒤 추가 명령은 영구 금지한다. **출처:** `runtime/stop.py`와 mock tests.
- **[확인(구현)]** 정지 시험은 세션 1의 `stop_test_movej`와 세션 2의 `stop_test_servoj`로 분리한다. 전자는 real home에서 J6 +2°, 5°/s 이하 MoveJoint 중 정지 서비스·판정 로직을 검증하며 Servoj 중 정지를 대신하지 않는다. 후자는 Servoj parameter 확인 뒤 같은 stop mode로 별도 검증한다. synthetic/policy는 `stop_servoj_verified`를 필수로 한다. **출처:** `runtime/gate.py`, `runtime/executor.py`, `docs/first_live_session.md`.
- **[확인(구현)]** 그리퍼 gate는 배선 선행 증거 `gripper_wiring_recorded`와 시험 결과 `gripper_io_verified`로 분리한다. gripper_test 선행 조건은 pendant limits, live topics, wiring 기록, 팔 정지 evidence이며 성공 결과 review 뒤에만 I/O verified를 등록한다. **출처:** `runtime/gate.py`, `runtime/gates.json`.
- **[확인(구현)]** 모든 물리 mode는 fresh human confirmation, explicit physical enable, mode별 evidence gate와 non-null 필수 설정을 요구한다. dryrun_live/gripper_test/stop_test_movej에는 Servoj time/vel/acc와 tracking threshold가 필요하지 않다. **출처:** `runtime/gate.py`, `runtime/executor.py`.
- **[미확인]** Servoj time/vel/acc, MoveJoint acceleration, MoveStop mode 현장 승인, pendant evidence, live topics/QoS/rate, tracking calibration, 그리퍼 배선/I/O 결과는 연구실 evidence 없이는 관련 gate를 통과할 수 없다. **출처:** `runtime/runtime_config.json`; `docs/first_live_session.md`.

## 10. 열린 항목(우선순위)

1. **[확인·종료]** sim world ↔ robot base 변환: 현재 Isaac runtime/scene 및 `base_link` 기준 `R_world_base≈I`; translation은 약 `[-0.000389,-0.000257,0.823526] m`. 다른 scene/base-yaw 인자에는 재적용할 수 없으므로 scene별 metadata 보존이 필요하다.
2. **[부분 종료]** matched15 sim 적용 규약과 평가 지시문은 확인했다. legacy 학습 dataset 자체의 instruction/action 생성 provenance는 학습 server `/mnt/SSD1` manifest 확인이 여전히 필요하다.
3. **[확인·종료]** `link_6→tool0_sim`은 xyz `[0,0,0.01] m`, yaw `-90°`; 현장 장착 보정을 포함한 `link_6→tool0_real`은 xyz `[0,0,0.01] m`, yaw `+90°`다. pendant TCP 설정값 자체는 별도로 기록한다.
4. **[미확인]** URDF와 실기 flange 일치: paired flange 필드가 없어 posx fallback만 검증했고 1 mm/0.1° 목표를 완전 통과하지 못함.
5. **[부분 종료]** pendant 관절각 Default 값은 사용자 보고로 채웠다. 화면 사진 evidence, 관절 속도/기타 Robot Limits, tool 설정, firmware version은 미확인이다.
6. **[미확인]** 실기 topic/QoS/주기, feedback rate와 ping jitter. `tools/live_readonly_check.sh`로 첫 세션에 기록한다.
7. **[확인·종료(matched15)]** center crop 없음, bottom crop 0.0, JPEG quality 92, RGB. legacy 학습 원본 instruction은 항목 2에 남긴다.
8. **[미확인]** inference 동시 100 Hz jitter.
9. **[확인·종료]** endpoint 감시는 arm 6차원만 사용하고 gripper는 상태 변화만 기록한다.
10. **[확인·종료(offline 분류)]** 새 σmin 0.0803 후보로 190404 replay를 재실행하면 world_left는 step 72/sample 12의 σmin 0.080275에서, tool_right는 step 36/sample 25의 J5 -130.059°에서 먼저 정지한다. **출처:** `reports/controller_replay.json`, `reports/matched15_branch_and_ik_timing.md`.
11. **[부분 종료·현장 확인 필요]** JEGH-3520의 기존 Tool DO 명령 경로와 OPEN 실물 관찰은 확인했다. CLOSE 물리 동작, 포트/극성, 완료 DI, 실제 개폐 시간은 `gripper_test` 전후 evidence로 확정해야 한다. sim과 실기의 연속/이진 및 동시/분리 차이는 통제 변수로 유지한다.
12. **[미확인·실행 gate]** `pendant_limits_recorded`, live topic/rate, gripper wiring/arm stationary/I/O 결과, `stop_movej_verified`, Servoj parameter/tracking calibration, `stop_servoj_verified`를 순차 evidence로 채워야 한다. 현재 모두 false/null이다. 확장 시 `extended_range_clearance_checked`도 필수다.
13. **[정정]** `matched15_kinematics_reconstructed.json`의 J4/J5/J6 범위, 특히 J5 양수는 IK 해 선택에 의존하는 참고값이다. sim home·시작 branch·관절 사용 범위 근거로 사용하지 않는다. 실제 Isaac 225 sample wrist 범위는 J4 `[0.141,5.926]°`, J5 `[-113.506,-68.638]°`, J6 `[-214.441,-178.500]°`다. **출처:** `reports/home_alignment.md`.
14. **[확인]** 위 Isaac 225 sample을 실기 관절 기준으로 바꾸면 J6 범위는 `[-34.441,1.500]°`이며 J1–J5는 동일하다. 이는 관절 margin 검토의 관측 범위이고 pendant limit을 대체하지 않는다. **출처:** `reports/home_alignment.json`.

상세 수치와 데이터 경로는 `reports/kinematics_validation.md` 및 기계 판독 가능한 `reports/kinematics_validation.json`에 있다.
