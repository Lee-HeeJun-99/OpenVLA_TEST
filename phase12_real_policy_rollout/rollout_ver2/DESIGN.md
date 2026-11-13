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
- **[확인(operator attestation)]** pendant 일반/감소는 동일한 Default: J1 ±360, J2 ±95, J3 ±135, J4 ±360, J5 ±135, J6 ±360°, angle tolerance ±3°다. 사진 미등록은 사용자 결정이며 직접 확인 attestation으로 `pendant_limits_recorded=true`다. **출처:** `operator_attestation.json`; `controller/pendant_limits.json`.
- **[가정 후보]** host margin은 관절별 5°이며 host stop은 J1 ±355, J2 ±90, J3 ±130, J4 ±355, J5 ±130, J6 ±355°다. 위반 사유는 `JOINT_LIMIT_MARGIN:J<n>`이고 clamp하지 않는다. 보호 정지 예상 지점은 pendant 바깥 방향으로 tolerance 3°를 더해 함께 기록한다. **출처:** 작업 지시; `controller/joint_limits.py`.
- **[확인(구현)]** 설정 loader는 host stop이 pendant 안쪽인지, pendant가 두산 설정 가능 최대치(J1/J2/J4/J5/J6 ±360, J3 ±160°) 이내인지, 출처 날짜/evidence 경로가 있는지 검사한다. **출처:** 사용자 제공 매뉴얼 값; `controller/joint_limits.py`.
- **[확인(구현)]** Default가 아닌 경우 `extension_reason`, `pendant_limits_recorded`, `extended_range_clearance_checked`의 실제 evidence 파일이 모두 없으면 실행을 거부한다. **출처:** `controller/joint_limits.py`, `runtime/executor.py`.
- **[확인]** 끝 토큰 연속 감시는 arm 6차원(`Δp,ω`)에만 적용한다. `endpoint_policy∈{log_only,stop}`이며 기본은 `log_only`다. gripper는 endpoint 감시에서 제외한다. **출처:** 작업 지시(2026-11-13); `controller/precheck.py`, `controller/controller_config.json`.
- **[가정]** N의 후보는 **3회**다. 이는 구현 한계나 승인된 안전값이 아니라 운영 검토용 후보이며 실기 전에 확정해야 한다. **출처:** 이번 설계 제안; 단발 양자화 노이즈와 지속 포화 구분 목적.
- **[확인(sim parity)]** matched15 gripper는 raw action[6]의 연속 absolute `[0,1]` target이며 full travel 0.6 s의 slew로 매 physics step 갱신되고 팔 joint target과 같은 step에 병행 적용된다. v2 offline controller도 debounce/binary 변환 없이 이 규약을 구현한다. **출처:** `vla_validation.py:1124-1181`; `controller/gripper.py`.
- **[확인(배선 기록)]** JEGH-3520은 flange X1 24V PNP의 DO1/DO2에 연결된다. 작업자 경험상 OPEN=DO1 On/DO2 Off, CLOSE=DO1 Off/DO2 On이다. 펜던트 양쪽 Off에서 get readback은 둘 다 0이므로 서비스값 `0=Off, 1=On`으로 추정한다. srv 주석과 반대이므로 첫 gripper_test에서 펜던트 표시와 재조회로 확정하며 불일치 시 둘 다 Off 후 중단한다. **출처:** `operator_attestation.json`; `gripper_readback.txt`.
- **[정정·확인(구현)]** 첫 Off write와 같은 service readback은 write/read encoding의 자기일관성만 검사하므로 물리 극성 반전을 검출하지 못한다. 극성 확정은 첫 OPEN의 `DO1←1` 직후 작업자가 펜던트 DO1 On 표시를 직접 확인하는 것으로 한다. `n`이면 즉시 DO1/DO2를 0으로 만들고 `POLARITY_MISMATCH`로 종료한다. **출처:** 사용자 지시(2026-11-13); `tools/gripper_test.py`; `runtime/gripper_io.py`.
- **[확인(구현 기본)]** 기본은 interlocked `LevelIOGripper`다. OPEN은 DO2 Off→DO1 On, CLOSE는 DO1 Off→DO2 On이며 동시 On을 금지하고 최종 DO1/DO2 readback 불일치를 fault 처리한다. 기존 `BinaryIOGripper` pulse는 비기본 대안으로만 남긴다. v1에서 CLOSE pulse ACK 뒤 실제 jaw가 열린 채 남은 사례가 level 유지 방식을 선택한 근거다. **출처:** `runtime/gripper_io.py`; `reports/gripper_control_audit.md`.
- **[통제 변수]** sim은 연속 absolute target, 0.6 s slew, 팔과 동시 동작이지만 실기는 thresholded binary held-level이다. `gripper_test` 동안 팔은 정지하며 policy에서는 `gripper_io_verified` 전까지 계속 기록만 한다. 이 차이는 성능 해석의 통제 변수다. **출처:** matched15 evaluator; `runtime/gripper_io.py`.
- **[확인]** action 회전 규약은 `rotation_frame=world_left`, 즉 world 좌표의 `Δp`와 `R'=Exp(ω)R`이다. sim matched15 evaluator가 `target_position=current_position+action[:3]` 및 `quaternion_multiply(delta,current)`를 사용한다. 기준 pose는 Lula `tool0`의 world pose다. 비교/회귀용 `tool_right` 설정은 유지한다. **출처:** `isaac_sim/a0509_control/vla_validation.py:119-151,1023-1060`; `lula_planner.py`; `reports/controller_replay.md`.
- **[확인]** 프레임을 `tool0_sim=T_link6·[xyz 0,0,0.01,yaw -90°]`, `tool0_real=T_link6·[xyz 0,0,0.01,yaw +90°]`로 구분한다. 실기 명령·피드백·로그는 `tool0_real`, sim 비교는 `tool0_real↔tool0_sim`을 사용한다. **출처:** sim FK 검증; 현장 육안 확인(2026-11-13); `controller/geometry.py`.
- **[확인]** offline controller에는 파일 전용 `DryRunPublisher`가 있고, 별도 runtime 계층에는 기본 비활성 `ServojPublisher`가 있다. 후자는 explicit enable, 모드 gate, 확인된 time/vel/acc가 모두 없으면 발행을 거부한다. `.msg` 정의대로 pos/vel/acc는 각각 6원소이며 pos는 deg, mode=0, QoS depth 1이다. **출처:** `controller/publisher.py`; `runtime/servoj.py`; 읽기 전용 `dsr_msgs2/msg/ServojStream.msg`.
- **[확인(구현)]** 관절 한계는 `pendant_limits.json`에서 로드하며 operator attestation으로 기록됐다. pendant 속도는 J1–J3 180°/s, J4–J6 360°/s, tolerance 10°/s다. host 속도/가속도/workspace 후보는 pendant 값을 대체하지 않는다. **출처:** `operator_attestation.json`; `controller/pendant_limits.json`.
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
- **[확인]** 100 Hz 유휴 더미 루프 jitter는 p50 10.00 ms, p99 11.92 ms, max 12.78 ms다. 프로세스 분리 후 추론 동시 dryrun 60.00 s의 p99는 10.904 ms이고 stale/fault 없이 통과했다. **출처:** 작업 지시의 유휴 측정; `reports/live/20261113_091859/dryrun_live/dryrun_live_summary.json`.
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
- **[확인(현장)]** controller_manager 실제 update rate는 100 Hz다. **출처:** `readonly.txt`; `feedback_selftest.md`.
- **[확인]** controller `192.168.0.110:12345`, `rt_host=192.168.137.50`; RT 제어는 사용하지 않는다. **출처:** 작업 지시의 설정 조사.
- **[확인]** `servoj_stream` subscriber QoS depth 20, 기본 reliable이며 callback은 별도 검사 없이 `Drfl->servoj(pos,vel,acc,time,mode)`를 호출한다. **출처:** 작업 지시의 driver source 조사.
- **[확인(source)]** 드라이버는 `motion/move_stop`(`dsr_msgs2/srv/MoveStop`)과 `motion/move_joint`(`dsr_msgs2/srv/MoveJoint`) 서비스를 만든다. namespace를 적용한 설정 이름은 `/dsr01/motion/move_stop`, `/dsr01/motion/move_joint`이며 현장 graph에서 전체 이름을 재확인한다. MoveStop mode는 0 QSTOP_STO, 1 QSTOP, 2 SLOW, 3 HOLD다. **출처:** 읽기 전용 `dsr_controller2.cpp:745-748,2407,2420`; `MoveStop.srv:6-11`; `MoveJoint.srv:6-15`.
- **[확인(source only)]** `RobotState.msg`에는 posj/posx/world pose/brake 필드가 있으나 현장 graph에 해당 topic이 없어 runtime 입력으로 사용하지 않는다. **출처:** message 정의; `readonly.txt`.
- **[확인·정정]** robot_state의 단일 정의는 0 INITIALIZING, 1 STANDBY, 2 MOVING, 3 SAFE_OFF, 4 TEACHING, 5 SAFE_STOP, 6 EMERGENCY_STOP, 7 HOMMING, 8 RECOVERY, 9 SAFE_STOP2, 10 SAFE_OFF2, 11–14 RESERVED, 15 NOT_READY다. `RobotState.msg` 주석은 7 중복으로 이후 번호가 밀린 오타이므로 사용하지 않는다. **출처:** `dsr_common2/include/DRFC.h:131-149`; `dsr_common2/imp/DRFC.py:76-91`; `dsr_msgs2/srv/system/GetRobotState.srv:6-21`; `runtime/robot_state.py`.
- **[확인]** servo 중에도 robot_state가 주로 1이므로 이를 정지 증거로 쓰지 않는다. **출처:** 작업 지시의 실기 로그 분석.
- **[확인]** Ikin/Fkin 서비스 정의가 존재한다. **출처:** 작업 지시의 source 조사.
- **[확인]** 실기 JointState/error/disconnection topic과 get 서비스 이름/QoS/주기를 확인했다. controller software는 통합 GV02120200, 메인 GF02120200, 안전 GD12120200, 인버터 GA33030300이다. **[미확인]** Ikin/Fkin `.srv` 경로. **출처:** `readonly.txt`, `readonly_services.txt`, `feedback_selftest.md`, `operator_attestation.json`.

## 4. 네트워크

- **[확인(현장)]** 직접 연결 NIC는 `enp6s0`, host `192.168.0.100`, controller `192.168.0.110`이며 ping 10회 손실 0%, 평균 0.229 ms다. Wi-Fi는 `192.168.1.98`이다. **출처:** `reports/live/20261113_091859/readonly.txt`.

## 5. 로봇 안전 설정

- **[확인(operator attestation)]** Robot Limits 각도/속도, TCP 힘 380N, 기계 동력 1000W, TCP 속력 4000mm/s, 운동량 40kg·m/s, 충돌 민감도 50%를 기록했다. **출처:** `operator_attestation.json`; `controller/pendant_limits.json`.
- **[확인·정정]** 과거 J5 약 -108°→-138° 뒤 `robot_abnormal_state=10`이 기록됐으며 확정 enum상 **SAFE_OFF2**다(SAFE_STOP2가 아님). **출처:** v1 실기 기록; 확정 `runtime/robot_state.py`.

## 6. 모델 / 추론 서버

- **[확인]** `vanilla_s1_balanced_step8130`은 OpenVLA-7B + LoRA r32, `openvla_token` 256 bins, K=1, action_dim=7, binary gripper, proprio 없음. **출처:** 작업 지시의 model/config 조사.
- **[확인]** 학습 데이터는 `openvla_a0509_legacy_midheight_corrective_speed1p712696_native5hz_300_v1`, key `a0509_sim_cube_pick`, train 12998/val 1604, trajectories 270, corrective_fraction 0.544, image augmentation 사용, sim-only다. **출처:** 작업 지시의 manifest/statistics 조사.
- **[확인]** q01/q99를 `[-1,1]`로 정규화한다. 통계 원본은 `models/vanilla_s1_balanced_step8130/dataset_statistics.json`. **출처:** 작업 지시.
- **[확인]** bin center 최대는 `1-1/255`; 끝 토큰 역정규화 값을 계산할 수 있다(예: rz max 0.064914, gripper min 0.001961). **출처:** 작업 지시의 tokenizer/statistics 계산.
- **[확인]** matched15 sim 평가 15 episode 성공률은 1.0. **출처:** 작업 지시의 평가 결과.
- **[확인]** `scripts/serve.sh vanilla <port>` → `runtime/vanilla-support/serve_adapter.py`; 별도 venv `environment/a6000_ubuntu22_py310`; JPEG base64와 instruction을 받는 HTTP `/predict`; port 8766 사용 이력이 있다. **출처:** 작업 지시의 server 조사.
- **[확인]** matched15 경로의 client는 Isaac camera RGB uint8을 JPEG quality 92로 보내며, server는 bottom crop 0.0, center crop 없음, processor `resize-naive` 224×224를 사용한다. **출처:** `isaac_sim/a0509_control/vla_client.py:45-78`; `runtime/vanilla-support/serve_adapter.py:108-176`; model `training_config.json`, `processor/preprocessor_config.json`; matched15 evidence의 `server.crop_bottom_fraction=0.0`.
- **[확인]** 기존 실기 rollout client는 RGB이지만 JPEG quality 95를 사용해 matched15의 quality 92와 다르다. vanilla server는 center crop을 추가하지 않는다. v2 `controller/observation.py`는 RGB→JPEG q92/optimize false→server RGB decode→resize-naive dual-backbone tensor를 재현하며 reference와 bit-identical tensor를 확인했다. **출처:** 읽기 전용 `../14_real_rollout_integration/rollout_latency.py`; `controller/test_observation.py`.
- **[확인(현장 측정·구현)]** ZED rect image는 1280×720 `bgra8`, 30 Hz, publisher RELIABLE/VOLATILE다. 20초 비교에서 sensor-data BEST_EFFORT는 14.94 Hz/100 ms 초과 52회, RELIABLE depth 1 VOLATILE는 19.98 Hz/31회여서 후자를 채택했다. 수정 후 최소 callback 재측정은 22.84 Hz, p50/p99/max 33.35/166.57/238.50 ms, 0.1 s 초과 40회, 1 s 초과 0회였다. 카메라는 전용 executor thread에서 spin하고 callback은 최신 raw message와 두 시각만 보관하며 RGB 변환은 관측 시 수행한다. **출처:** `reports/live/20261113_091859/camera_qos_diag.md`; `runtime/camera.py`; `tools/dryrun_live.py`.
- **[확인(사용자 결정·구현)]** 모드 A 관측은 정착 시각 `t_settle` 뒤에 host에서 수신되고 camera `header.stamp`도 `t_settle`의 ROS 시각 뒤인 프레임만 사용한다. 이미지가 늦으면 로봇은 정착 위치에서 새 명령 없이 기다리며 직전 action을 반복하지 않는다. joint feedback/state/error/disconnection 안전 감시는 계속하고 이상은 기존 정지 경로로 처리한다. 10 s 대기 상한은 **후보**이며 초과는 정지 상태의 정상 episode 종료 `CAMERA_NO_FRAME_TIMEOUT`이지 fault가 아니다. 연속 수신 간격과 공백은 log-only이고 camera age 0.1 s 연속 fault는 사용하지 않는다. 모든 움직임이 모델 출력에만 대응해야 sim-to-real 결과를 해석할 수 있기 때문이다. **출처:** 사용자 결정(2026-11-13); `runtime/camera.py`; `tools/dryrun_live.py`.
- **[확인(시계 대응)]** 정착 순간 host `time.monotonic()`과 ROS node clock nanoseconds를 한 쌍으로 기록한다. callback은 같은 frame의 host 수신 monotonic과 `header.stamp`를 기록하며, 각 시계 영역에서 독립적으로 `frame_host>settle_host`와 `frame_header>settle_ros`를 검사한다. 서로 다른 epoch를 직접 빼지 않는다. 관측 로그에는 네 시각과 host 기준 대기 시간을 보존한다. **출처:** `runtime/camera.py`; `tools/dryrun_live.py`.
- **[확인]** matched15 지시문은 layout target에 따라 정확히 `Pick up the red/yellow/blue cube.`였고 15 episode에 반복됐다. **출처:** `validation/evidence/vanilla/success_profile_s1_balanced_step8130_matched15_k1_seed9406_15ep.json`의 results.
- **[확인(실험 조건 차이)]** v2 기본/이번 세션 지시문은 `Pick up the blue cube.`이며 matched15의 red/yellow/blue 정확 문자열만 기본 허용한다. 집합 밖은 explicit `--allow-ood-instruction`과 로그 기록이 필요하다. v1은 집합 밖 `Pick up the orange cube.`를 사용했으므로 결과 비교 시 OOD instruction 차이를 통제한다. **출처:** v1 실행 설정; 사용자 결정(2026-11-13); `tools/dryrun_live.py`.
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

- **[확인(현장·구현)]** RobotState topic은 없다. `RobotFeedback`는 RELIABLE+TRANSIENT_LOCAL `/dsr01/joint_states`를 이름으로 J1–J6 정규화하고, 최초 cached sample을 freshness에서 제외하며 host monotonic과 `header.stamp`를 모두 기록한다. 이름 누락/중복, `/error`, `/robot_disconnection`은 즉시 fault다. state는 `get_robot_state` 10Hz 후보로 polling하며 실패/지연을 기록하고 stale 처리한다. **출처:** `reports/live/20261113_091859/readonly.txt`; `runtime/feedback.py`; `feedback_selftest.md`.
- **[확인(현장)]** `/dsr01/joint_states.name`의 실제 순서는 `[joint_1,joint_2,joint_4,joint_5,joint_3,joint_6]`이며 배열 위치를 신뢰하지 않고 이름으로 J1–J6에 매핑한다. **출처:** `readonly.txt`; `feedback_selftest.md`.
- **[확인(현장)]** 30초 read-only self-test에서 JointState 3,000개, host period mean 9.9999ms, state poll 301/301 성공, latency p50/p99/max 0.954/1.943/3.292ms였다. 이름 정규화 후 get_current_posj와 최대 차이는 `6.53e-9°`다. **출처:** `reports/live/20261113_091859/feedback_selftest.json`.
- **[확인(구현·현장 재검증)]** joint_states/error/disconnection과 get_* timer/future는 feedback 전용 node의 전용 executor thread에서 처리한다. main thread의 image 변환·HTTP·frame wait와 독립이다. 감시 시작 전 시작 시각 이후 joint_states와 robot_state 응답을 각각 1개 이상 기다리며 2 s 후보 초과는 `FEEDBACK_NO_FIRST_SAMPLE`이다. 0.45 s main blocking을 반복한 30.19 s read-only 점검에서 joint age p50/p99/max는 5.24/10.10/10.62 ms, state poll은 303/303 성공했다. **출처:** `runtime/feedback.py`; `tools/feedback_thread_check.py`; `reports/live/20261113_091859/feedback_thread_check.md`.
- **[확인(구현)]** `JOINT_STATE_STALE` 기준은 0.1 s로 유지한다. 발생 로그에는 직전 수신 간격, 마지막 host monotonic 수신 시각, 누적 joint message 수, state poll 수를 기록한다. thread 간 joint/state/fault/진단 공유 상태는 lock/condition으로 보호한다. **출처:** `runtime/feedback.py`; `tools/dryrun_live.py`.
- **[확인(현장 원인 범위)]** thread 분리 dryrun에서 50.35 s 시점에 joint age 109.93 ms stale이 1회 발생했다. 마지막 정상 joint 간격은 9.91 ms이고 총 5,013개로 발행률은 정상이다. 공백 `136141.3213–136141.4312`는 step 99의 frame 수신 `136141.2582`부터 prediction 완료 `136141.6788`까지 420.60 ms 처리 창 안에 완전히 포함된다. 기존 로그는 JPEG/HTTP 단계별 시작을 남기지 않아 특정 단계의 인과는 확정하지 않고, 같은 Python process의 image/JPEG/HTTP 부하와 겹친다는 것까지만 확인한다. **출처:** `dryrun_live_summary.json`; `predictions.jsonl`; `reports/live/20261113_091859/process_stall_analysis.md`.
- **[확인(구현)]** dryrun과 향후 runtime 경계는 OS process로 분리한다. 프로세스 A는 robot feedback/state/safety와 향후 command/stop 소유자이며 image를 import/처리하지 않는다. 프로세스 B는 camera, post-settle frame barrier, JPEG q92, HTTP inference만 소유한다. A→B 요청은 두 `t_settle` 시각을 포함하고 B→A 응답은 frame timing, 단계별 processing timing, raw action을 포함한다. B 사망은 A에서 `OBSERVATION_PROCESS_DIED_STOP_REQUIRED`로 검출해 향후 기존 stop 경로로 연결하며, B는 parent watchdog으로 A 사망을 검출하고 종료한다. `JOINT_STATE_STALE=0.1 s`는 유지한다. **출처:** `runtime/observation_process.py`; `tools/dryrun_live.py`.
- **[확인(교차 검증·정지 아님)]** 2Hz get flange posx/rotm 60쌍에서 FK(link_6) 대비 위치 1.0164mm, 회전 0.08427°의 일정 오차를 관찰했다. 이는 log-only이며 정지 조건이 아니다. **출처:** `reports/live/20261113_091859/feedback_selftest.md`.
- **[가정 후보]** 정지 속도 판정은 `max|qdot|≤0.001 rad/s`다. 2026-11-13 정지 self-test 3,000 sample의 18,000개 관절 속도 값은 모두 0.0 rad/s였으며 후보는 이 관측 노이즈보다 큰 비영 threshold다. 안전 한계가 아니며 현장 검토 대상이다. **출처:** `feedback_selftest.json`; `runtime/runtime_config.json`.
- **[가정 후보]** 매 sample 감시값은 추종 위치 10 mm/회전 3°, zero-velocity 유지 0.3 s다. 승인 한계가 아니며 첫 세션 evidence로 보정 전 policy 실행에 사용할 수 없다. **출처:** 작업 지시; `runtime/monitor.py`, `runtime/stop.py`.
- **[확인(현장 시험 2026-11-13)]** MoveStop mode는 `DR_QSTOP=1`, MoveJoint acceleration은 `10°/s²`다. `stop_test_movej` 통과 결과를 사용자 review하여 `stop_movej_verified=true`로 등록했다. 향후 `stop_test_servoj`도 같은 stop mode를 사용한다. **출처:** `reports/live/20261113_091859/stop_test_movej/summary.json`; 사용자 결정; `runtime/runtime_config.json`.
- **[확인(구현)]** 정지 순서는 command publisher inhibit → 설정된 MoveStop 요청 → 계속 수신한 JointState의 zero-velocity 구간 확인이다. ACK 실패는 SAFE_STOP/OFF 계열이 먼저 관찰된 경우만 허용하며, 확인 실패 뒤 추가 명령은 영구 금지한다. **출처:** `runtime/stop.py`와 mock tests.
- **[확인(구현)]** 정지 시험은 세션 1의 `stop_test_movej`와 세션 2의 `stop_test_servoj`로 분리한다. 전자는 real home에서 J6 +2°, 5°/s 이하 MoveJoint 중 정지 서비스·판정 로직을 검증하며 Servoj 중 정지를 대신하지 않는다. 후자는 Servoj parameter 확인 뒤 같은 stop mode로 별도 검증한다. synthetic/policy는 `stop_servoj_verified`를 필수로 한다. **출처:** `runtime/gate.py`, `runtime/executor.py`, `docs/first_live_session.md`.
- **[확인(구현·미실행)]** `tools/gripper_test.py`와 `tools/stop_test_movej.py`는 operator UI와 ROS control/safety를 spawn process로 분리한다. setter/MoveJoint/MoveStop 객체는 gate, real-home, 2초 정지 측정과 fresh confirmation 뒤에만 만든다. 결과는 candidate gate update로만 표시하며 `gates.json`을 자동 변경하지 않는다. 이번 작업에서는 mock test만 수행했다. **출처:** 해당 도구와 `runtime/test_runtime.py`.
- **[확인(구현)]** gate evidence는 `photo`, `operator_attestation`, `measurement`이며 type을 기록한다. live topic과 feedback rate는 measurement evidence를 사용한다. 이 근거로 `pendant_limits_recorded`와 `gripper_wiring_recorded`는 true지만 `gripper_io_verified`는 실제 시험 전까지 false다. **출처:** `runtime/gate.py`, `runtime/gates.json`, `operator_attestation.json`.
- **[확인(구현)]** 모든 물리 mode는 fresh human confirmation, explicit physical enable, mode별 evidence gate와 non-null 필수 설정을 요구한다. dryrun_live/gripper_test/stop_test_movej에는 Servoj time/vel/acc와 tracking threshold가 필요하지 않다. `live_topics_verified`와 `feedback_rate_verified`는 2026-11-13 read-only/self-test evidence로 true이며 나머지 현장 gate는 닫혀 있다. **출처:** `runtime/gate.py`, `runtime/executor.py`, `runtime/gates.json`.
- **[확인(결정·오프라인 구현 2026-11-13)]** `policy_movej`는 mode A의 별도 실행 계층이다. 프로세스 A가 feedback/safety와 MoveJoint/MoveStop만 소유하고 프로세스 B가 image/JPEG/inference를 소유한다. 명령 자세 기준 `world_left`, `tool0_real` 목표를 IK한 뒤 action을 clamp하지 않고 전체 거부/수락한다. 목표까지 관절 직선 20점의 margin/`σmin≥0.0803`와 관절 변화 10°를 사전 검사하고, MoveJoint 5°/s·10°/s² 뒤 0.001 rad/s 및 0.1°를 0.3 s 연속 확인한다. gripper는 record-only, sim envelope은 log-only, 자동 home 복귀는 없다. Servoj `policy`와 그 gate는 그대로 유지한다. **출처:** `runtime/policy_movej_core.py`; `tools/policy_movej.py`; 사용자 결정.
- **[확인(사용자 결정 2026-11-13)]** `policy_movej`의 translation/rotation 행동 크기 상한과 `TRANSLATION_STEP_CANDIDATE_EXCEEDED`/`ROTATION_STEP_CANDIDATE_EXCEEDED` 검사를 제거했다. 설정값을 늘려 우회한 것이 아니라 이 실행 경로에서 검사하지 않는다. 근거는 token 모델 출력이 학습 통계 q01~q99로 유계라는 사용자 결정이다. action은 NaN/inf만 거부하고 크기는 매 step에 mm/deg norm으로 기록한다. 관절 변화 10°, 관절 한계 margin, IK, 20점 경로 margin/특이점, workspace, feedback/state, 정지 경로는 변경하지 않았다. **출처:** `runtime/policy_movej_core.py`; `tools/policy_movej.py`.
- **[확인(사용자 결정·오프라인 구현 2026-11-13)]** 전체 `policy_movej` rollout 예산은 기본 300 step/600 s 후보이며 CLI 조정 가능하다. `σmin=0.0803`은 유지하고 낮추려면 값과 사유를 명시·기록해야 한다. table 높이 자동 검사는 없고 실행 전 `TABLE_WATCH_BY_OPERATOR` 확인과 지속 작업자 감시를 요구한다. 선택적 `--max-descent-mm`만 real-home 대비 tool0_real 목표 및 20점 경로 하강을 `DESCENT_LIMIT`으로 거부하며 기본 비활성이다. Enter/q는 이동 중에도 MoveStop을 요청하고 응답시간을 기록한다. summary는 종료 사유, step/시간, 누적 변위, 최저 높이, envelope 최초 이탈과 arm endpoint를 포함한다. **출처:** `tools/policy_movej.py`; `runtime/policy_movej_core.py`; `runtime/runtime_config.json`.
- **[확인(사용자 결정·오프라인 검증 2026-11-13)]** `policy_movej.max_joint_delta_deg_candidate`는 5°에서 **10°**로 변경했다. 근거는 home 부근 12 mm 수직 action이 J3 약 5.5–6.4°를 요구하기 때문이다. MoveJoint 5°/s 조건에서는 10° 관절 step이 약 2 s다. matched15 성공 15 episode의 연속 branch pose 전이 1,038개(6,228 joint 성분; home seed→첫 관측 정렬 제외)에서 |Δq| 전체 p99/max는 **3.3825°/5.0182°**였다. 관절별 p99/max는 J1 0.9916/1.1747°, J2 2.2812/2.4773°, J3 3.7015/5.0182°, J4 0.4906/1.5447°, J5 3.4924/4.0026°, J6 3.5178/3.7818°다. 따라서 10°는 관측 최대보다 작지 않고 약 1.99배 크다. Cartesian action 크기 상한은 후속 사용자 결정으로 제거됐지만 joint margin, singularity, 20점 경로 검사는 그대로다. **출처:** `reports/sim_reference/matched15_reference.json`; `runtime/runtime_config.json`.
- **[확인(현장)]** pendant TCP `Tool_v1` offset은 `[0,0,0 mm,0,0,0°]`라 posx=flange다. payload의 `get_current_tool` 응답은 빈 문자열이며 v1 동일 조건 유지를 위해 변경하지 않는다. controller software는 통합 `GV02120200`, 메인 `GF02120200`, 안전 `GD12120200`, 인버터 `GA33030300`이다. 펜던트 Tool_v1의 Controller Digital 출력1/입력1 항목은 실제 X1 배선과 무관한 legacy 항목이다. **출처:** `operator_attestation.json`; 사용자 결정(2026-11-13).
- **[미확인]** Servoj time/vel/acc, tracking calibration, stop_test_servoj와 그리퍼 실제 I/O 결과가 남았다. MoveJoint acceleration 10°/s²와 MoveStop mode 1은 현장 시험으로 확인됐다. **출처:** `runtime/runtime_config.json`; `runtime/gates.json`; `docs/first_live_session.md`.

## 10. ServoJ 방식 A 계획(오프라인 구현, 실기 미검증)

- **[확인(설계·오프라인 구현 2026-11-13)]** 행동 하나는 명령 `tool0_real` 자세에 `world_left` action을 누적한 목표로 정의한다. 고정 T 후보 0.4/0.6/1.0 s에서 위치 선형·회전 slerp를 minimum-jerk 시간척도로 100 Hz 표본화한다. 전 표본 IK는 직전 해 seed로 발행 전에 계산하며 pendant-5° margin, 표본 관절 변화, 후보 속도/가속도, `σmin` 및 workspace 중 하나라도 실패하면 action을 수정하지 않고 구간 전체를 거부한다. **출처:** `runtime/servoj/planner.py`; 사용자 작업 지시.
- **[확인(오프라인 구조)]** 전송은 제어/안전 프로세스 A의 전용 absolute-deadline thread만 담당한다. 프로세스 B는 정착 이후 frame/JPEG/inference만 담당한다. 전송 중 command-vs-JointState 관절 오차, 0.1 s freshness, robot state `{1,2}`, error/disconnection과 joint margin을 매 표본 감시하고 이상 시 publisher inhibit 후 기존 MoveStop/zero-speed 경로로 들어간다. 마지막 목표 뒤 `|qdot|≤0.001 rad/s` 0.3 s 및 도착 오차 후보를 만족한 후에만 다음 관측을 요청한다. **출처:** `runtime/servoj/stream.py`; `runtime/servoj/tracking.py`; `runtime/servoj/mode_a.py`.
- **[확인(텔레메트리 계획)]** ServoJ step은 기존 policy_movej 필드에 `servoj_sent_count`, send jitter p50/p99/max, 최대 관절 추종오차, `segment_T_s`, IK precheck/move/settle/total 시간을 추가하고 system monitor를 자동 시작한다. 출력은 `reports/live/<session>/policy_servoj/<timestamp>/`다. **출처:** `docs/servoj_session.md`.
- **[확인(오프라인 재생)]** 20261113_140038의 222 action을 strict-primary/fallback DLS로 재생한 결과 T=0.4/0.6/1.0 s 모두 IK·margin·sigma·속도·가속도 위반 0개였다. 최대 후보 한계 사용률은 T별 속도 8.96/5.98/3.59%, 가속도 73.47/32.69/11.78%다. 이는 실제 controller 추종 성능을 확인한 결과가 아니다. **출처:** `reports/servoj_offline/servoj_offline.md`; `analysis.json`.
- **[미확인]** `ServojStream.time`의 controller 내 정확한 의미, 승인할 vel/acc/time 값, stream 전송 중단 시 마지막 목표 유지/감속 거동, 실제 100 Hz jitter와 추종 지연·overshoot는 현장 hold→time 탐색→stop 순으로 확인해야 한다. `servoj_time_param_verified`, `tracking_thresholds_calibrated`, `arm_stationary_verified`, `stop_servoj_verified`는 계속 false이며 추론으로 열지 않는다. **출처:** `runtime/gates.json`; `docs/servoj_session.md`.
- **[확인(안전 상태)]** 이번 산출물의 field tool은 ROS 명령 객체를 생성하지 않는 fail-closed 초안이다. 현장 adapter review와 해당 gate evidence 전에는 `FIELD_ADAPTER_REVIEW_REQUIRED`로 끝난다. MoveJ 경로와 테스트는 변경하지 않았다. **출처:** `tools/servoj_*.py`; `tools/policy_servoj.py`; `runtime/servoj/session_contract.py`.

## 11. 열린 항목(우선순위)

1. **[확인·종료]** sim world ↔ robot base 변환: 현재 Isaac runtime/scene 및 `base_link` 기준 `R_world_base≈I`; translation은 약 `[-0.000389,-0.000257,0.823526] m`. 다른 scene/base-yaw 인자에는 재적용할 수 없으므로 scene별 metadata 보존이 필요하다.
2. **[부분 종료]** matched15 sim 적용 규약과 평가 지시문은 확인했다. legacy 학습 dataset 자체의 instruction/action 생성 provenance는 학습 server `/mnt/SSD1` manifest 확인이 여전히 필요하다.
3. **[확인·종료]** `link_6→tool0_sim`은 xyz `[0,0,0.01] m`, yaw `-90°`; 현장 장착 보정을 포함한 `link_6→tool0_real`은 xyz `[0,0,0.01] m`, yaw `+90°`다. pendant TCP 설정값 자체는 별도로 기록한다.
4. **[미확인]** URDF와 실기 flange 일치: paired flange 필드가 없어 posx fallback만 검증했고 1 mm/0.1° 목표를 완전 통과하지 못함.
5. **[확인·종료(attestation)]** pendant angle/velocity와 기타 Robot Limits, Tool_v1 zero offset, payload 미등록 유지 조건, controller software version을 operator attestation으로 기록했다. 사진은 사용자 결정으로 미등록이다.
6. **[확인·종료]** 실기 topic/QoS/100 Hz feedback와 유선 ping을 read-only evidence로 기록했다.
7. **[확인·종료(matched15)]** center crop 없음, bottom crop 0.0, JPEG quality 92, RGB. legacy 학습 원본 instruction은 항목 2에 남긴다.
8. **[확인·종료]** 프로세스 분리 후 inference 동시 100 Hz loop p99 10.904 ms, 60 s 통과.
9. **[확인·종료]** endpoint 감시는 arm 6차원만 사용하고 gripper는 상태 변화만 기록한다.
10. **[확인·종료(offline 분류)]** 새 σmin 0.0803 후보로 190404 replay를 재실행하면 world_left는 step 72/sample 12의 σmin 0.080275에서, tool_right는 step 36/sample 25의 J5 -130.059°에서 먼저 정지한다. **출처:** `reports/controller_replay.json`, `reports/matched15_branch_and_ik_timing.md`.
11. **[부분 종료·현장 확인 필요]** JEGH-3520의 기존 Tool DO 명령 경로와 OPEN 실물 관찰은 확인했다. CLOSE 물리 동작, 포트/극성, 완료 DI, 실제 개폐 시간은 `gripper_test` 전후 evidence로 확정해야 한다. sim과 실기의 연속/이진 및 동시/분리 차이는 통제 변수로 유지한다.
12. **[부분 종료·실행 gate]** `pendant_limits_recorded`, `gripper_wiring_recorded`, `live_topics_verified`, `feedback_rate_verified`, `stop_movej_verified`는 2026-11-13 evidence/review로 true다. arm stationary/I/O 결과, Servoj parameter/tracking calibration, `stop_servoj_verified`는 아직 false다. `gripper_io_verified=false`이므로 policy_movej는 gripper를 기록만 한다. 확장 시 `extended_range_clearance_checked`도 필수다.
13. **[정정]** `matched15_kinematics_reconstructed.json`의 J4/J5/J6 범위, 특히 J5 양수는 IK 해 선택에 의존하는 참고값이다. sim home·시작 branch·관절 사용 범위 근거로 사용하지 않는다. 실제 Isaac 225 sample wrist 범위는 J4 `[0.141,5.926]°`, J5 `[-113.506,-68.638]°`, J6 `[-214.441,-178.500]°`다. **출처:** `reports/home_alignment.md`.
14. **[확인]** 위 Isaac 225 sample을 실기 관절 기준으로 바꾸면 J6 범위는 `[-34.441,1.500]°`이며 J1–J5는 동일하다. 이는 관절 margin 검토의 관측 범위이고 pendant limit을 대체하지 않는다. **출처:** `reports/home_alignment.json`.

상세 수치와 데이터 경로는 `reports/kinematics_validation.md` 및 기계 판독 가능한 `reports/kinematics_validation.json`에 있다.
