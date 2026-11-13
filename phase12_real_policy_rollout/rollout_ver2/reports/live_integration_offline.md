# V2 실기 연동 계층 offline 구현 보고

## 안전 상태

실기/드라이버/ROS graph에 접속하지 않았다. 모든 command-capable class는 기본 disabled이고, mock 단위 테스트만 실행했다. `runtime/run_v2.py`도 gate·사람 확인 뒤 실제 adapter가 결합되지 않으면 fail-closed한다.

## 구현

| 구성 | 계약 |
|---|---|
| ServojPublisher | `/dsr01/servoj_stream`, `ServojStream`, pos/vel/acc 각 6원소, pos deg, mode 0, QoS depth 1. enabled flag + policy gate. time/vel/acc null이면 거부 |
| RobotFeedback | 이름 기반 J1–J6 JointState, host/header 시각, get_robot_state 10Hz polling, mode/system preflight, error/disconnection fault, flange get 교차 검증. RobotState topic에는 의존하지 않음 |
| RuntimeMonitor | 매 sample tracking 10 mm/3° 후보, stale, state not in {1,2}, joint margin, σmin 감시 |
| StopCoordinator | publisher inhibit → 설정의 MoveStop `DR_QSTOP=1` 후보 → ACK와 계속 갱신되는 state/velocity 기록 → zero velocity 0.3 s 후보. ACK 실패는 SAFE_STOP/OFF 계열 state일 때만 허용. 실패 시 영구 inhibit |
| LevelIOGripper (기본) | opposite Off 후 OPEN DO1 On/CLOSE DO2 On level 유지, 양쪽 readback, 동시 On 금지. Binary pulse는 비기본 대안. policy는 record-only |
| CameraFeedback / ObservationRuntime | read-only Image를 RGB로 변환하고 host monotonic age 감시; sim-aligned JPEG q92 저장, `/predict` raw action/arm endpoint 기록 |
| FeedbackJsonlLogger | 실기 FK 자세를 `frame=tool0_real`, position, rotation matrix로 기록; sim 비교 시 `tool0_sim`과 대응 |
| ExecutionGate | 선행/결과를 분리한 evidence-path gate와 모드별 requirement |

## 모드별 gate

| 모드 | 필수 gate |
|---|---|
| dryrun_offline | 없음 |
| dryrun_live | command gate 없음; read-only 확인 및 사람 입력은 필요 |
| gripper_test | pendant_limits_recorded, live_topics_verified, gripper_wiring_recorded, arm_stationary_verified |
| stop_test_movej | pendant_limits_recorded, live_topics_verified, feedback_rate_verified |
| stop_test_servoj | 위 movej 결과 + servoj_time_param_verified, tracking_thresholds_calibrated |
| synthetic/policy | Servoj 필수 항목 + stop_servoj_verified |

모든 gate는 `passed=true`, 존재하는 evidence와 `photo`/`operator_attestation` type이 필요하다. `pendant_limits_recorded`, `gripper_wiring_recorded`, `live_topics_verified`, `feedback_rate_verified`가 true이고 나머지는 false다.

## matched15 IK 복원값 — 참고 전용 정정

아래 복원값은 IK branch에 의존하므로 home, wrist 구성 또는 관절 사용 범위의 기준으로 사용하지 않는다. 실제 기준은 `reports/home_alignment.md`의 Isaac `actual_joint_position_rad` 225개다.

## 과거 복원 결과(참고 전용)

matched15 evidence는 joint target을 저장하지 않았다. 따라서 아래는 logged current tool0 pose 1,053개를 legacy home seed와 bounded IK로 복원한 **[가정: reconstructed]** 값이다. 직접 joint telemetry가 아니며 live threshold로 승격할 수 없다. 복원 FK 최대 잔차는 위치 `1.16e-11 m`, 회전 `1.51e-10°`다.

| σmin | 값 |
|---|---:|
| p0 | 0.101366 |
| p1 | 0.104164 |
| p5 | 0.110257 |
| p50 | 0.156781 |

성공 복원 sample 최소의 80%는 0.08109였으나 이 wrong-branch 복원 근거는 폐기했다. 최신 후보는 sim-home seed 연속 branch matched15와 Isaac225를 함께 사용한 **0.0803**이며 `reports/matched15_branch_and_ik_timing.md`가 기준이다.

| joint | min° | max° |
|---|---:|---:|
| J1 | -17.892 | 22.428 |
| J2 | -20.936 | 40.987 |
| J3 | 37.854 | 89.027 |
| J4 | -3.397 | 5.771 |
| J5 | 73.921 | 106.939 |
| J6 | -108.847 | 1.540 |

기계 판독 결과는 `matched15_kinematics_reconstructed.json`에 있다. pendant limits와 비교할 때는 이 범위를 승인 한계로 쓰지 않는다.

## 오프라인 검증 결과

- `/usr/bin/python3 -m unittest discover -s . -p 'test_*.py' -v`: 최신 전체 결과는 `reports/joint_limits_analysis.md` 및 최종 작업 보고를 따른다. ROS 발행은 in-memory mock에서만 수행한다.
- `/usr/bin/python3 -m compileall -q controller kinematics runtime`: 통과.
- `bash -n tools/live_readonly_check.sh`: 통과. 스크립트에는 `ros2 topic pub`, `ros2 service call`, action send가 없다.
- 실기, ROS graph, controller IP에는 접속하지 않았다.

## 이번 작업 생성/변경 파일

| 구분 | 파일 |
|---|---|
| 설계/설정 | `DESIGN.md`, `controller/controller_config.json`, `runtime/runtime_config.json`, `runtime/gates.json` |
| runtime | `runtime/__init__.py`, `camera.py`, `config.py`, `executor.py`, `feedback.py`, `gate.py`, `gripper_io.py`, `http_client.py`, `monitor.py`, `observation_runtime.py`, `ros_transports.py`, `run_v2.py`, `servoj.py`, `stop.py` |
| 테스트 | `runtime/test_runtime.py` |
| 분석 | `controller/analyze_matched15_kinematics.py`, `reports/matched15_kinematics_reconstructed.json` |
| 조사/보고 | `reports/gripper_control_audit.md`, `reports/live_integration_offline.md` |
| 세션 준비 | `tools/live_readonly_check.sh`, `docs/first_live_session.md` |
