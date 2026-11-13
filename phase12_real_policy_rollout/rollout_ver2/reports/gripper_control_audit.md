# 기존 JEGH-3520 그리퍼 제어 조사

조사 범위는 읽기 전용 `../14_real_rollout_integration`, 번들 전체, `~/robot_ws/src`, `~/A0509_OFT_selected_training_kit_20260805`, 홈 아래 Python/shell/JSON 로그이며 `gripper`, `grip`, `digital_output`, `tool_do`, `JEGH`, `JRT`, `open`, `close`를 교차 검색했다.

## 결론

가장 최근 실기 사용 기본 후보는 **Tool/flange digital output의 operator-timed pulse**다. ROS 서비스는 `/dsr01/io/set_tool_digital_output`, 타입은 `dsr_msgs2/srv/SetToolDigitalOutput`이다. Control-box DO가 아니다.

| 방식 | OPEN | CLOSE | 유지/대기 | 완료 확인 | 실기 근거 |
|---|---|---|---|---|---|
| **기본 후보: 2026-11-06 operator-timed** | Tool DO1: `1` 1.3 s → `0` 0.2 s → `1` 0.5 s → `0` | Tool DO2: `1` 0.5 s → `0` | 최종 OFF/OFF, 펄스 방식 | 각 service ACK만. DI/physical completion 없음 | `20261106_154158_full_policy_episode/gripper_initialization.json`, `gripper_commands.json`; 실제 write/ACK 기록. physical jaw 상태는 UNKNOWN |
| 2026-10-14 operator calibration | Tool DO1 이중 펄스, 당시 일부 1.0/0.5 s | Tool DO2 0.5 s 펄스 | 최종 OFF/OFF | ACK와 output readback 시도; input limit 없음 | OPEN은 operator가 실제 OPEN 관찰. CLOSE는 ACK는 성공했으나 physical CLOSE 확인 실패 |
| historical real collector | Tool DO1 `1` 1 s → `0` 1 s, 2회 | Tool DO2 `1` 1 s → `0` 1 s, 1회 | OPEN 총 4 s, CLOSE 총 2 s | service ACK만, 내부 commanded state | 실기 portable dataset collector에서 episode마다 사용됨 |
| interlocked held level 후보 | opposite OFF 후 OPEN DO1=1 또는 CLOSE DO2=1 | level 유지 | level 방식 | ACK만 | 2026-10-14 시험 코드/로그는 있으나 readback/physical 결과가 불완전하여 기본에서 제외 |

## 코드 근거

- ROS service 생성: 읽기 전용 `~/robot_ws/.../single_robot_simple.py:757-761`; request `index/value`: `:915-926`.
- historical pulse: 같은 파일 `:929-961`; DO1/DO2 상수는 `:113-115`; episode OPEN/CLOSE 호출은 `:2001-2002`, `:2053-2055`.
- phase12 서비스 allowlist: `../14_real_rollout_integration/real_sink.py:8-14`.
- 중복 상태 억제와 timed pulse: `real_sink.py:196-226`.
- 최신 calibration sequence/config: `run_gripper_level_open_once.py:79-85,103-118`.
- Tool output과 Control-box output 구분 및 ROS one-based index 설명: `real_trials/20261014_gripper_mapping_validation_01/GRIPPER_MAPPING_VALIDATION_REPORT.md:5-12`.
- service type 원본: 읽기 전용 `~/robot_ws/src/doosan-robot2/dsr_msgs2/srv/io/SetToolDigitalOutput.srv:1-8` (`index` 1–6, `value` 0/1, response `success`).

## 실기 이력 해석

- OPEN 물리 관찰: `20261014_operator_timed_gripper_validation_002/open_01/operator_observation.json:2-7`에 operator message와 `observed_physical_state=OPEN`.
- CLOSE 반례: mapping report `gripper_close_test.json:2-7`은 service 요청 이후 실제 상태가 OPEN으로 남았음을 기록한다.
- 최신 2026-11-06 full-policy run은 DO1 initialization pulse와 ACK를 기록하지만 `observed_open=null`, `measured_gripper_state=null`이다. 따라서 “전기 명령이 실제 사용됨”은 확인되지만 jaw 완료는 확인되지 않았다.
- tool digital input의 jaw-open/jaw-close 매핑은 찾지 못했다. 완료 시각은 마지막 output service ACK 시각으로만 기록해야 하며 물리 완료로 해석하면 안 된다.

후속 2026-11-13 attestation/readback에 따라 v2 기본은 `LevelIOGripper`로 변경했다. OPEN은 DO2 Off→DO1 On, CLOSE는 DO1 Off→DO2 On을 level 유지하고 양 채널을 readback한다. `BinaryIOGripper`는 비기본 대안이다. 모델값 `>=0.5`는 CLOSE, `<0.5`는 OPEN이며 logical state가 바뀔 때만 명령한다.

주의: srv 주석은 `0=ON, 1=OFF`지만 펜던트 양쪽 Off 상태의 getter는 둘 다 0이었다. 따라서 v2는 **[추정] `0=Off, 1=On`**을 사용하며 첫 gripper_test에서 펜던트 표시와 getter로 확정한다. 불일치 시 두 출력 Off 후 중단하며 `gripper_io_verified`는 그 전까지 false다.
