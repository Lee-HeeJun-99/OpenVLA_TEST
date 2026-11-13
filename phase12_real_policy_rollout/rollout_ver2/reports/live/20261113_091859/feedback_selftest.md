# RobotFeedback read-only 자체 점검 — 2026-11-13

## 안전 범위

유효 측정 30.0001초 동안 수정한 `RobotFeedback`만 인스턴스화했다. `/dsr01/joint_states`, `/dsr01/error`, `/dsr01/robot_disconnection` 구독과 `get_robot_state/mode/system/current_posj/current_tool_flange_posx/current_rotm` 호출만 사용했다. publisher, ServojPublisher, MoveJoint, MoveStop, gripper I/O 객체는 생성하지 않았다.

첫 sandbox 실행은 UDP socket 차단으로 수집 0건이었으며 폐기했다. 최종 실행은 DDS warm-up 1.002초 뒤 통계를 초기화하고 30초 전 구간을 측정했다.

## 결과

| 항목 | 결과 |
|---|---:|
| JointState sample | 3,000 |
| host 수신 주기 mean / std / p99 / max | 9.9999 / 0.3476 / 10.9125 / 12.1787 ms |
| header stamp 주기 mean / std / p99 / max | 10.0000 / 0.2028 / 10.7387 / 12.1503 ms |
| 최초 TRANSIENT_LOCAL sample | 기록됨, freshness 판정 제외 확인 |
| 수신 name 순서 | J1,J2,J4,J5,J3,J6 |
| 정규화 순서 | J1,J2,J3,J4,J5,J6 |
| get_current_posj 대비 최대 절대차 | 6.53e-9° |
| robot_state poll | 301/301 성공, 실패 0 |
| poll 주기 mean / std / p99 / max | 99.9986 / 0.3715 / 100.9834 / 102.3368 ms |
| 응답 지연 p50 / p99 / max | 0.954 / 1.943 / 3.292 ms |
| preflight | mode=1, system=0, state=1; 사유 없음 |
| error/disconnection fault | 0 |

host monotonic 수신 시각과 ROS `header.stamp`는 모두 원자료에 저장했다. 서로 다른 clock domain일 수 있어 두 값의 절대 차이를 freshness로 사용하지 않고, host age와 각 clock의 주기만 평가했다.

## 플랜지 교차 검증 — log only

2Hz 후보로 60쌍을 수집했다. 세 get 서비스의 완료 시각 span은 평균 0.685ms, 최대 1.291ms다.

| 오차 | p50 | p99 | max |
|---|---:|---:|---:|
| FK(link_6) ↔ flange 위치 | 1.0164mm | 1.0164mm | 1.0164mm |
| FK(link_6) ↔ rotm 회전 | 0.08427° | 0.08427° | 0.08427° |

정지 조건이 아니며 상수 bias 성격의 교차 검증 결과다. 위치는 기존 목표 1mm보다 0.016mm 크고 회전은 0.1° 이내다. 기계 판독 원자료는 `feedback_selftest.json`에 있다.

## 명령 배열 순서 보호

`RobotFeedback`은 수신 배열 index가 아니라 `joint_1`…`joint_6` 이름으로 정규화한다. `SegmentGenerator`와 IK는 URDF chain의 J1→J6 배열을 유지하고, `ServojPublisher.publish()`는 이 정규 배열을 shape `(6,)`로 검사한 뒤 단위만 rad→deg로 변환하여 `ServojStream.pos`에 쓴다. 섞인 현장 순서 J1,J2,J4,J5,J3,J6 입력으로 만든 mock 명령이 `[J1,J2,J3,J4,J5,J6]`가 되는 단위 테스트를 추가했다. action이나 관절값 재배열 외 수정/clamp는 없다.

## 판정

- `live_topics_verified`: 갱신 가능. readonly topic/service 목록과 실제 구독·get 호출 성공을 확인했다.
- `feedback_rate_verified`: 갱신 가능. 30초 동안 JointState 100Hz와 state poll 10Hz를 확인했다.
- `pendant_limits_recorded`: 후속 사용자 결정에 따라 `operator_attestation.json` evidence로 true 갱신.
- `gripper_wiring_recorded`: X1 24V PNP·DO1/DO2 attestation과 read-only readback으로 true 갱신. 실제 jaw 결과인 `gripper_io_verified`는 false 유지.
