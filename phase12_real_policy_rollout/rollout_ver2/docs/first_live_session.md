# 첫 연구실 세션 절차 — stop_test_movej까지

첫 세션 목표는 **read-only 확인 → dryrun_live → gripper_test → stop_test_movej**까지다. Servoj 탐색, `stop_test_servoj`, synthetic, policy는 세션 2 이후다. 한 단계라도 실패하면 이후 단계로 가지 않는다.

## 1. 펜던트·배선 기록

Robot Limits 일반/감소 모드, tool TCP/무게, firmware와 함께 JEGH-3520의 Tool DO1/DO2 실제 배선과 완료 입력 포트의 존재/번호를 사진과 수기 JSON으로 기록한다.

- `pendant_limits_recorded`: Robot Limits 화면 사진과 날짜/작업자/단위가 있어야 한다.
- `gripper_wiring_recorded`: Tool DO1/DO2 단자와 배선, 입력 포트 유무를 식별하는 사진/기록이 있어야 한다. 이는 동작 성공을 뜻하지 않는다.
- `gripper_io_verified`: 이 단계에서는 false를 유지하며 gripper_test 결과 review 뒤에만 true로 바꾼다.
- 중단: 포트·극성·TCP가 불명확하거나 기존 후보와 다름.
- evidence: `reports/live/<session>/pendant_limits.*`, `gripper_wiring.*`, 사진 manifest.

첫 세션은 Default limit을 유지한다. 사람이 펜던트로 real home `[-182.536,-1.058,-40.984,1.254,-113.506,+1.500]°`까지 수동·저속 이동하고 특히 J6 `+1.5°`를 확인한다. v2는 home 이동을 자동화하지 않는다.

## 2. driver real mode와 read-only 수집

기존 승인 절차로 driver를 실행한 뒤 다음만 수행한다.

`tools/live_readonly_check.sh reports/live/<session>/readonly.txt`

- 통과: REAL/AUTO/STANDBY, `/dsr01` topic/service와 QoS/주기 정상, servoj publisher 0, state 1, unexpected motion 없음.
- 중단: SAFE/RECOVERY/NOT_READY, stale/저주기, namespace/type 불일치, 기존 publisher 존재.
- evidence: launch stdout, pendant 상태 사진, `readonly.txt`.
- review 결과로 `live_topics_verified`, `feedback_rate_verified`를 등록한다.

## 3. 설정과 모드별 필수값

| 모드 | 필수 gate | Servoj time/vel/acc | tracking threshold |
|---|---|---|---|
| dryrun_live | 없음(read-only) | 불필요 | 불필요 |
| gripper_test | pendant limits, live topics, gripper wiring, 팔 정지 | 불필요 | 불필요 |
| stop_test_movej | pendant limits, live topics, feedback rate | 불필요 | 불필요 |
| stop_test_servoj | 위 항목 + stop_movej 결과, Servoj parameter, tracking calibration | 필요 | 필요 |
| synthetic/policy | 위 Servoj 항목 + `stop_servoj_verified` | 필요 | 필요 |

`stop_test_movej` 전에 `runtime_config.json`의 movej acceleration과 MoveStop mode를 사람이 검토·입력한다. null/[미확인] 필수값이 있으면 executor가 거부한다. action clamp나 limit 완화는 금지한다.

## 4. dryrun_live

feedback/camera/model만 읽고 명령은 파일에만 기록한다.

- 통과: 60초 이상 fresh feedback, 유효 `tool0_real`, JPEG q92/raw action/endpoint 기록, command count 0.
- 중단: stale/state fault/topic gap/image 오류/기존 publisher.
- evidence: feedback/action JSONL, timing, command-count-zero audit.

## 5. gripper_test

`pendant_limits_recorded`, `live_topics_verified`, `gripper_wiring_recorded`, `arm_stationary_verified` evidence와 fresh confirmation이 모두 필요하다. 팔 정지를 JointState 속도와 작업자 확인으로 먼저 기록한 뒤 OPEN/CLOSE를 각 1회만 수행한다.

- 통과: 기존 Tool DO pulse 순서와 ACK, jaw 상태·개폐 시간, 입력 포트가 있으면 완료 timestamp 기록.
- 중단: 팔 속도 비영, mapping/readback 불일치, ACK 실패, jaw 미완료, unexpected motion.
- evidence: arm-stationary sample, pulse/ACK JSONL, 사진/영상, operator observation, 선택적 input log.
- 독립 review 완료 뒤에만 `gripper_io_verified=true`로 등록한다.

## 6. stop_test_movej — 세션 1 종료점

real home에서 `/dsr01/motion/move_joint`(`dsr_msgs2/srv/MoveJoint`)로 J6만 `+2°`, 속도 `5°/s 이하`로 비동기 이동시키는 동안 `/dsr01/motion/move_stop`을 1회 호출한다. acceleration은 현장 검토값을 채워야 한다. Servoj parameter와 tracking threshold는 필요하지 않다.

MoveStop 후보는 `DR_QSTOP=1`이며 **[미확인: 현장 검토]**다. `stop_test_servoj`도 반드시 같은 설정값을 사용한다.

- 모든 RobotState callback을 수신 주기 그대로 `robot_state_full_rate.jsonl`에 기록한다. `current_posj/current_velj/current_posx/current_tool_posx/rotation_matrix/f_current_pos_world/sync_time`과 host monotonic 수신 시각을 보존하여 FK 변화량과 posx 변화량 일치도를 사후 분석한다.
- 통과: MoveStop ACK(또는 선행 SAFE 상태), 계속 갱신된 JointState에서 zero velocity 0.3초 후보, 추가 command 0, full-rate log 완전성 확인.
- 중단: 시작이 real home 아님, 설정 null, 한계/상태/stale fault, ACK와 SAFE 상태 모두 없음, zero velocity 미확인.
- evidence: MoveJoint request/ACK, full-rate RobotState와 JointState, stop receipt, monotonic timeline, pendant 영상.
- review 뒤 `stop_movej_verified=true`로 등록한다.

**`stop_movej_verified`는 정지 서비스와 판정 로직의 검증이며, servoj 스트림 중 정지 동작의 검증을 대신하지 않는다.**

## 세션 2 첫 Servoj 순서

1. 같은 MoveStop mode가 선택됐는지 재확인한다.
2. 현재 위치 유지 Servoj stream으로 움직임 0을 확인한다.
3. 관절 1개를 미소 이동하며 Servoj time/vel/acc 후보를 탐색한다.
4. `stop_test_servoj`를 수행하고 evidence review 뒤 `stop_servoj_verified=true`로 등록한다.
5. 그 다음에만 synthetic로 진행한다. policy는 별도 승인 이후다.

`stop_servoj_verified` 없이는 synthetic와 policy gate가 열리지 않는다.
