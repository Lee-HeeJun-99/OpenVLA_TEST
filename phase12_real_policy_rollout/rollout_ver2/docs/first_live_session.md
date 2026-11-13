# 첫 연구실 세션 절차 — stop_test_movej까지

첫 세션 목표는 **read-only 확인 → dryrun_live → gripper_test → stop_test_movej**까지다. Servoj 탐색, `stop_test_servoj`, synthetic, policy는 세션 2 이후다. 한 단계라도 실패하면 이후 단계로 가지 않는다.

## 1. 펜던트·배선 기록

Robot Limits 일반/감소 모드, tool TCP/무게, firmware와 함께 JEGH-3520의 Tool DO1/DO2 실제 배선과 완료 입력 포트의 존재/번호를 photo 또는 operator attestation으로 기록한다.

- `pendant_limits_recorded`: 사진 또는 `operator_attestation`의 날짜·작업자·값·확인 방법·사진 미등록 사유가 있어야 한다. 2026-11-13 사용자 결정으로 attestation을 등록했다.
- `gripper_wiring_recorded`: Tool DO1/DO2와 X1 24V PNP 배선을 operator attestation으로 등록했다. 이는 동작 성공이나 극성 확정을 뜻하지 않는다.
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
- 2026-11-13 read-only/self-test 결과로 `live_topics_verified`, `feedback_rate_verified`는 등록됐다. 이후 세션에서는 같은 검사를 재확인한다.

## 3. 설정과 모드별 필수값

| 모드 | 필수 gate | Servoj time/vel/acc | tracking threshold |
|---|---|---|---|
| dryrun_live | 없음(read-only) | 불필요 | 불필요 |
| gripper_test | pendant limits, live topics, gripper wiring, 팔 정지 | 불필요 | 불필요 |
| stop_test_movej | pendant limits, live topics, feedback rate | 불필요 | 불필요 |
| stop_test_servoj | 위 항목 + stop_movej 결과, Servoj parameter, tracking calibration | 필요 | 필요 |
| synthetic/policy | 위 Servoj 항목 + `stop_servoj_verified` | 필요 | 필요 |
| policy_movej | pendant limits, live topics, feedback rate, `stop_movej_verified` | 불필요 | 불필요 |

2026-11-13 현장 시험으로 MoveJoint acceleration `10°/s²`와 MoveStop `DR_QSTOP=1`을 확인했다. null/[미확인] 필수값이 있으면 executor가 거부한다. action clamp나 limit 완화는 금지한다.

## 4. dryrun_live

### 추론 서버와 v2 health

터미널 1:

```bash
cd /home/ubuntu/a0509_vla_linux_field_bundle_20260903
./scripts/serve.sh vanilla 8766
```

터미널 2:

```bash
cd /home/ubuntu/a0509_vla_linux_field_bundle_20260903/lhj/phase12_real_policy_rollout/rollout_ver2
/usr/bin/python3 tools/check_v2_health.py --url http://127.0.0.1:8766
```

기존 `scripts/healthcheck.sh vanilla http://127.0.0.1:8766`은 vanilla의 ready/variant/chunk/action_dim만 검사한다. 그 파일의 `center_crop=True` 검사는 OFT 분기 전용이다. v2/matched15 vanilla 경로는 center crop 없음이 기준이므로 `check_v2_health.py`가 `center_crop=false`, RGB, vision-only를 명시적으로 요구한다.

### 카메라 확인

```bash
ros2 topic list -t | grep 'sensor_msgs/msg/Image'
export V2_CAMERA_TOPIC=/확인한/image/topic
ros2 topic info "$V2_CAMERA_TOPIC" --verbose
ros2 topic echo "$V2_CAMERA_TOPIC" sensor_msgs/msg/Image --once --field width
ros2 topic echo "$V2_CAMERA_TOPIC" sensor_msgs/msg/Image --once --field height
ros2 topic echo "$V2_CAMERA_TOPIC" sensor_msgs/msg/Image --once --field encoding
```

실제 topic 이름, 해상도, encoding을 session evidence에 기록하고 `rgb8/bgr8/rgba8/bgra8` 중 하나인지 확인한다.

### 실행

feedback/camera/model만 읽으며 publisher와 상태 변경 service client를 만들지 않는다.

환경 setup과 `ROS_DOMAIN_ID=71 ROS_LOCALHOST_ONLY=1` 설정을 마친 뒤 아래 **실행 명령 한 줄만 붙여넣는다**. 확인 문구를 명령에 pipe하거나 함께 붙여넣지 말고 prompt가 나타나면 작업자가 `I_CONFIRM_V2_MODE:dryrun_live`를 직접 입력한다.

```bash
/usr/bin/python3 tools/dryrun_live.py --duration 60 --server http://127.0.0.1:8766 --camera-topic /zed/zed_node/rgb/color/rect/image --instruction "Pick up the blue cube." --scene-cube-color blue --output reports/live/20261113_091859/dryrun_live
```

preflight는 명령 전에 robot_mode=1, robot_system=0, robot_state=1과 real_home 관절별 ±2°를 검사하며 wrap을 허용하지 않는다. 허용 지시문은 matched15의 red/yellow/blue 정확 문자열뿐이다. 그 밖의 지시문은 `--allow-ood-instruction`을 명시해야 하며 summary에 OOD 여부가 기록된다.

실행기는 두 OS process를 `spawn`한다. 프로세스 A는 feedback/get_robot_state/안전 감시만 담당하고 image를 다루지 않는다. 프로세스 B는 ZED/frame barrier/JPEG q92/HTTP inference만 담당한다. 감시 시작 전 실행 시작 이후의 새 joint_states와 robot_state 응답을 각각 기다리며 2초 후보 안에 확보하지 못하면 `FEEDBACK_NO_FIRST_SAMPLE`로 중단한다. B가 죽거나 pipe가 끊기면 A는 `OBSERVATION_PROCESS_DIED_STOP_REQUIRED`로 처리한다. dryrun에는 물리 command가 없으며 향후 command mode에서는 기존 stop 경로로 연결한다.

- 흐름: 가상 `t_settle` 기록 → host/ROS 시각 모두 그 뒤인 새 frame 대기 → 추론/기록 → 새 가상 `t_settle`. 기다리는 동안 action이나 명령을 반복하지 않고 feedback/state/error/disconnection 감시를 계속한다.
- 통과: 60초 이상, camera 10초 대기 상한 초과 0, 관측별 settle/frame 두 시각과 대기 시간 기록, state=1, feedback host age ≤0.1s, virtual 100Hz period p99 <15ms, 유효 `tool0_real`, JPEG q92/raw action/endpoint 기록, command publish 및 상태 변경 service call 0회.
- 정상 종료: 새 frame 대기 10초 후보 초과는 `CAMERA_NO_FRAME_TIMEOUT`으로 기록하고 정지 상태에서 episode를 끝낸다(fault 아님).
- 중단: robot feedback stale/state/error/disconnection fault, image decode 오류, 기존 publisher.
- evidence: feedback/action JSONL, timing, command-count-zero audit.

### 선택 검토: ZED publish 15 Hz

현재 적용하지 않는다. 읽기 전용 source 조사상 ZED2i capture 설정은 `/home/ubuntu/robot_ws/src/zed-ros2-wrapper/zed_wrapper/config/zed2i.yaml`의 `general.grab_frame_rate: 30`이고, image publish 상한은 `config/common_stereo.yaml`의 `general.pub_frame_rate: 0.0`이다. 주석상 `0.0`은 grab rate와 같고 `pub_frame_rate`는 camera rate 이하여야 한다. 다음 세션에서 부하 저감 실험을 별도로 승인하면 복사본/launch용 설정에서 `general.pub_frame_rate: 15.0` 후보를 사용한다. 원본 read-only 파일은 수정하지 않으며 이번 세션 설정에도 적용하지 않는다. capture 자체를 15 Hz로 낮추는 `grab_frame_rate: 15`는 별도 변수이므로 publish-only 시험과 섞지 않는다.

## 5. gripper_test

도구가 먼저 `pendant_limits_recorded`, `live_topics_verified`, `gripper_wiring_recorded`를 검사하고, 2초 정지 측정으로 `arm_stationary.json` 후보 evidence를 만든다. 기존 `arm_stationary_verified`를 미리 true로 바꾸지 않는다. 모든 preflight와 아래 fresh confirmation이 끝난 뒤에만 setter client를 만든다.

```bash
/usr/bin/python3 tools/gripper_test.py
```

명령 한 줄만 붙여넣고 prompt에서 `I_CONFIRM_V2_MODE:gripper_test`를 직접 입력한다.

1. JointState와 작업자가 팔 정지를 확인한다.
2. OPEN: DO2 Off를 먼저 확인/설정하고 DO1 On, 두 출력 readback, 펜던트 표시, 육안 jaw 상태와 소요 시간을 기록한다.
3. CLOSE: DO1 Off를 먼저 확인/설정하고 DO2 On, 같은 항목을 기록한다.
4. 마지막에 DO1/DO2를 모두 Off로 두고 jaw가 유지되는지 또는 풀리는지 관찰한다.

첫 Off write/readback은 같은 service encoding의 일관성만 검사하므로 물리 극성 검사가 아니다. 첫 `DO1←1` 뒤 펜던트 DO1 On 여부를 작업자가 확인한다. `n`이면 즉시 두 출력을 0으로 만들고 `POLARITY_MISMATCH`로 중단한다. 두 출력 동시 On은 금지한다. 모든 예외와 정상 종료의 `finally`에서도 두 출력을 0으로 만든다.

- 통과: level 순서와 ACK/readback, jaw 상태·개폐 시간, 입력 포트가 있으면 완료 timestamp 기록.
- 중단: 팔 속도 비영, mapping/readback 불일치, ACK 실패, jaw 미완료, unexpected motion.
- evidence: arm-stationary sample, level write/readback JSONL, 펜던트 표시, 사진/영상 또는 operator observation, 선택적 input log.
- 독립 review 완료 뒤에만 `gripper_io_verified=true`로 등록한다. policy는 그 전후 모두 gripper hardware를 기록만 하며, 향후 활성화에는 별도 `gripper_io_verified` 조건이 필요하다.

## 6. stop_test_movej — 세션 1 종료점

real home에서 `/dsr01/motion/move_joint`(`dsr_msgs2/srv/MoveJoint`)로 J6만 `+2°`, 속도 `5°/s`, 가속도 `10°/s²` 후보로 비동기 이동시키는 동안 `/dsr01/motion/move_stop`을 1회 호출한다. 가속도와 stop mode는 실행 직전 현장 검토 대상이다. Servoj parameter와 tracking threshold는 필요하지 않다.

```bash
/usr/bin/python3 tools/stop_test_movej.py
```

명령 한 줄만 붙여넣고 prompt에서 `I_CONFIRM_V2_MODE:stop_test_movej`를 직접 입력한다. 자동 재시도는 구현하지 않았다.

2° 이동에서 `v=5°/s`, `a=10°/s²`는 최고 속도 도달 전 감속해야 하는 삼각 프로파일이다. 예상 최고 속도는 `sqrt(a·distance)=4.472°/s`, 가속/감속 각 0.4472초, 총 0.8944초다.

| 시간 (s) | 예상 속도 (°/s) |
|---:|---:|
| 0.0000 | 0.000 |
| 0.4472 | 4.472 |
| 0.8944 | 0.000 |

MoveStop trigger는 측정 J6 속도가 예상 최고 속도의 50%인 `2.236°/s` 이상이 된 첫 JointState다. 예상 총시간의 70%인 `0.6261초` 안에 도달하지 않으면 시험 무효로 표시하되 MoveStop을 호출하고 종료한다.

MoveStop은 현장 시험에서 확인한 `DR_QSTOP=1`이다. `stop_test_servoj`도 반드시 같은 설정값을 사용한다.

- RobotState topic은 현장 graph에 없다. 모든 `/joint_states` callback의 이름 정규화 관절각/속도, host 수신 시각, header stamp를 기록하고, `get_robot_state`를 계속 polling한다. flange 교차 검증은 저주기 `get_current_tool_flange_posx/get_current_rotm` 로그로 수행한다.
- 통과: stop 요청 sample의 J6 속도가 0이 아니고, MoveStop ACK(또는 선행 SAFE 상태), `max|qdot|≤0.001 rad/s` 0.3초 연속, MoveStop 이후 MoveJoint 추가 호출 0, full-rate log 완전성 확인.
- 중단: 시작이 real home 아님, 설정 null, 한계/상태/stale fault, ACK와 SAFE 상태 모두 없음, zero velocity 미확인.
- evidence: MoveJoint request/ACK, full-rate JointState, state polling과 flange get 로그, stop receipt, monotonic timeline, pendant 영상.
- review 뒤 `stop_movej_verified=true`로 등록한다.

2026-11-13 사용자 review 완료 뒤 `reports/live/20261113_091859/stop_test_movej/summary.json`을 measurement evidence로 `stop_movej_verified=true`에 등록했다.

**`stop_movej_verified`는 정지 서비스와 판정 로직의 검증이며, servoj 스트림 중 정지 동작의 검증을 대신하지 않는다.**

## 세션 2 첫 Servoj 순서

1. 같은 MoveStop mode가 선택됐는지 재확인한다.
2. 현재 위치 유지 Servoj stream으로 움직임 0을 확인한다.
3. 관절 1개를 미소 이동하며 Servoj time/vel/acc 후보를 탐색한다.
4. `stop_test_servoj`를 수행하고 evidence review 뒤 `stop_servoj_verified=true`로 등록한다.
5. 그 다음에만 synthetic로 진행한다. policy는 별도 승인 이후다.

`stop_servoj_verified` 없이는 synthetic와 policy gate가 열리지 않는다.

## policy_movej — 별도 승인된 Mode A 경로

`policy_movej`는 Servoj 경로를 대체하거나 그 gate를 완화하지 않는 별도 실행 모드다. 매 step에서 정착 뒤 새 frame만 관측하고, action 전체를 사전 검사한 뒤 비동기 MoveJoint 한 번으로 목표 관절각에 이동한다. 그리퍼는 `gripper_io_verified=false`인 현재 계속 기록만 한다. 자동 home 복귀는 없으며 종료 뒤 작업자가 펜던트로 복귀한다.

환경 setup과 추론 서버 준비 후 아래 **한 줄만 붙여넣고**, prompt에서 `I_CONFIRM_V2_MODE:policy_movej`와 `TABLE_WATCH_BY_OPERATOR`를 각각 직접 입력한다. 후자는 자동 table geometry/height 검사가 없으며 작업자가 충돌 위험을 계속 감시한다는 확인이다. 실행 뒤 Enter 또는 `q`+Enter는 MoveJoint 이동 중에도 즉시 MoveStop을 요청하며 key 입력→MoveStop 요청 시간을 summary에 기록한다.

```bash
/usr/bin/python3 tools/policy_movej.py --server http://127.0.0.1:8766 --camera-topic /zed/zed_node/rgb/color/rect/image --instruction "Pick up the blue cube." --scene-cube-color blue --max-steps 300 --max-episode-time 600 --session reports/live/20261113_091859
```

- preflight: AUTO/REAL/STANDBY, real home 각 관절 ±2°(wrap 불허), matched15 지시문, pendant/live/feedback/stop_movej gate.
- 이동 전 통과: IK, host joint margin(J5 하한 -130° 포함), `σmin≥0.0803`, translation ≤20 mm, rotation ≤4°, 각 관절 변화 ≤10°, 현재→목표 직선 관절 경로 20점의 margin/singularity.
- 실행/정착 후보: MoveJoint 5°/s, 10°/s², async; `|qdot|≤0.001 rad/s` 및 목표 오차 ≤0.1°를 0.3 s 연속. 5 s 초과나 안전 이상은 DR_QSTOP 경로로 종료한다.
- episode 예산 기본값: 최대 300 step, 600 s 후보이며 CLI에서 양의 값으로 조정할 수 있다. 이는 기존 80-step 관찰 제한을 제거한 전체 rollout 예산이다. camera 지연 중 action을 반복하지 않는다.
- `--sigma-min` 기본값은 `0.0803`이다. 더 높은 값은 즉시 적용할 수 있지만 더 낮은 값은 반드시 `--sigma-min-reason "..."`을 함께 입력해야 하며 값·사유를 summary에 남긴다. 관절 margin과 다른 안전 조건은 변하지 않는다.
- 선택 보조장치 `--max-descent-mm N`은 기본 비활성이다. 활성화하면 real-home tool0_real z보다 N mm 이상 낮은 목표 또는 20점 관절 경로를 `DESCENT_LIMIT`으로 이동 전에 거부한다. 이는 table 형상 검사를 대신하지 않는다.
- 정상 종료: step/time 예산 또는 정지 상태의 camera 새-frame 대기 10 s 초과. 안전 이상·작업자 중단은 기존 MoveStop 경로다.
- evidence: timestamp 폴더의 `steps.jsonl`, 입력 JPEG, `predictions.jsonl`, `summary.json`. 종료 사유/실행 step/경과시간, 누적 tool0_real 변위, home 대비 최저 높이, sim envelope 최초 이탈, endpoint 요약과 command/actual Δ를 검토한다.
