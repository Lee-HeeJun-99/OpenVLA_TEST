# 계측 활성 Doosan driver baseline 준비 결과

- 일자: 2026-10-02 (Asia/Seoul)
- 요청 범위: 계측 활성 driver 정상 기동 및 60초 baseline까지만 수행
- 결과: **BLOCKED_UNSAFE_DRIVER_STARTUP_SIDE_EFFECTS**

## 1. 기존 process 및 graph 상태

로컬 process 목록에는 다음 process가 없었다.

- `ros2_control_node`
- controller manager process
- `dsr_controller2` spawner/process
- Doosan bringup launch process

따라서 중복 launch process도 발견되지 않았다. 반면 ROS graph에는 이전 `/dsr01/*` node/service/topic endpoint가 일부 남아 있었고, subscriber-only 검사에서 10.034초 동안 JointState 실제 메시지는 0개였다. 이 graph 정보는 정상 driver data plane의 증거로 사용하지 않았다.

## 2. build/overlay 확인

적용 순서는 다음과 같이 확인했다.

```bash
source /opt/ros/humble/setup.bash
source /home/ubuntu/robot_ws/install/setup.bash
```

`ros2 pkg prefix` 결과:

```text
dsr_common2     /home/ubuntu/robot_ws/install/dsr_common2
dsr_hardware2   /home/ubuntu/robot_ws/install/dsr_hardware2
dsr_controller2 /home/ubuntu/robot_ws/install/dsr_controller2
```

다른 overlay가 세 패키지의 새 build를 가리는 증거는 없었다. 설치 library hash도 build 검증 시 기록한 값과 일치했다.

## 3. DSR_TRACE 전달 방식

계측 코드는 process 환경변수 `DSR_TRACE_ENABLE=1`을 읽는다. 따라서 launch를 시작하는 shell에서 다음처럼 export되어야 child `ros2_control_node`까지 전달된다.

```bash
export DSR_TRACE_ENABLE=1
```

이후 runtime log에 실제 `DSR_TRACE seq=...` record가 나타나는 것으로 활성 여부를 검증해야 한다. 바이너리 문자열 존재만으로 활성 판정하지 않는다.

최근 PID 2903100의 log에는 `DSR_TRACE`가 없으므로 해당 relaunch에 환경변수가 전달됐다는 증거는 없다.

## 4. launch를 실행하지 않은 안전 근거

현재 real hardware plugin의 `DRHWInterface::on_init()`은 observation-only가 아니다. 정상 launch 과정에서 다음 state-changing API를 호출한다.

```text
ManageAccessControl(MANAGE_ACCESS_CONTROL_FORCE_REQUEST)
set_robot_control(CONTROL_SERVO_ON)
SetRobotMode(ROBOT_MODE_AUTONOMOUS)
SetRobotSystem(ROBOT_SYSTEM_REAL)
set_auto_servo_off(...)
connect_rt_control(...)
set_rt_control_output(...)
start_rt_control()
set_velj_rt(...)
set_accj_rt(...)
set_safety_mode(...)
```

사용자는 이번 단계에서 robot mode 변경 및 상태 변경 명령을 금지했다. 이 driver launch를 실행하면 그 제한을 직접 위반하므로 새 driver를 launch하지 않았다. 이에 따라 새 PID, runtime DSR_TRACE, 60초 JointState/CPU baseline은 생성하지 않았다.

## 5. 이전 launch 종료와 관련해 확인된 로그

최근 launch에서 직접 확인된 흐름:

1. hardware `a0509` initialization/configure/activate 성공
2. `joint_state_broadcaster` load/configure/activate 성공
3. `dsr_controller2` load/configure 시작
4. `switch_controller` 결과 대기 10초 timeout 3회
5. spawner PID 2903096이 exit code 1로 종료
6. 약 48.166565초 update-loop gap 기록
7. controller manager가 `Aborting, no controller is switched! (::STRICT switch)` 기록
8. 늦게 도착한 service response 전송 실패 기록

이 로그는 `dsr_controller2` spawner 실패를 직접 증명한다. 다만 `ros2_control_node` PID 2903100이 최종적으로 종료된 정확한 원인은 launch log에 명시적인 crash/segfault/abort/process-exit record가 없어 확정하지 않았다.

## 6. 요청된 baseline 결과

| 항목 | 결과 |
|---|---|
| 새 `ros2_control_node` PID | NOT_EXECUTED |
| controller/broadcaster activation | NOT_EXECUTED |
| runtime `DSR_TRACE` | NOT_EXECUTED |
| JointState 60초 count/rate/gap | NOT_EXECUTED |
| process/thread CPU baseline | NOT_EXECUTED |
| getter endpoint 신규 확인 | NOT_EXECUTED |
| getter 재현 준비 | **NOT_READY** |

## 7. 다음 조치

다음 중 하나가 명시적으로 결정돼야 한다.

1. 기존 real driver startup의 servo-on/mode/system/RT/safety 설정 부작용을 현장 작업자가 별도로 승인한 뒤 launch한다.
2. state-changing startup API를 호출하지 않는 별도의 observation-only hardware path를 설계·검증한다.

어느 경우든 launch 이후 실제 `DSR_TRACE` 출력과 60초 JointState baseline을 통과하기 전에는 `get_current_posx`를 호출하면 안 된다.

## 8. 실행 횟수

- driver/controller launch or restart: 0
- Robot command/state-changing API: 0
- getter/service: 0
- motion/gripper/Home/trajectory: 0
- Hold/E-stop: 0
- AI action/closed-loop: 0

## 9. 현장 재launch 후속 관측 (14:49 KST)

사용자가 현장에서 다시 launch한 뒤 새 runtime을 확인했다.

### 기동 및 계측

- 새 `ros2_control_node` log PID: `2905006`
- hardware `a0509`: initialize/configure/activate 성공
- `joint_state_broadcaster`: load/configure 기록 존재
- `dsr_controller2`: load/configure 기록 존재
- getter endpoint: `/dsr01/aux_control/get_current_posx`, type `dsr_msgs2/srv/GetCurrentPosx`
- runtime `DSR_TRACE`: **활성 확인**

실제 trace에는 TID `2905110`에서 아래 event가 약 10 ms 주기로 반복됐다.

```text
DRHWInterface::read ENTER/EXIT
READ_DATA_RT_START/END
CDRFLEx::read_data_rt ENTER/EXIT
_read_data_rt CALL_START/CALL_END
STATE_INTERFACES_UPDATED
```

따라서 새 build와 `DSR_TRACE_ENABLE=1`이 runtime에 적용된 사실은 직접 확인됐다.

### data plane 및 controller 상태

그러나 subscriber-only probe는 다시 실패했다.

```text
wall elapsed: 10.048171140 s
first receive: null
JointState messages: 0
feedback_ready: false
```

`ros2 control list_controllers`도 `/dsr01/controller_manager/list_controllers` 응답을 10초 안에 받지 못했다. 그러므로 controller/broadcaster 정상 상태를 확인하지 못했다.

### 계측 부하

launch log는 약 3분 만에 31 MB를 넘었고 trace sequence가 `161000`을 초과했다. 현재 계측은 100 Hz read cycle마다 여러 줄을 동기적으로 출력한다. `_read_data_rt`와 `DRHWInterface::read`는 계속 진행하지만 controller-manager service와 JointState publish data plane은 정상 확인되지 않았다.

이 결과만으로 trace logging이 장애의 단독 원인이라고 확정하지 않는다. 다만 현재 고빈도 계측 상태는 원래 baseline과 동등하지 않고 timing을 교란할 가능성이 크므로 60초 baseline 및 getter 재현 조건으로 승인하지 않는다.

### 후속 판정

| 조건 | 결과 |
|---|---|
| 새 runtime 기동 | PASS |
| runtime DSR_TRACE | PASS |
| hardware read 지속 | PASS (trace 기준) |
| JointState 실제 약 100 Hz | FAIL: 0 messages/10.048 s |
| controller/broadcaster 정상 확인 | BLOCKED: list_controllers timeout |
| 60초 baseline | NOT_EXECUTED |
| CPU baseline | NOT_AVAILABLE_FROM_CURRENT_PROCESS_NAMESPACE |
| getter 재현 준비 | **NOT_READY** |

안전 gate에 따라 getter는 호출하지 않았다. 다음 단계는 고빈도 정상 경로 trace를 rate-limit/edge-trigger 방식으로 줄이되 getter, stall, timeout 전후 핵심 event는 전부 남도록 계측을 조정하고 다시 build하는 것이다.

## 고빈도 계측 rate-limit 및 rebuild (2026-10-02)

정상 100 Hz read 경로의 동기 로그 부하를 줄이기 위해 다음과 같이 계측을 조정했다.

- `DSR_TRACE_HIGH_RATE_EVERY`: 고빈도 정상 경로 샘플 간격. 미설정 시 기본값 `100`(약 1초에 한 번)
- `CDRFLEx::read_data_rt`: 정상 ENTER/EXIT 및 CALL_START/CALL_END를 샘플링
- `DRHWInterface::read`: 정상 ENTER/EXIT, READ_DATA_RT_START/END, STATE_INTERFACES_UPDATED를 샘플링
- `DRHWInterface::read` 시작 간격이 50 ms 이상이면 `READ_CYCLE_GAP`을 샘플링과 무관하게 항상 기록
- null RT data 등 오류 event는 샘플링과 무관하게 항상 기록
- getter, command `0x0472`, response/dispatch/event 및 lock 계측은 rate-limit 대상이 아니므로 그대로 유지

빌드 명령:

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select dsr_common2 dsr_hardware2 dsr_controller2 --event-handlers console_direct+
```

결과:

- `dsr_common2`: PASS
- `dsr_hardware2`: PASS (deprecated API warning만 존재)
- `dsr_controller2`: PASS (deprecated API warning만 존재)
- 총 3 package 성공, build time 약 9분 2초
- 설치 라이브러리 SHA-256:
  - `libdsr_hardware2.so`: `caa6e84a5fc165b837243fece9b486460d92c44a41a42837842cd91d50c5a5b7`
  - `libdsr_controller2.so`: `65fd47b7d066778bd8f81e7e3af9fa86da5da8f56b748c03a70d3fb72102c8a9`
- 설치된 `libdsr_hardware2.so`에서 `DSR_TRACE_HIGH_RATE_EVERY`와 `READ_CYCLE_GAP` 문자열을 확인했다.

현재 실행 중인 driver process는 재빌드 전 라이브러리를 이미 적재했으므로 새 rate-limit 코드는 **다음 현장 재launch 이후에만** 적용된다. 이번 단계에서는 driver/controller를 재시작하지 않았고 getter도 호출하지 않았다.

## Rate-limit build 적용 후 현장 재launch baseline (2026-10-02 15:05 KST)

- launch PID: `2906356`
- 새 `ros2_control_node` PID: `2906371`
- install overlay: `/home/ubuntu/robot_ws/install`이 우선 적용됨
- `/dsr01/joint_states`: publisher 1 (`/dsr01/joint_state_broadcaster`)
- getter endpoint: `/dsr01/aux_control/get_current_posx`, type `dsr_msgs2/srv/GetCurrentPosx`
- runtime `DSR_TRACE`: 활성
- rate-limit 적용: 정상 read trace가 약 1초 간격으로 출력됨
- launch log: 관찰 시점 6,419 lines / 1,097,487 bytes (이전 약 3분 31 MB 대비 크게 감소)

Subscriber-only 60초 baseline:

| 항목 | 결과 |
|---|---:|
| readiness 이후 message count | 6,001 |
| source span | 60.000109 s |
| source rate | 99.999818 Hz |
| source max gap | 12.126 ms |
| receive max gap | 13.120 ms |
| gap >= 50 ms | 0 |
| gap >= 100 ms | 0 |
| duplicate / non-monotonic | 0 / 0 |
| invalid position / velocity | 0 / 0 |
| missing joint | 0 |
| effort unsupported | 6,202 (예상된 NaN/미지원) |

관절 배열은 기존과 동일하게 `joint_1, joint_2, joint_4, joint_5, joint_3, joint_6`였으며 소비자는 name 기반 canonical reorder가 필요하다. 원본 결과는 `jointstate_baseline_rate_limited_20261002.json`에 저장했다.

Controller read-only 조회는 15초 제한 안에 성공했고 `joint_state_broadcaster`와 `dsr_controller2`가 모두 active였다. `ros2_control_node` 전체 CPU는 약 201%였고 TID `2906394`, `2906534`가 각각 약 100%였다. 높은 CPU는 유지되지만 이번 60초 JointState 연속성을 방해하지 않았다.

로그의 `READ_CYCLE_GAP gap_ns=42903653844`는 startup 시점의 단일 약 42.9초 gap이며, 이후 실제 subscriber baseline에는 50 ms 이상 gap이 없었다. startup 중 과거 `switch_controller` response timeout warning 두 건은 존재하지만 현재 controllers는 active다.

판정: **GETTER_SINGLE_REPRODUCTION_READY**. 단, 본 단계에서는 getter를 호출하지 않았다. 다음 단계는 별도 승인 후 `get_current_posx(ref=0)` 정확히 1회와 전후 계측 timeline 수집이다.
