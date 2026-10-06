# get_current_posx 단발 계측 재현 시도

- 일자: 2026-10-02 (Asia/Seoul)
- 목적: 계측 적용 상태에서 `get_current_posx(ref=0)` 장애 위치를 단 한 번의 호출로 식별
- 결과: **NOT_EXECUTED_BASELINE_FAILED**

## 안전 gate 결과

사용자가 보고한 현장 baseline 이후 현재 상태를 다시 확인했다. ROS graph에는 아래 endpoint가 남아 있었다.

- `/dsr01/dsr_controller2`
- `/dsr01/joint_state_broadcaster`
- `/dsr01/joint_states` publisher 1개
- `/dsr01/aux_control/get_current_posx` type `dsr_msgs2/srv/GetCurrentPosx`

그러나 endpoint 존재 여부와 실제 data plane을 분리해 subscriber-only continuity probe를 수행한 결과는 다음과 같다.

```text
requested measurement duration: 5.0 s
discovery timeout: 10.0 s
wall elapsed: 10.034141432 s
first receive: null
JointState message count: 0
feedback_ready: false
classification: PASSIVE_JOINTSTATE_CONTINUITY_PROBE
```

따라서 "실제 JointState 수신 및 약 100 Hz 유지"라는 필수 사전 조건을 현재 실행 시점에는 충족하지 못했다.

## 실행 중 바이너리와 계측 상태

- 최근 launch의 `ros2_control_node` PID는 `2903100`이었다.
- 검사 시 `/proc/2903100`은 존재하지 않았고 로컬 process 목록에도 해당 `ros2_control_node`가 없었다.
- 해당 프로세스 로그 마지막 수정 시각은 `2026-10-02 14:24:48 +0900`이었다.
- 최근 로그에서 `DSR_TRACE` record는 발견되지 않았다. 즉 `DSR_TRACE_ENABLE=1`이 적용된 실행 중 바이너리를 현재 확인할 수 없었다.
- launch log에는 `dsr_controller2` spawner PID 2903096이 exit code 1로 종료된 기록이 있다. 이것만으로 `ros2_control_node` 종료 원인을 확정하지 않는다.

ROS graph에 보이는 endpoint는 stale discovery 정보일 수 있으며, 실제 메시지가 0개였으므로 정상 실행 증거로 사용하지 않았다.

## Getter 실행 감사

사전 안전 gate 실패로 아래 호출은 수행하지 않았다.

```text
/dsr01/aux_control/get_current_posx(ref=0): 0 calls
```

따라서 이번 시도에서는 다음 항목을 판정할 수 없다.

- getter 성공/timeout과 latency: NOT_EXECUTED
- 0x0472 send: NOT_OBSERVED
- TCP receive: NOT_OBSERVED
- response dispatch: NOT_OBSERVED
- Event set: NOT_OBSERVED
- UDP RT packet 지속: NOT_MEASURED
- `read_data_rt` 지속: NOT_MEASURED
- `DRHWInterface::read` 지속: NOT_MEASURED
- JointState publish 중단 시점: baseline 시작 전부터 수신 0개
- mutex 이상: NOT_MEASURED
- A/B/C/D/E 분류: **UNCLASSIFIED_NO_REPRODUCTION**

## 이번 시도의 timeline

| 순서 | 관측 |
|---:|---|
| 1 | ROS graph에서 JointState publisher와 getter endpoint 발견 |
| 2 | 최근 PID 2903100의 `/proc` 부재 및 로컬 process 부재 확인 |
| 3 | 최근 driver log에서 `DSR_TRACE` 출력 없음 확인 |
| 4 | subscriber-only JointState probe 시작 |
| 5 | 10.034초 동안 JointState 0개, first receive 없음 |
| 6 | safety gate 실패로 getter 호출 없이 종료 |

monotonic timestamp를 가진 getter/driver event가 생성되지 않았으므로 요청된 통합 장애 timeline은 만들 수 없다. 존재하지 않는 event timestamp를 추정하지 않았다.

## 직접 증명된 사실과 미확정 사항

직접 증명:

- 현재 검사 시점에 JointState 실제 수신은 0개였다.
- 최근 `ros2_control_node` PID는 더 이상 존재하지 않았다.
- 현재 확보한 최근 로그에는 `DSR_TRACE` 출력이 없었다.
- getter, motion 및 상태 변경 명령은 실행하지 않았다.

미확정:

- 현장 baseline 종료 후 driver가 언제, 왜 종료됐는지
- ROS graph endpoint가 stale 상태로 남은 정확한 이유
- 계측 환경변수가 현장 relaunch에 전달됐는지
- 0x0472 응답, TCP dispatch, RT UDP, hardware read, broadcaster 중 실제 장애 위치

정확한 단발 재현은 `DSR_TRACE_ENABLE=1`이 적용된 새 driver process가 실제로 살아 있고, 계측 로그가 출력되며, JointState subscriber가 연속 100 Hz baseline을 확인하는 동일 시점에 다시 승인된 절차로 수행해야 한다.

## 안전 실행 횟수

- Robot command: 0
- getter/service: 0
- motion service/action: 0
- gripper/Home/trajectory: 0
- Hold/E-stop: 0
- controller/driver restart: 0
- AI action publish/closed-loop: 0

---

# Follow-up: rate-limited instrumentation으로 단발 재현 완료

- 일자: 2026-10-02 (Asia/Seoul)
- runtime: `ros2_control_node` PID `2906371`
- 호출: `/dsr01/aux_control/get_current_posx`, request `ref=0`, 정확히 1회
- client timeout: 3.0 s
- 결과: `TIMEOUT`, measured latency `3.001260999 s`
- command issued: `false`

## 호출 전 gate

60초 subscriber-only baseline에서 6,001 samples, 99.999818 Hz, source max gap 12.126 ms, 50 ms 이상 gap 0회를 확인했다. `dsr_controller2`와 `joint_state_broadcaster`는 active였고 runtime trace는 약 1초 간격으로 동작했다.

## 호출 전후 JointState

호출 전후 probe 전체에서는 3,190 messages를 받았지만 아래 세 source feedback stall이 검출됐다.

| 순서 | source gap | receive gap 관측 | 의미 |
|---:|---:|---:|---|
| 1 | 1.038931346 s | 직전 callback receive gap 1.042685263 s | source와 subscriber 모두 약 1.04초 중단 |
| 2 | 1.450020562 s | 직전 callback receive gap 1.451129890 s | source와 subscriber 모두 약 1.45초 중단 |
| 3 | 3.059996411 s | 직전 callback receive gap 3.063667192 s | source와 subscriber 모두 약 3.06초 중단 |

세 gap 뒤에는 메시지가 다시 수신됐고 이후 20.009968초 동안 2,002 samples, 100.000158 Hz, max source gap 11.785 ms로 일시 회복했다. 원본은 `jointstate_getter_reproduction_20261002.json`이다.

그러나 후속 5초 recovery probe에서는 정상 연속 상태가 유지되지 않았다. 14.20초 wall interval 동안 87 messages만 수신했고 source 기준 약 3.058~3.070초 gap 4개가 반복됐다. 따라서 driver process는 살아 있지만 JointState publish/delivery는 약 3초 주기의 비정상 burst 상태로 남았다. 원본은 `jointstate_post_getter_recovery_20261002.json`이다.

## Driver 계측 결과

정확한 marker 검색 결과 다음 event는 모두 0개였다.

- `function=get_current_posx_cb`
- `event=GET_CURRENT_POSX*`
- `function=CDRFLEx::_get_current_posx`
- `function=SendCurrentTaskPoseCommand`
- `command=0x0472` 또는 `command_id=0x0472`
- 관련 `LOCK_WAIT_START`, `LOCK_ACQUIRED`, `LOCK_RELEASE`

따라서 이번 request가 `dsr_controller2` service callback에 진입했다는 증거가 없고, `0x0472`가 controller로 전송됐다는 증거도 없다. TCP response/dispatch/Event set 역시 발생하지 않았다.

반면 rate-limited trace는 stall 전후 및 후속 비정상 구간에도 매초 다음 경로가 지속됐음을 보여준다.

```text
DRHWInterface::read
→ CDRFLEx::read_data_rt
→ _read_data_rt (non-null result)
→ STATE_INTERFACES_UPDATED
```

예를 들어 real timestamp `1790921668.658947561`부터 `1790921681.658936555`까지 매초 정상 return과 state-interface update가 기록됐다. `ros2_control_node` PID도 생존했다.

## 장애 분류

요청된 A~E 중 feedback 경로에는 **E가 가장 가깝다**.

```text
E. DRHWInterface::read까지 계속 동작하지만 JointState publish/delivery가 중단
```

다만 getter 경로 자체는 A/B 이전 단계다. callback ENTER 및 0x0472 SEND가 없으므로 `controller response 없음(A)`이나 `TCP dispatch 실패(B)`로 분류할 수 없다. 더 정확한 판정은 다음과 같다.

```text
PRE_CALLBACK_SERVICE_EXECUTION_STALL
+
E_LIKE_JOINTSTATE_PUBLISH_OR_DDS_DELIVERY_STALL
```

## 직접 증명 / 미증명

직접 증명:

- getter client는 정확히 1회 요청했고 3.001261초에 timeout됐다.
- 해당 실행 전 60초 JointState baseline은 안정적이었다.
- 요청 구간부터 source timestamp 자체에 1.04/1.45/3.06초 gap이 생겼다.
- callback/0x0472 marker는 0개다.
- hardware read/RT data/state-interface update는 계속 진행했다.
- 이후 약 3.06초 feedback gap이 반복되어 완전 회복되지 않았다.

미증명:

- service request가 DDS/RMW에서 controller executor까지 어느 지점에서 정체됐는지
- broadcaster update callback이 실행되지 않은 것인지, publish 이후 DDS delivery가 정체된 것인지
- service-client discovery/creation이 controller-manager executor 또는 DDS를 block한 정확한 mutex/thread
- getter request와 stall 사이의 인과 메커니즘(시간적 재현 상관은 강하지만 내부 원인은 추가 계측 필요)

추가 getter, controller restart 및 자동 복구는 수행하지 않았다.

## 실행 횟수

- read-only `get_current_posx`: **1**
- getter success: 0
- getter timeout: 1
- Robot command: 0
- motion service/action: 0
- gripper/Home/trajectory: 0
- Hold/E-stop: 0
- controller/driver restart: 0
- AI action publish/closed-loop: 0
