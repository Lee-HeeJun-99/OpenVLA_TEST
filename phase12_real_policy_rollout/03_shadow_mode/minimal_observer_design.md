# Minimal JointState Observer 설계

## 목적

기존 `dsr_bringup2_rviz.launch.py`에서 startup 병목을 일으키는 `dsr_controller2`를 제외하고 다음 두 요소만 구성하는 진단용 launch 후보이다.

```text
dsr_hardware2 (ros2_control hardware plugin)
→ joint_state_broadcaster
→ /dsr01/joint_states
```

포함하지 않는 기능:

- `dsr_controller2`
- motion service/subscriber/action
- ActionAdapter
- DoosanBridge
- gripper command
- Home/trajectory/AI command
- RViz 및 불필요한 application node

## 중요한 안전 제한

이 launch는 command controller를 포함하지 않지만 완전한 read-only launch는 아니다. 현재 `DRHWInterface::on_init()`는 real robot 연결 과정에서 다음을 실행한다.

```cpp
Drfl.set_robot_control(CONTROL_SERVO_ON);
```

따라서 기본값 `operator_approved_hardware_initialization=false`에서는 시작 단계에서 거부한다. 현장 작업자의 별도 승인 전에는 실행하지 않는다.

## 기대 효과

- 거대한 `RobotController::on_activate()`를 호출하지 않음
- 수백 개 ROS/DDS endpoint를 생성하지 않음
- 두 spawner의 동시 `switch_controller` 경쟁 제거
- JointState data plane을 command controller와 분리해 검증 가능

## 검증 순서

1. Python syntax 및 launch description 정적 검사
2. unsafe capability 정적 검사
3. 현장 작업자 E-stop/servo 상태 확인
4. 별도 명시적 승인
5. 기존 driver가 완전히 종료된 상태 확인
6. minimal launch 1회
7. controller 상태 및 JointState 60초 측정
8. 60초 검사를 총 3회 반복

## 실행 상태

```text
IMPLEMENTED_NOT_EXECUTED
BLOCKED_REQUIRES_LOCAL_OPERATOR_AND_HARDWARE
```

정적 검증 결과:

- Python `py_compile`: PASS
- launch AST parse: PASS
- `dsr_controller2` spawner 제외: PASS
- ActionAdapter/DoosanBridge 제외: PASS
- operator gate 기본값 false: PASS
- joint_state_broadcaster 단독 spawner: PASS

환경에 `pytest` module이 없어 동일 test 함수 4개를 직접 로드하여 실행했으며 4/4 통과했다.

## 최초 승인 실행 결과 (2026-10-02 16:14 KST)

- `dsr_controller2`: 미기동
- hardware DRCF/RT 연결: 성공
- `CONTROL_SERVO_ON`: hardware 초기화 코드에 의해 로그상 5회 요청
- robot state: `STATE_SAFE_OFF`에서 `STATE_STANDBY`로 전환
- hardware initialize/configure/activate: 성공
- controller-manager update rate: 100 Hz
- joint_state_broadcaster load/configure/activate: 성공
- spawner: clean exit
- 기존 full launch에서 발생한 36~49초 switch timeout: 발생하지 않음
- 대량 microsecond catch-up/Skip flood: 발생하지 않음 (startup 1회 제외)

그러나 원격 PTY 실행 세션이 종료되면서 `ros2_control_node`도 유지되지 않았고, 60초 probe 결과 파일이 생성되지 않았다. launch log에는 driver crash, disconnect, segfault 또는 shutdown 원인이 기록되지 않았다. 따라서 이는 `REMOTE_EXEC_SESSION_TERMINATION_SUSPECTED`로 분류하며 JointState 60초 검증은 미완료다.

지속 baseline은 현장 터미널에서 launch를 foreground로 유지한 상태에서 별도 터미널로 측정해야 한다.

## 중복 launch 사건

최초 원격 launch가 실제로 PID `2915326`/`2915346`으로 계속 살아 있었지만 중간 점검에서 보이지 않는 것으로 잘못 판단됐다. 현장 launch PID `2915728`/`2915730`가 동일 `/dsr01` namespace로 추가 기동되면서 controller-manager가 두 개가 됐다.

- 현장 spawner가 기존 controller-manager의 active broadcaster를 보고 `Controller already loaded`를 출력
- 새 controller-manager에서는 broadcaster configure 실패
- JointState 100 Hz는 기존 PID `2915346`에서 정상 수신됨
- 기존 원격 세션은 Ctrl+C로 종료
- hardware deactivate/shutdown 및 connection close 후 vendor thread가 `Poco::SystemException`으로 abort
- 현장 PID `2915730`은 살아 있지만 broadcaster가 없어 `/dsr01/joint_states`는 없음

따라서 현장 launch도 정상 종료 후 단일 instance로 다시 시작해야 한다. controller-manager service를 이용한 동적 load/activate는 이번 안전 범위에서 수행하지 않는다.

## 단일 instance JointState 연속성 결과

clean-slate 후 단일 minimal observer PID `2916414`/`2916416`을 기동했다.

- controller-manager: 1개
- joint_state_broadcaster: 1개, 정상 활성화
- dsr_controller2: 없음
- 실제 JointState: 60.010초 / 6,002 messages
- source rate: 100.000015 Hz
- source max gap: 12.139 ms
- receive max gap: 12.113 ms
- 50/100/500/1000 ms 이상 gap: 모두 0
- duplicate/non-monotonic: 0/0
- invalid position/velocity: 0/0
- missing joint: 0
- effort NaN: unsupported로 처리
- command/publisher/service client created by probe: false/false/false

측정은 통과했지만 출력 파일명 충돌로 두 번째 결과의 신규 JSON 저장은 실패했다. 기존 동일 파일은 앞선 minimal observer에서 얻은 별도의 정상 60초 결과(6,002 messages, 100.001645 Hz, max gap 11.953 ms)이며 덮어쓰지 않았다. 두 측정 모두 실제 결과지만 두 번째는 console evidence로만 남았다.

초기 process 조회에서 PID가 일시적으로 보이지 않아 process 종료로 잘못 판정했지만, 사용자 foreground 터미널과 후속 host 조회에서 PID `2916414`/`2916416`이 계속 살아 있음이 확인됐다. 따라서 `PROCESS_LIFETIME_UNRESOLVED` 판정은 철회한다.

총 3회의 독립 60초 결과:

| Run | Messages | Source rate | Source max gap | >=100 ms | Invalid pos/vel |
|---|---:|---:|---:|---:|---:|
| 1 | 6,002 | 100.001645 Hz | 11.953 ms | 0 | 0 / 0 |
| 2 | 6,002 | 100.000045 Hz | 11.683 ms | 0 | 0 / 0 |
| 3 | 6,001 | 100.000014 Hz | 11.648 ms | 0 | 0 / 0 |

모든 run에서 50 ms 이상 gap, duplicate/non-monotonic timestamp 및 missing joint가 0이었다. 판정은 `MINIMAL_OBSERVER_JOINTSTATE_3X60S_PASS`다.
