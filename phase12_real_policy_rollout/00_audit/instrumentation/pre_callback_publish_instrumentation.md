# Service pre-callback / JointState publish 추가 계측

일자: 2026-10-02

## 목적

단발 `get_current_posx` 요청이 `get_current_posx_cb`에 도달하지 않았고, hardware read가 계속되는 동안 JointState 전달이 정체된 결과를 다음 경계에서 분리하기 위한 계측이다.

1. controller-manager controller update loop
2. JointState broadcaster update 및 pre-publish
3. RCL/RMW service request take
4. RCL/RMW publish

## 정적 구조 확인

- `get_current_posx`를 포함한 일반 Doosan service는 callback group을 명시하지 않아 controller lifecycle node의 default callback group을 사용한다.
- 별도 `MutuallyExclusive` callback group `cb_group_`는 현재 `motion/move_stop` service에만 전달된다.
- `dsr_controller2::RobotController::update()` 본문은 원래 즉시 `OK`를 반환하는 no-op이었다.
- `joint_state_broadcaster` source는 workspace에 없으며 `/opt/ros/humble/lib/libjoint_state_broadcaster.so` 바이너리 패키지로만 설치돼 있다.
- 설치 버전: `2.53.1-1jammy.20260505.183509`.
- 바이너리에는 `JointStateBroadcaster::update`, JointState `RealtimePublisher::publishingLoop`, `pthread_mutex_trylock`, `rcl_publish` 심볼이 존재한다.
- active RMW는 process map 기준 Fast DDS (`librmw_fastrtps_cpp.so`)이다.

## 작성한 계측

### dsr_controller2 내장 계측

`RobotController::update()`에 rate-limited `SampledScope`를 추가했다. 기본 `DSR_TRACE_HIGH_RATE_EVERY=100`에서 약 1초마다 controller-manager가 이 controller update에 도달했는지 기록한다.

### 동적 uprobe 계측 초안

`doosan_vendor_trace.bt`에 다음 경계를 추가했다.

- `JointStateBroadcaster::update` ENTRY/EXIT
- 기존 binary offset 기반 `JOINT_STATE_PRE_PUBLISH`
- `rcl_take_request_with_info` ENTRY/EXIT
- Fast DDS `rmw_take_request` ENTRY/EXIT
- `rcl_publish` ENTRY/EXIT
- Fast DDS `rmw_publish` ENTRY/EXIT

각 event는 monotonic timestamp, TID, handle/pointer 및 반환 code를 남긴다. 이는 service request가 middleware에서 take됐는지와 broadcaster update 이후 publish가 반환되는지를 구분한다.

## 빌드 검증

```text
colcon build --packages-select dsr_controller2 --event-handlers console_direct+
```

- 결과: PASS
- package: 1/1 성공
- build time: 약 3분 31초
- 오류: 없음
- 기존 deprecated/unused warning: 존재
- 설치 library SHA-256: `3bea4202d7baf1a35c63d6c933b2baa008e8039b34a0100a9bf7a140901df331`

## 제한과 blocker

- 이 호스트에는 `bpftrace` 실행 파일이 설치돼 있지 않아 `.bt` 스크립트의 parser/load 검증은 수행하지 못했다.
- `/opt/ros/humble`의 JointState broadcaster source를 직접 수정하지 않았다.
- binary offset `+0x820` probe는 현재 설치 버전에 종속되므로 재사용 전 symbol/version 확인이 필요하다.
- 현재 실행 중인 PID `2906371`은 build 전에 적재된 controller library를 계속 사용한다. 새 내장 update 계측은 다음 재launch 이후에만 적용된다.
- 현재 driver는 getter 재현 뒤 약 3초 feedback gap이 반복되는 비정상 상태다. 이 단계에서는 상태 보존을 위해 재시작하지 않았다.

## 안전 실행 감사

- Robot command: 0
- getter/service: 0
- motion/gripper/Home/trajectory: 0
- Hold/E-stop: 0
- controller/driver restart: 0
- AI action publish: 0

## bpftrace 설치 시도

사용자 승인 후 다음 설치를 시도했다.

```bash
sudo apt-get update
sudo apt-get install -y bpftrace
```

원격 세션에서 `sudo`가 TTY password 입력을 요구해 설치 전에 종료됐다. 패키지 변경은 발생하지 않았다. 비밀번호를 자동 전달하거나 우회하지 않았다. 현장 터미널에서 작업자가 직접 설치한 뒤 `bpftrace --version`을 확인해야 한다.

## 현장 설치 후 검증

- 설치 확인: `/usr/bin/bpftrace`
- 버전: `bpftrace v0.14.0`
- 이 버전은 probe listing 및 dry-run도 root 권한을 요구한다.
- 원격 세션의 `sudo -n`은 `a password is required`로 종료되어 실제 attach/load 및 parser dry-run은 수행하지 않았다.
- 정적 symbol validation은 별도로 수행했으며 `.bt`에 선언된 uprobe/uretprobe 37개가 참조하는 모든 library와 base symbol이 존재했다: `37 PASS / 0 FAIL`.

따라서 남은 검증은 현장 sudo 터미널에서 `bpftrace -d`를 실행하는 것이다. 이는 probe를 attach하거나 getter를 호출하지 않는 parser/IR dry-run이다.

## 저부하 pipeline 계측 dry-run

고빈도 정상 경로를 event별 출력하지 않고 1초 단위 counter/last-timestamp로 집계하는 `doosan_pipeline_low_overhead.bt`를 추가했다.

- getter/vendor command 및 RCL/RMW service take: event-level 기록
- RT read, TCP/UDP receive, JointState broadcaster update/pre-publish, RCL/RMW publish: 1초 요약
- 대상 base symbol 정적 검사: 18 PASS / 0 FAIL
- 현장 `sudo bpftrace -d`: `exit_code=0`
- LLVM IR 생성 완료

따라서 low-overhead script는 parser/IR dry-run을 통과했다. 아직 실제 process attach는 수행하지 않았다.

## 계측 build 적용 재launch 후 baseline 재검증 (2026-10-02 15:37 KST)

- launch PID: `2911055`
- `ros2_control_node` PID: `2911070` (관찰 시 CPU 약 200%)
- runtime `DSR_TRACE`: 활성 (`dsr_controller2::RobotController::update` ENTRY/EXIT 확인)
- `/dsr01/joint_states`: publisher endpoint 1개 확인
- 실제 JointState 수신: **0 messages / 10초**
- `dsr_controller2` spawner: `switch_controller` 10초 timeout 반복 후 exit code 1
- launch log: `/home/ubuntu/.ros/log/2026-10-02-15-37-44-744350-scps1-2911055/launch.log`
- `dsr_hw_interface2::write()`에서 controller-manager 제공 `dt`가 약 11~30 us로 붕괴한 `[REAL] Skip dt` 경고가 대량 발생했다. 설정상 기대 주기는 10 ms이고 허용 범위는 3~15 ms다.

`RobotController::update` trace가 출력된다는 사실은 새 라이브러리가 적재됐음을 보이지만, publisher endpoint만 존재하고 실제 JointState 데이터는 전달되지 않았다. 따라서 분류는 `RELAUNCH_BASELINE_FAILED_CONTROL_LOOP_DT_COLLAPSE`이며, getter 재현과 BPF 실부착의 사전 안전 gate를 통과하지 못했다.

이번 확인에서 getter/service 호출, BPF attach, robot command, motion, gripper, Home, trajectory, Hold/E-stop 및 AI action publish는 모두 0회다.
