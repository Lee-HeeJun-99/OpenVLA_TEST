# Doosan 계측 코드 build 검증

- 검증 일시: 2026-10-02 (Asia/Seoul)
- ROS 2 workspace: `/home/ubuntu/robot_ws`
- 범위: 전체 workspace 컴파일·링크·설치 검증만 수행
- 결과: **PASS**

## 실행 명령

```bash
source /opt/ros/humble/setup.bash
cd /home/ubuntu/robot_ws
colcon build --event-handlers console_direct+
```

실행 로그는 `/home/ubuntu/robot_ws/log/build_2026-10-02_14-06-47`에 보존돼 있다.

## 전체 결과

```text
Summary: 28 packages finished [12min 41s]
2 packages had stderr output: dsr_controller2 dsr_hardware2
exit code: 0
```

`stderr`는 기존 코드의 deprecated API, unused variable, VLA, class-memaccess 및 Boost/realtime_tools deprecation 경고다. `error:`, `undefined reference`, `collect2:`, `FAILED:` 또는 `CMake Error`는 세 대상 패키지 로그에서 발견되지 않았다. 계측 코드 수정을 요구하는 build error가 없었으므로 이번 단계에서 계측 소스는 추가 수정하지 않았다.

## 대상 패키지 검증

| 패키지 | 컴파일/링크/설치 | 검증 근거 |
|---|---|---|
| `dsr_common2` | PASS | 계측 헤더가 install space에 symlink됐고 `libDRFL.a` export 유지 |
| `dsr_hardware2` | PASS | `libdsr_hardware2.so` 생성, `DSR_TRACE`, `DRHWInterface::read`, `READ_DATA_RT_*` 문자열 포함 |
| `dsr_controller2` | PASS | `libdsr_controller2.so` 생성, `DSR_TRACE`, `get_current_posx_cb`, `GET_CURRENT_POSX_*` 문자열 포함 |

## 생성 산출물

```text
/home/ubuntu/robot_ws/install/dsr_common2/include/drfl_instrumentation.hpp
/home/ubuntu/robot_ws/install/dsr_common2/lib/libDRFL.a
/home/ubuntu/robot_ws/install/dsr_hardware2/lib/libdsr_hardware2.so
/home/ubuntu/robot_ws/install/dsr_controller2/lib/libdsr_controller2.so
/home/ubuntu/robot_ws/install/dsr_controller2/lib/libdsr_joint_trajectory.so
```

`dsr_common2`는 symlink-install 구조다. 실제 대상은 각각 source tree의 계측 헤더와 vendor archive이다. 생성된 shared library는 x86-64 ELF, dynamically linked, not stripped로 확인했다.

| 산출물 | 크기(bytes) | SHA-256 |
|---|---:|---|
| `libdsr_hardware2.so` | 1,480,296 | `56f5a23e90ab780bbf2abd77fbac6a5fe0d957d03f8d8719bb563f035c6b7a02` |
| `libdsr_controller2.so` | 101,818,600 | `5af7f81e8e7718a9764b523add42c302fb3ed0c17453ec1bf061360ce09e9eaf` |
| `libdsr_joint_trajectory.so` | 5,849,920 | `70a020690993ec494123ab63c67b65401dcded540a4fc930e24271d66b4d3aa7` |

## 안전 실행 감사

이번 단계에서 아래 항목은 모두 0회다.

- Robot command: 0
- getter/service 호출: 0
- motion service/action: 0
- gripper command: 0
- Home/trajectory: 0
- Hold/E-stop: 0
- controller restart/switch: 0
- driver restart: 0
- 계측 attach 및 baseline 재현: 0

빌드 성공 조건은 충족했다. 다음 단계인 driver 재시작 및 baseline 계측은 이 검증에 포함하지 않았으며 수행하지 않았다.
