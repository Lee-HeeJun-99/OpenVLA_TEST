# Gate·정지 시험 구조 오프라인 갱신

실기, ROS graph, 로봇/드라이버 접속 및 명령 발행 없이 읽기 전용 source만 조사했다.

## 두산 서비스 근거

| 기능 | 설정 이름 | type/필드 | source 근거 |
|---|---|---|---|
| 정지 | `/dsr01/motion/move_stop` | `dsr_msgs2/srv/MoveStop`: `int32 stop_mode → bool success` | `dsr_msgs2/srv/motion/MoveStop.srv:6-11`; service 생성 `dsr_controller2.cpp:2420`; callback이 DRFL `stop()`에 전달 `:745-748` |
| 관절 이동 | `/dsr01/motion/move_joint` | `dsr_msgs2/srv/MoveJoint`: pos[6] deg, vel deg/s, acc deg/s², time, radius, mode, blend_type, sync_type → success | `MoveJoint.srv:6-15`; service 생성 `dsr_controller2.cpp:2407`; Python request mapping `DSR_ROBOT2.py:2580-2590` |

MoveStop 상수는 `DR_QSTOP_STO=0`, `DR_QSTOP=1`, `DR_SSTOP=2`, `DR_HOLD=3`이다(`DSR_ROBOT2.py:272-279`; `MoveStop.srv:6-9`). 설정 후보는 **DR_QSTOP=1**로 고정했다. 일반 stop API 기본값도 QUICK이다(`dsr_robot.h:196`). 다만 안전 특성과 현장 운용 적합성은 **[미확인: 현장 검토]**이며 값을 임의 확정하지 않는다.

## 모드와 gate

| 모드 | 선행 evidence | 불필요한 설정 |
|---|---|---|
| dryrun_live | 없음; read-only | Servoj parameter, tracking threshold |
| gripper_test | pendant limits, live topics, gripper wiring, arm stationary | Servoj parameter, tracking threshold |
| stop_test_movej | pendant limits, live topics, feedback rate | Servoj parameter, tracking threshold |
| stop_test_servoj | stop_movej 결과 + pendant/live/rate + Servoj parameter + tracking calibration | — |
| synthetic/policy | `stop_servoj_verified` + pendant/live/rate + Servoj parameter + tracking calibration | — |

`gripper_io_verified`는 gripper_test의 입력 gate가 아니라 결과 review gate다. `stop_movej_verified` 역시 정지 서비스와 판정 로직만 검증하며 Servoj stream 정지 검증을 대신하지 않는다.

현장 확인 결과 RobotState topic이 없으므로 이 계획은 정정됐다. `stop_test_movej`는 모든 JointState를 이름 기반 정규화하여 host/header 시각과 함께 기록하고, `get_robot_state`를 재폴링한다. FK↔controller flange 비교는 저주기 `get_current_tool_flange_posx/get_current_rotm` 로그를 사용한다. **출처:** `reports/live/20261113_091859/feedback_selftest.md`.
