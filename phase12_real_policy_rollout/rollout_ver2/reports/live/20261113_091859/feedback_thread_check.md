# Feedback 전용 thread 읽기 전용 점검

## 조건

- 일시: 2026-11-13
- 측정 시간: 30.189 s
- main thread에서 0.45 s blocking을 67회 반복해 HTTP inference blocking을 모사했다.
- 허용된 `/dsr01/joint_states`, `/dsr01/error`, `/dsr01/robot_disconnection`, ZED image 구독과 get_* service만 사용했다.
- publisher, set_*, move_*, Servoj/MoveJoint/MoveStop/gripper 객체를 만들지 않았다.
- 원본: `feedback_thread_check.json`; 도구: `tools/feedback_thread_check.py`.

## 결과

| 항목 | 결과 |
|---|---:|
| JointState 수신 수 | 3,027 |
| JointState age p50 | 5.241 ms |
| JointState age p99 | 10.101 ms |
| JointState age max | 10.616 ms |
| robot_state polling | 303/303 성공 (100%) |
| robot_state latency p50 | 0.995 ms |
| robot_state latency p99 | 1.950 ms |
| robot_state latency max | 794.365 ms |
| 종료 시 robot_state age | 83.491 ms |
| Camera 수신 수 | 510 |
| Camera interval p50 | 34.059 ms |
| Camera interval p99 | 233.647 ms |
| Camera interval max | 633.917 ms |
| Camera gap >0.1 s / >1 s | 98 / 0 |

robot_state service의 단일 최대 latency 794 ms가 관찰됐지만 요청 303건은 모두 성공했고 종료 시 state age는 83.5 ms였다. 이는 숨기지 않고 원본 event에 보존한다. state freshness는 기존 runtime 감시가 계속 적용한다.

## 판정

- **통과:** main thread가 0.45 s씩 막혀도 JointState age max는 10.62 ms로 0.1 s 기준보다 충분히 작았다. feedback callback이 main loop와 독립적으로 갱신됨을 확인했다.
- Camera gap은 계속 log-only다. 전용 camera thread에서도 이 측정 구간은 p99 233.6 ms였으므로 post-settle 새 frame barrier를 유지한다.
- `JOINT_STATE_STALE=0.1 s`는 완화하지 않는다.
