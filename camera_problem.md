# A0509 현실 rollout 실험 정리 — 2026-11-09

## 문서 범위와 판정 기준

- 저장소: `Lee-HeeJun-99/OpenVLA_TEST`, 브랜치: `lhj-research`.
- 작성 시 확인한 HEAD: `656a6feb060a660a45f3cb1c802fe50c76599c5d`. 아래 실험들이 모두 이 HEAD에서 실행됐다는 의미는 아니다. 각 실행의 `config_snapshot.json`, 명령 기록과 분석 provenance를 우선한다.
- 주요 근거: 2026-10-14 및 2026-11-05~06의 저장된 실험·오프라인 분석 결과. 11월 9일 새 물리 실험 결과가 아니다.
- 이 문서 작성에서는 서버·ROS·로봇·그리퍼를 실행하지 않았고, 기존 코드·설정·로그를 변경하지 않았다.
- **정상 runtime 종료, arm 통합 테스트 PASS, 실제 task 성공을 구분한다.** `task_success=null`, `OPERATOR_RESULT_CONFIRMATION_PENDING`은 cube pick 성공으로 세지 않는다.
- 측정 trajectory는 별도 명시가 없으면 `/dsr01/joint_states`의 실제 관절값으로 계산한 nominal URDF flange FK이다. 독립적인 TCP 위치 센서 측정이 아니다.

기준 디렉터리:

```text
/home/ubuntu/a0509_vla_linux_field_bundle_20260903/lhj/
└─ phase12_real_policy_rollout/
   └─ 14_real_rollout_integration/
      └─ real_trials/                  # 아래 실험 ID에 해당하는 디렉터리
```

## 요약

| 항목 | 확인된 결과 | 남은 문제 |
|---|---|---|
| Camera | 케이블 변경 직후 180초 추론 측정에서 500ms 이상 공백 0회 | 이후 수신 저하 재발. 재부팅 후에도 500~900ms 공백 관측. 원인 미확정 |
| MoveL | 1-step 실제 방향·크기 PASS, 70초 arm-only PASS, 30초 full-policy runtime 정상 | ACK 성공인데 이동하지 않는 NOT REACHABLE 사례. target 도달 검증 부족 |
| ServoL | 150 predictions / 약 70.8초 runtime 정상 및 종료 Stop ACK 성공 | 후속 두 실행에서 J5 한계 초과·SAFE_OFF2·Stop ACK 실패. 가속도/시간·보간·feedback·IK 문제 미해결 |
| Task | arm 경로 및 일부 gripper sequence의 실제 동작 근거 있음 | 이 문서에서 확인한 대표 로그만으로 orange cube grasp+lift SUCCESS를 확정하지 않음 |

## 1. Camera: 교체 직후 개선, 이후 수신 공백 재발

### 1.1 측정 조건과 구분

- 토픽: `/zed/zed_node/rgb/color/rect/image`, 영상: `1280×720`, `bgra8`.
- ZED 진단의 설정 grab rate는 30Hz. 아래 평균 Hz는 **독립 구독자가 실제 수신한 영상 기준**이다.
- A: 서버는 로딩된 채 대기, 카메라 구독만 수행. SDK-only 또는 서버가 꺼진 조건은 아니다.
- B: fresh 실제 영상으로 서버 추론을 반복.
- C: 실제 추론 + JPEG/PNG 저장·JSONL/fsync·read-only 로봇 상태 조회·settle 검사. **물리 dispatch, 모터/그리퍼 동작, 전체 physical Controller 검사는 포함하지 않는다.**
- 독립 모니터는 image와 camera_info의 수신/소스 timestamp, ZED diagnostics, host 기록을 저장한다. SDK 진단은 이동 집계값이며 개별 `grab()` 호출 timestamp가 아니다.

### 1.2 케이블 변경 시점 비교

동일하게 추론 부하가 있는 B 조건 중심의 비교이다. 순차 실험이며 케이블 외 모든 변수가 고정된 인과 A/B 실험은 아니다.

| 시점 / 실험 ID | 연결에 대한 당시 기록 | 시간 | 평균 수신 Hz | 최대 receive gap | receive gap ≥500ms |
|---|---|---:|---:|---:|---:|
| `20261105_163056_camera_inference_cpu_audit` | 교체 전 진단 구간 | 60초 | 10.50 | 1,235ms | 8회 |
| `20261105_181006_camera_inference_cpu_audit` | C-to-C direct, `2-1`, 5000Mbps | 180초 | 19.16 | 330ms | 0회 |
| `20261106_094617_camera_inference_cpu_audit` | 사용자 확인: 내부 중계칩이 있는 새 케이블 | 180초 | 21.84 | 300ms | 0회 |
| `20261106_155713_current_camera_three_cases` | 이후 재발 측정; 케이블 동일성 자체는 새로 입증하지 않음 | 120초 | 11.38 | 1,103ms | 14회 |

해석: 교체 직후 개선은 실제 측정으로 지지된다. 하지만 영구 해결은 아니다. 이후 다시 악화됐다는 사실만으로 새 케이블 불량, CPU 부족 또는 코드 변경 중 하나를 원인으로 확정할 수 없다.

### 1.3 3조건 재측정과 재부팅 결과

각 행은 120초 관측이다. 500ms/1초 횟수는 receive gap 기준이다.

| 측정 세션 | 조건 | 평균 Hz | 최대 공백 | ≥500ms | ≥1초 |
|---|---|---:|---:|---:|---:|
| 11/6 15:57 | A 서버 대기 | 10.25 | 967ms | 25 | 0 |
| 동일 | B 추론 | 11.38 | 1,103ms | 14 | 1 |
| 동일 | C 액션 준비 | 8.98 | 1,100ms | 13 | 1 |
| 11/6 16:20 | A 서버 대기 | 10.20 | 898ms | 9 | 0 |
| 동일 | B 추론 | 14.48 | 736ms | 1 | 0 |
| 동일 | C 액션 준비 | 14.87 | 501ms | 2 | 0 |
| 11/6 17:12 재부팅·서버 재기동 후 | A 서버 대기 | 9.68 | 901ms | 9 | 0 |
| 동일 | B 추론 | 13.74 | 634ms | 2 | 0 |
| 동일 | C 액션 준비 | 17.08 | 558ms | 1 | 0 |

- 15:57 C에서 `validate_action_latency.py`의 `camera_stream_loss`가 실제 재현됐다. 이후 16:20 및 17:12 세션의 해당 probe들에는 같은 오류가 기록되지 않았다.
- 17:12 C는 표본 80개가 기록됐고 모든 probe summary가 PASS였다. 마지막 bounded probe는 독립 카메라 120초 window 뒤에도 끝까지 실행될 수 있으므로, 80개 전부가 정확히 같은 120초 안에 수집됐다고 해석하지 않는다.
- 재부팅 후 추론 latency 중앙값: B 약 389ms, C 약 400ms. B 최대는 약 1.028초로 첫 요청 등을 포함한 최대값이며, 전부 400ms였다는 뜻은 아니다.
- 이번에 1초 공백이 없었다고 장시간 physical rollout 안정성 PASS로 확대하지 않는다.

### 1.4 버퍼·서버 uptime·yes 제거에 대한 결론

**수신 큐에 오래된 영상이 계속 쌓이는 전형적 패턴은 확인되지 않았다.**

- 15:57 세션 수신 영상 source age 중앙값은 조건별 약 40~47ms, 최대 약 64ms였다.
- 약 1.1초 끊긴 뒤 첫 영상도 약 42~49ms 전의 영상이었다. source timestamp도 약 1.1초 건너뛰었다.
- 시간이 지나면서 source age가 누적 증가하지 않았다. 프로그램의 camera callback은 최신 영상 한 개를 덮어쓴다.
- 이는 backlog 재생보다 중간 프레임 누락/획득·발행 정체 후 최신 프레임 수신에 가깝다. 단, SDK/USB 내부 정체 후 drop까지 배제한 것은 아니다.

**추론 때문에만 끊긴다는 증거는 없다.** A에서도 공백이 있고, 일부 세션은 B/C가 오히려 더 양호했다. ZED diagnostics에도 capture rate 저하와 frame drop이 보였다. CPU 스케줄링, GPU 경합, SDK/USB/카메라 경로 중 단일 원인은 미확정이다.

**서버 장기 실행만으로 설명되지 않는다.** 11/6 재부팅 후 새 서버에서도 수신 저하가 남았다. 재부팅은 여러 상태를 동시에 바꾸므로 특정 요소의 인과 실험도 아니다.

**yes 제거 이후 재발했다는 시간 순서는 있으나, 원인 확정은 아니다.** 변경은 단순 입력 삭제가 아니라 OPEN 실물 확인 대기를 ACK 기반 자동 진행으로 바꾼 것이다. 그러나 standalone 진단에서도 재발했으며, 수정 전/후 코드를 같은 환경에서 직접 대조한 검사는 수행하지 않았다. 기존 14:52/15:41 trial snapshot에서 `rollout_latency.py`, `camera_inference_timing.py` hash는 같았다.

이전 launch 두 개(PID 57201, 63094)는 자식 state publisher만 남아 있었으며 당시 image publisher는 1개였다. 종료 후에도 공백이 남았고, ZED 세션도 바뀌었으므로 중복 launch가 원인이었다고 결론내리지 않는다.

### 1.5 판정 기준과 다음 확인 사항

- 선택 프레임 freshness: **inference 시작 시 age <0.5초**. inference 종료 age는 분석용이다.
- 별도 stream silence watchdog: 마지막 callback 이후 **≥1.0초**이면 종료. 앞선 요청으로 0.5→1.0초 변경된 것으로, 카메라 안정화 조치가 아니다.
- camera source age와 receive silence는 다르다. 도착한 영상이 fresh여도 그 사이 무수신 공백으로 watchdog이 발동할 수 있다.
- 다음 원인 분석은 capture/publish/subscribe timestamp와 CPU/GPU·USB 오류를 같은 시각에 연계해야 한다. 현 로그만으로 USB 버퍼 확대나 timeout 추가 완화를 해결책으로 확정하지 않는다.

근거:

- [C-to-C 측정](phase12_real_policy_rollout/14_real_rollout_integration/real_trials/20261105_181006_camera_inference_cpu_audit/comparison.json)
- [새 중계칩 케이블 측정](phase12_real_policy_rollout/14_real_rollout_integration/real_trials/20261106_094617_camera_inference_cpu_audit/comparison.json)
- [15:57 3조건](phase12_real_policy_rollout/14_real_rollout_integration/real_trials/20261106_155713_current_camera_three_cases/analysis.json)
- [16:20 3조건](phase12_real_policy_rollout/14_real_rollout_integration/real_trials/20261106_162035_current_camera_three_cases/analysis.json)
- [재부팅 후 17:12 3조건](phase12_real_policy_rollout/14_real_rollout_integration/real_trials/20261106_171214_current_camera_three_cases/analysis.json)

## 2. MoveL 방식 rollout

### 2.1 명령 흐름과 끊겨 보이는 이유

```text
fresh image → OpenVLA → canonical translation(m)/rotation(rotvec rad)
→ safety / workspace → BASE absolute Cartesian target(mm / ZYZ deg)
→ /dsr01/motion/move_line → 동기 service 응답 → feedback/settle → 다음 관측
```

- 기존 실행은 추론·로깅·상태 조회·동기 이동·settle이 순차적이다. 설정상 5Hz 값이 있어도 실제 새 명령이 정확히 5Hz로 나온다는 의미가 아니다.
- 예: 14:52 실행은 약 279.9초에 117개 arm 요청으로, episode 전체 평균은 약 0.42요청/초다. 전체 평균에는 pre/post 처리와 중단 구간이 포함된다.
- 각 endpoint 이동 후 다음 action을 기다리는 stop-and-go 특성이 있다. speed를 올리는 것만으로 연속 추종 구조가 되지 않는다.
- **서비스 ACK 성공과 실제 target 도달은 별개**다. 로그의 `steps_completed` 역시 독립적인 도달 측정으로 해석하면 안 된다.

### 2.2 현재 파일 기준 MoveLine 설정과 변경 이력

| 설정 | 현재 파일 값 / 의미 |
|---|---|
| translation velocity / acceleration | 120mm/s / 120mm/s² |
| rotation velocity / acceleration | 20deg/s / 20deg/s² |
| MoveLine delta limiter | OFF; 허용 raw arm delta 전체를 endpoint로 사용 |
| MoveLine raw translation hard rejection | OFF; 이전 50mm 거부 검사 해제 |
| rotation gate | 4deg 유지 |
| workspace / finite / runtime gates | 유지 |
| episode cap | 500초 / 150 actions |
| launch 기본 | MoveLine direct mode; 수동 yes/실제 OPEN 확인 대기 없음 |

이동 hard gate는 13→20→50mm 등의 단계와 MoveLine 전용 해제 과정을 거쳤다. 초반 정상 실험과 현재 설정은 같지 않다. default/Servo/legacy의 50mm 검사를 MoveLine 전용 OFF와 혼동하지 않는다. delta limiter를 끈 MoveLine에서는 이전 canonical delta/dt 기반 동적 검사도 controller profile 담당으로 구분되며, 모든 과거 동적 제한이 그대로 적용되는 것은 아니다.

현재 설정의 120/120 profile에는 파일상 물리 검증 미완료 경고가 남아 있다. 이후 해당 값으로 실험한 로그가 존재하더라도 전체 도달 가능성·안전성 검증 완료를 의미하지 않는다.

기본 YAML의 `motion_enabled=false`만 보고 launcher가 무물리 검사라고 판단하면 안 된다. 승인/실행 모드를 통해 실제 sink가 생성될 수 있다. 본 문서는 실행 지시서가 아니다.

### 2.3 주요 실행 결과

| 실험 ID | 기록된 결과 | 시간 / completed | 핵심 해석 |
|---|---|---|---|
| `20261014_arm_only_short_horizon_1step_real_01` | `ARM_ONLY_SHORT_HORIZON_PASS` | prediction 1, arm 1 | 실제 방향·크기와 operator 결과 확인까지 있는 기준점 |
| `20261014_arm_only_70s_episode_001` | `ARM_ONLY_FULL_EPISODE_PASS` | 70.385초 / 25 | gripper 0, operator 이상 motion·E-stop 없음. task 평가 아님 |
| `20261105_161811_full_policy_episode` | `OPERATOR_RESULT_CONFIRMATION_PENDING`, runtime valid | 30.500초 / 10 | 정상 duration 종료, arm 10, gripper IO writes 6. task 성공 미확정 |
| `20261106_101231_full_policy_episode` | `INVALID: raw_translation_step_limit` | 260.692초 / 83 | 84번째 시도에서 gate 종료. 카메라 장시간 개선 근거지만 episode PASS 아님 |
| `20261106_132009_arm_only_moveline_episode` | `OPERATOR_ABORT` | 222.274초 / 101 | ACK 후 무이동 및 NOT REACHABLE가 확인된 실패 분석 기준 |
| `20261106_145225_full_policy_episode` | `OPERATOR_ABORT` | 279.913초 / 116 | arm 117, gripper IO writes 8. 정상 장시간 이동 성공 사례로 선정하지 않음 |
| `20261106_153541_full_policy_episode` | `INVALID: workspace_violation` | 130.003초 / 42 | workspace gate 종료 |
| `20261106_154158_full_policy_episode` | `INVALID: camera_stream_loss` | 80.927초 / 25 | arm 26, stop 1; yes 제거 후 사례. 전체 PASS 아님 |

gripper command 수는 개별 digital IO write 수이며 OPEN/CLOSE 왕복 횟수가 아니다. 초기화와 abort OFF도 포함될 수 있다.

### 2.4 실제 검증된 1-step 방향·크기

| 항목 | 요청 | 실제 관절 기반 flange FK |
|---|---:|---:|
| translation norm | 0.800mm | 0.806688mm |
| rotation angle | 0.642879deg | 0.641841deg |

- translation 벡터 오차 norm: 0.007924mm. norm끼리의 차이(약 0.006688mm)와 구분한다.
- direction cosine: 0.999986, ACK 약 556ms.
- JointState ≥100ms event 0, gripper 0, hold/stop 0.
- 최종 trial summary와 operator confirmation은 예상 motion, gripper 이상 없음, E-stop 미사용을 반영한다. 중간 `feedback_comparison.json`에 남은 confirmation PENDING 문구만으로 최종 상태를 덮어쓰지 않는다.
- raw model translation은 약 1.435mm였고 당시 limiter를 거쳐 0.8mm가 전달됐다. 이 PASS를 limiter OFF/120mm/s의 현재 설정 검증으로 옮기지 않는다.

### 2.5 ACK는 오지만 멈춰 있던 구간

기존 timestamp 연계 분석에서 `20261106_132009_arm_only_moveline_episode`의 steps 43~100은 상승 target을 반복 요청하고 ACK가 성공해도 FK Z net 변화가 약 +0.000792mm에 불과했다. 해당 58개 요청과 `1206 NOT REACHABLE` 경고가 연계됐다.

- 50mm 초과 거부와 구분해야 한다. gate 거부면 dispatch 전에 종료해야 하며, ACK 후 무이동은 다른 문제다.
- 도달 불가능 경고는 근거가 있지만 정확한 IK branch·특이점·관절 한계 기여를 한 원인으로 확정하지 않는다.
- 현재 검증 공백은 controller alarm 수집, requested target 대비 실제 pose 변화/도달 검사, 무이동 연속 발생 시 inhibit이다.

또한 과거 limiter가 켜진 실행에서는 이전 속도를 유지하며 감속하는 과정에서 raw -Z / delivered +Z가 가능했다. 예: raw -0.360211mm → submitted +0.602644mm → observed +0.573479mm. 이는 target 변환에서 Z 부호가 일괄 뒤집힌 증거가 아니다. 현재 MoveLine limiter OFF와 과거 로그를 구분해야 한다.

근거:

- [MoveLine 설정](phase12_real_policy_rollout/14_real_rollout_integration/configs/hardware_interface.yaml)
- [1-step 최종 판정](phase12_real_policy_rollout/14_real_rollout_integration/real_trials/20261014_arm_only_short_horizon_1step_real_01/trial_summary.json), [실제 feedback 비교](phase12_real_policy_rollout/14_real_rollout_integration/real_trials/20261014_arm_only_short_horizon_1step_real_01/feedback_comparison.json)
- [70초 arm-only](phase12_real_policy_rollout/14_real_rollout_integration/real_trials/20261014_arm_only_70s_episode_001/trial_summary.json)
- [30초 full-policy 정상 runtime](phase12_real_policy_rollout/14_real_rollout_integration/real_trials/20261105_161811_full_policy_episode/trial_summary.json)
- [Z 방향·무이동 분석](phase12_real_policy_rollout/14_real_rollout_integration/real_trials/20261106_z_direction_audit_02/findings.json)

## 3. ServoL 방식 rollout — 가속도·목표 시간·관절 한계

### 3.1 구조와 MoveL과의 차이

```text
fresh observation → OpenVLA → 새 absolute target
→ 기존 velocity/acceleration 제약을 가진 보간기
→ 20Hz /dsr01/servol_stream
→ 실제 로봇 tracking
→ 종료 시 기존 MoveStop HOLD(3), ACK 확인
```

- 추론은 새로운 target을 갱신하고, 20Hz stream은 그 사이 보간된 pose를 보낸다. **20Hz는 inference 속도가 아니다.**
- stream publish 함수가 돌아온 것은 vendor ACK나 실제 위치 도달이 아니다. 확인된 driver callback은 DRFL boolean return을 상위 ACK로 전달하지 않는다.
- 마지막 source timestamp를 매 tick마다 새로 만들면 안 된다. 현재 보간기는 원래 source age를 유지한다.
- 전체 trajectory와 stop 성능의 formal 검증이 완료된 production controller로 표현하지 않는다.

### 3.2 대표 실험

| 실험 ID | 시간 | predictions / targets | publish 반환·시도 수 | 종료와 Stop |
|---|---:|---:|---:|---|
| `20261106_110547_arm_only_servo_episode` | 70.822초 | 150 / 150 | 1,379 | prediction cap 정상 종료, runtime valid, HOLD ACK true, final STANDBY |
| `20261106_111406_arm_only_servo_episode` | 58.132초 | 124 / 124 | 1,131 | `robot_abnormal_state:10`, runtime invalid, HOLD ACK false |
| `20261106_125319_arm_only_servo_episode` | 53.195초 | 135 / 135 | trial 내 stream 기록 참조 | 같은 state 10 종료, runtime invalid, HOLD ACK false |

첫 실행의 기록상 status는 `ARM_ONLY_SERVO_EPISODE_OPERATOR_RESULT_PENDING`이다. runtime이 정상이고 마지막 stop이 성공한 상대적으로 좋은 기준점이지, operator 결과나 task 성공까지 확정된 PASS는 아니다. 첫 실행은 gripper 0, MoveLine 0, JointState max source 12.612ms / receive 19.605ms였다.

추론 cap을 150→2000으로 올린 후 실제로 2000개까지 도달한 것이 아니다. 두 실패는 각각 124/135개에서 끝났다.

### 3.3 관절 한계와 stop 실패

driver alarm 9008 기록:

| 실행 | controller가 보고한 J5 | controller minimum |
|---|---:|---:|
| 11:14 실행 | -137.500deg | -135.000deg |
| 12:53 실행 | -137.501deg | -135.000deg |

- state 10은 기존 분석에서 SAFE_OFF2로 식별됐다. 단순 JointState 지연 때문에 중단된 사례로 분류하지 않는다.
- Cartesian workspace 내부라도 IK 관절 자세가 허용 범위라는 보장은 없다.
- 당시 runtime의 `joint_limits_ok`는 기본 true로 남아 실측 한계/target IK를 평가하지 않았다. state 검사는 prediction 단위라 한계 도달 전 예방 검사와 다르다.
- Stop ACK false는 중대한 미검증 사항이다. 이미 SAFE_OFF2가 된 결과인지 독립 stop 구현 결함인지까지 단정하지 않는다. 첫 정상 실행의 Stop ACK true가 후속 실패를 상쇄하지 않는다.

### 3.4 가속도와 목표 시간 문제

기존 transport profile은 velocity `[20,20]` (mm/s, deg/s), acceleration `[20,20]` (mm/s², deg/s²), target time 0.1초였다.

11:14 실패 실행의 기존 offline 분석은 대응 가능한 123쌍을 대상으로 다음을 기록했다.

- command/feedback translation 차이: 중앙값 1.751mm, p95 3.187mm.
- rotation 차이: 중앙값 0.703deg, p95 2.098deg.
- 0.1초의 전체 시간 budget을 허용해도 speed-only budget 초과가 53쌍.
- 정지 출발·정지 도착의 이상적 가감속 reference에서는 필요한 시간 중앙값 0.596초, p95 0.802초, 최대 0.852초.
- driver warnings에 1215, 1216이 포함됐다. 이것만으로 모든 한계 초과의 유일 원인을 가속도라고 확정하지 않는다.

위 reference는 실제 움직이는 closed-loop 초기 속도, IK, 특이점, 관절 제약을 시뮬레이션하지 않는다. command/feedback의 정확한 측정 시각도 일부 부족하다. 따라서 값을 높이면 해결된다는 증명이 아니다.

두 실행의 272쌍을 이용한 offline sweep:

| transport candidate | 이상적 rest-to-rest 시간 중앙값 / p95 / 최대 |
|---|---|
| velocity 20/20, acceleration 20/20 | 0.583 / 0.832 / 0.899초 |
| velocity 20/20, acceleration 40/40 | 0.412 / 0.588 / 0.636초 |

현재 `configs/servo_episode.yaml`에는 velocity `[20,20]`, acceleration `[40,40]`, target time **0.7초**, cap 2000이 반영돼 있다. 이는 **offline candidate이며 파일의 physical_validation은 NOT_EXECUTED**이다. sweep 결과의 `production_parameters_changed=false`는 당시 offline 분석 작업에 대한 기록이고, 이후 파일 반영 여부와 구분해야 한다. Servo planning limiter 자체는 transport 가속도 40과 같은 값으로 자동 바뀐 것이 아니다.

### 3.5 보간·feedback 문제: 가속도만 높이면 안 되는 이유

기존 source/offline audit에서 확인된 사항:

1. 보간기의 virtual pose가 실제 CurrentPosX와 별도로 유지되고, following-error guard가 없다.
2. 움직이는 동안 추론이 끝난 뒤 current pose를 읽어 target을 구성하므로 observation/action/pose의 시점 차이가 있다.
3. CurrentPosX의 첫 6개 값만 사용하고 solution-space 값은 버린다. Cartesian orientation 연속성이 IK branch 연속성을 보장하지 않는다.
4. 고정 target 0.8mm의 합성 시험에서 virtual command 최대 1.0mm, overshoot 약 0.2mm, 7.5초 동안 target crossing 47회가 관측됐다.

4번은 **로봇을 움직이지 않은 합성 characterization**이다. 이것이 실물 J5 한계 초과의 직접 원인이었다고 단정하지 않는다. 다만 보간 안정성·tracking·IK 검증 없이 speed/acceleration만 올려 안전성을 주장할 수 없다는 근거다.

다음 단계의 우선순위: 실측/target 관절 한계와 IK 검사, following-error/무응답 guard, 보간기 고정목표 안정성, 현재 정상 상태에서 stop 경로 검증. 이후 bounded tuning을 한 조건씩 수행해야 한다. 이 문서는 추가 물리 실행을 승인하지 않는다.

근거:

- [정상 runtime ServoL 실행](phase12_real_policy_rollout/14_real_rollout_integration/real_trials/20261106_110547_arm_only_servo_episode/trial_summary.json)
- [첫 한계 실패 실행](phase12_real_policy_rollout/14_real_rollout_integration/real_trials/20261106_111406_arm_only_servo_episode/trial_summary.json)
- [실패 timing 분석](phase12_real_policy_rollout/14_real_rollout_integration/real_trials/20261106_failed_servo_timing_audit/timing_analysis.json)
- [가속도 offline sweep](phase12_real_policy_rollout/14_real_rollout_integration/real_trials/20261106_servo_speed_tuning_stage1/tuning_summary.json)
- [변환·보간·관절 audit](phase12_real_policy_rollout/14_real_rollout_integration/real_trials/20261106_servo_conversion_audit_02/audit_findings.json)
- [controller alarm 증거](phase12_real_policy_rollout/14_real_rollout_integration/real_trials/20261106_z_direction_audit_02/driver_alarm_evidence.json)
- [현재 Servo 설정](phase12_real_policy_rollout/14_real_rollout_integration/configs/servo_episode.yaml)

## 4. 잘된 rollout 선별 — 재분석 기준점

| 우선순위 / 용도 | 선정 실험 | 선정 이유 | 주장하면 안 되는 내용 |
|---|---|---|---|
| 1. MoveL 방향·크기 기준 | `20261014_arm_only_short_horizon_1step_real_01` | 실제 0.800→0.806688mm, 0.642879→0.641841deg, operator 확인 및 최종 PASS | 현재 limiter OFF/120 profile, full task 성공까지 검증됐다는 주장 |
| 2. arm-only multi-step 기준 | `20261014_arm_only_70s_episode_001` | 70.385초, arm 25, 통합 PASS, 이상 motion·E-stop false | gripper 포함 policy 성능이나 cube grasp 성공 |
| 3. full-policy runtime 기준 | `20261105_161811_full_policy_episode` | 10/10 completed, 30.500초, runtime valid, 정상 시간 종료 | operator 결과 미확정인데 task SUCCESS로 집계 |
| 4. ServoL runtime/stop 기준 | `20261106_110547_arm_only_servo_episode` | 150 targets, 1,379 publishes, 70.822초, runtime valid, Stop ACK true | 모든 target 도달, 전체 trajectory 안전성, task 성공, 후속 joint fault 해결 |

이 선정은 확인한 대표 summary·feedback·audit 기준이다. 기록이 길거나 명령 수가 많다는 이유로 성공을 고르지 않았다. 특히 `20261106_145225`의 117개 요청과 `20261106_132009`의 102개 요청은 무이동/도달성 문제 분석 대상으로 남기고 좋은 실행에서 제외한다. `20261106_101231`도 83 steps 이후 safety rejection이므로 정상 완료 사례가 아니다.

Camera 양호 기준점은 rollout과 별도로 `20261106_094617_camera_inference_cpu_audit`를 사용한다. 180초·420 predictions·21.84Hz·최대 300ms·500ms 이상 0회이며 물리 명령은 0이다. 이를 모터/로깅을 포함한 전체 rollout 성공으로 간주하지 않는다.

## 5. 공통 미해결 사항과 인수인계

- **TCP:** `TCP_ZERO_OFFSET_SUPPORTED_NOT_VERIFIED`. pendant zero-offset candidate, controller flange=CurrentPosX, FK와 약 1.112mm/0.178deg 차이 및 1-step 실제 방향·크기를 engineering basis로 사용했다. active TCP name/offset의 formal 검증 완료로 승격하지 않는다.
- **JointState:** 과거 DDS discovery 간섭은 `ROS_LOCALHOST_ONLY=1` 적용 후 충분한 정상 수신 근거가 있다. 카메라 문제와 혼동하지 않는다. runtime source/receive gap ≥100ms, latest age ≥500ms watchdog은 별도 유지한다.
- **Gripper:** full-policy는 OPEN 초기화와 상태 transition만 IO 명령을 보내는 구조다. OPEN DO1 double pulse(1.3초 high → 0.2초 low → 0.5초 high → low), CLOSE DO2 0.5초 high → low가 현재 operator-timed contract다. abort는 DO1/DO2 모두 OFF, 최대 3회 IO 시도이며 실제 jaw OPEN/HOLD를 보장하지 않는다. 초기 단독 CLOSE 실패 이력과 이후 OPEN initialization의 중요성을 보존한다.
- **Direct launch:** 실제 OPEN 확인 없이 IO ACK로 command knowledge만 OPEN으로 시작한다. measured jaw는 UNKNOWN/null이다. 현장 안전 항목도 자동으로 실제 확인됐다고 기록하지 않는다. 기존 수동 확인 경로는 별도로 남아 있다.
- **No-motion 보호:** ACK, STANDBY, publish 반환만으로 실제 움직임을 판정하지 않는다. alarm/target-reach/following-error 근거가 필요하다.
- **Task 판정:** 이 문서의 좋은 runtime/arm 사례를 task 성공률 분모·분자에 임의 편입하지 않는다. full-policy라도 task_success가 미확정이면 미확정으로 남긴다.
- **실행 제한:** 자동 재시도·자동 다음 episode·자동 Home/recovery를 하지 않는 기존 원칙을 유지한다. 미검증 profile/IK/stop 문제는 operator 대기만으로 해결된 것으로 간주하지 않는다.

### 원본 로그에서 확인할 파일

| 목적 | 파일 |
|---|---|
| 실행 판정·중단 원인 | `trial_summary.json`, `runtime_events.jsonl` |
| 실제 모델 출력과 영상 | `predictions.jsonl`, `camera_*_model_input.jpg`, `observations.jsonl` |
| raw/limiter/target 구분 | `action_pipeline.jsonl`, `commands.jsonl` 등 해당 실행 파일 |
| MoveLine 실제 request/ACK | `arm_commands.jsonl`, `command_events.jsonl` 또는 summary의 command events |
| 실제 관절 trajectory | `jointstate.jsonl` 또는 해당 실행의 `jointstate_*.json` |
| Servo 실제 publish pose | 해당 Servo trial의 stream 로그 및 변환 audit의 matched records |
| 환경·설정 | `preflight.json`, `config_snapshot.json`, `model_health.json`, 각 audit의 provenance |

파일 이름은 실행 세대별로 다르므로 해당 디렉터리의 실제 파일을 확인한다. `*.partial` 또는 summary가 없는 기록은 완결 로그와 구분한다. 원본 값은 보존하고 후속 분석은 새 디렉터리에 저장한다.

## 최종 결론

1. Camera는 케이블 변경 직후 좋아졌으나, 이후 단독 수신과 재부팅 후에도 공백이 남았다. `yes` 제거·서버 uptime·CPU·USB 중 단일 원인은 아직 확정하지 못했다.
2. MoveL은 작은 실제 방향·크기와 일부 multi-step runtime이 검증됐다. 그러나 NOT REACHABLE를 ACK로 통과시킨 무이동 사례가 있어 독립 도달성 검사가 필요하다.
3. ServoL은 한 실행의 정상 streaming/stop 근거가 있으나 후속 J5 한계 실패와 stop 미확인 문제가 남았다. acceleration/time 후보는 offline 근거이며 물리 해결 완료가 아니다.
4. 잘된 실행은 위 4개를 목적별 기준점으로 선정한다. full-task cube pick 성공을 입증하는 자료로 과장하지 않는다.
