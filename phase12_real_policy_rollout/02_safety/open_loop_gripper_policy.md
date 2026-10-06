# Open-loop gripper safety policy

상태: `APPROVED_AND_IMPLEMENTED_COMMAND_DISABLED`

Measured gripper position feedback은 발견되지 않았다. 사용자 승인을 근거로
command 상태만 추적하며 실제 jaw 상태를 측정했다고 표현하지 않는다.

- 상태는 `UNKNOWN`, `COMMAND_OPEN`, `COMMAND_CLOSED`이다.
- 초기 상태는 `UNKNOWN`이며 현장 초기-open 확인 후에만 진행한다.
- `0=open`, `1=closed`, threshold는 0.3/0.7이다.
- close 후보는 `grasp_close`에서 rollout당 1회만 허용한다.
- lift에서는 이미 생성된 `COMMAND_CLOSED` 유지뿐이며 새 close를 만들지 않는다.
- 동일 명령은 억제한다.
- stale/timeout, 통신/logger 실패, NaN/Inf 또는 결과 불명은 `UNKNOWN`으로
  전이하며 lift와 후속 명령을 차단한다.
- measured state는 항상 null이고 commanded state와 분리한다.
- grasp 성공은 영상/cube lift 결과로 사후 판정한다.

자동검사는 상태 전이, phase/1회 제한, hysteresis, duplicate 및 모든 실패
경로를 포함한다. 실제 gripper command, robot motion, Home, Hold/E-stop은 0회다.
이 정책 승인은 실제 gripper actuation 또는 motion 승인이 아니다.
