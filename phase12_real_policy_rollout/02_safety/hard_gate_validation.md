# Hard command gate validation

상태: `PASS_COMMAND_DISABLED_ONLY`

## 구성

- 안전하지 않은 8개 capability flag는 모두 명시적으로 `false`여야 한다.
- `action_adapter`와 `doosan_bridge` 포함은 시작 단계에서 거부한다.
- 최종 sink는 `NULL_COMMAND_SINK`이며 candidate audit 외 기능이 없다.
- 기술적으로 유효한 candidate도 delivery는 항상 차단된다.
- 기술적으로 무효한 candidate의 blocker와 `hold_required`는 그대로 보존한다.
- Shadow 단계에서는 Hold/E-stop service를 호출하지 않는다.

## 실제 저장 prediction dry-run

- 입력: stationary OpenVLA K=1 한 개와 OFT K=5 한 chunk
- 총 candidate: 6
- technical valid: 4
- 기존 gate 차단: OFT alignment 단계 premature-close 2개
- hard-gate delivery blocked: 6/6
- executed action null: 6/6
- robot delivered command null: 6/6
- command issued false: 6/6

## 자동 검사

표준 `unittest` 전체 결과: **55/55 PASS**.

검사에는 unsafe/missing flag 거부, command capability 정적 부재, K=5 순서,
polarity/hysteresis, stale/duplicate/NaN, timeout, workspace, rate/velocity/
acceleration, logger flush/fsync/partial/disk-full 및 null delivery가 포함된다.

## 제한

이는 command-disabled pre-motion 통합이다. 실제 robot publisher/service와의
연결은 존재하지 않으므로 robot motion readiness를 승인하지 않는다.

