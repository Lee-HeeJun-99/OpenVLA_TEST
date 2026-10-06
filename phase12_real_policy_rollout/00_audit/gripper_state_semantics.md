# Gripper State Semantics

## 판정

기존 `CommandDisabledPolicyRuntime`은 `fresh=safety.accepted`를 전달했다. 따라서 translation 또는 acceleration reject도 prediction/command-channel 손실처럼 처리되어 `OpenLoopGripperSupervisor.invalidate("stale_or_timeout")`가 command knowledge를 UNKNOWN으로 만들었다. 이는 safety action rejection과 command-state uncertainty를 혼합한 구현이었다.

새 runtime은 `GripperRuntimeContext`를 사용한다. 다음 사실은 독립적이다.

| 개념 | 의미 | 현재 offline 값 |
|---|---|---|
| Model intent | `model_gripper_closedness`, 0=open, 1=closed | 모델별 prediction |
| Candidate command | threshold와 phase gate로 만든 OPEN/CLOSE 후보 | 기록만 함 |
| Command knowledge | 마지막으로 확인된 command-side 상태 | `COMMAND_OPEN/CLOSED/UNKNOWN` |
| Measured feedback | 물리 gripper 센서 상태 | `MEASURED_UNKNOWN`/`null` |
| Executed command | 실제 actuator에 전달·확인된 명령 | 항상 null/false |

**Command knowledge is not measured feedback.** Command knowledge를 measured state로 승격하지 않는다.

## Context 분리

```text
prediction_fresh
action_accepted
communication_ok
command_channel_ok
initial_state_known
candidate_executed
```

- `action_accepted=false`: 이번 후보는 실행되지 않으며 이전 command knowledge를 유지한다.
- `prediction_fresh=false`: stale prediction을 거절하지만 이전 command knowledge를 유지한다.
- `communication_ok=false` 또는 `command_channel_ok=false`: actuator command 결과를 신뢰할 수 없으므로 UNKNOWN으로 무효화할 수 있다.
- `initial_state_known=false`: UNKNOWN을 유지한다.
- `candidate_executed=false`: 후보가 만들어져도 command knowledge나 close count를 변경하지 않는다.
- phase 밖 close candidate: `premature_gripper_close`로 거절하고 이전 knowledge를 유지한다.

## Runtime 흐름

```text
model_gripper_closedness
→ prediction freshness 확인
→ phase/hysteresis candidate 생성
→ whole-action safety 결과 확인
→ HardCommandGate / sink
→ acknowledged execution일 때만 command knowledge commit
```

현재 `NullCommandSink`에서는 `gripper_candidate_executed=false`이므로 state commit이 없다. Real runtime을 나중에 연결할 때도 sink acknowledgement 이전에는 `candidate_executed=true`를 설정하면 안 된다.

## State invalidation

허용하는 invalidation:

- 실제 communication failure
- gripper command channel failure 또는 acknowledgement uncertainty
- runtime restart/explicit reset
- 확인되지 않은 초기 상태

invalidation하지 않는 사건:

- translation/rotation step reject
- velocity/acceleration reject
- workspace reject
- premature close phase reject
- logger failure 자체

Logger failure는 전체 action을 fail-closed로 막지만, 이미 알고 있던 과거 command state가 물리적으로 사라졌다는 증거는 아니다.

## 호환성

기존 `resolve(... fresh=...)`는 과거 결과 causal replay와 직접 unit-test 호환을 위해 남아 있다. Phase 12 통합 runtime은 새 `resolve_with_context()`만 사용한다. 기존 policy 결과는 역사적 baseline으로 보존하며 새 정책 결과와 섞지 않는다.

