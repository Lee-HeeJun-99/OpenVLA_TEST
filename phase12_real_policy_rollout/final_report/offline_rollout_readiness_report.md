# Phase 12 Offline Rollout Readiness Report

## 판정

```text
OFFLINE_ROLLOUT_PIPELINE_READY
REAL_MOTION_VALIDATION_PENDING
```

`REAL_ROLLOUT_READY` 또는 `MOTION_READY` 판정이 아니다.

## 구현 결과

- 공통 `CanonicalAction`, 1000 mm/m, world rotvec→Doosan ZYZ matrix composition
- `0=open, 1=closed` open-loop gripper contract
- OFT sequential K=5와 pending/stale/duplicate/partial/underrun 검사
- 통합 SafetyPipeline → HardCommandGate → sink
- command-disabled default 및 자동 motion 진입 불가
- Null/Mock sink, 항상 거부하는 future Real skeleton
- measured/FK/recorded/unavailable TCP abstraction
- recorded stationary/Shadow runner와 mock closed-loop
- model/action/safety/mock/future-real config 분리

## Recorded-data 검증

Phase 10 Episode 4의 실제 저장 모델 결과를 재사용했다.

- Stationary observation: 30/30 valid, command 0
- OpenVLA step8130: 45 action, accepted 16, rejected 29
- OFT Vision step28560: 9 inference × K=5 = 45 action, accepted 3, rejected 42
- 모든 record: `command_issued=false`, executed/delivered null
- Mock closed-loop: 5/5 software-path accepted, 실제 command 0

거부 수는 rollout 실패율이 아니다. 엄격한 raw-step, acceleration, workspace,
gripper phase 및 fail-stop 정책과 저장 action의 agreement 결과다.

Episode 8/9/10은 Phase 8 OFT-only 비교에는 포함되지만 Phase 12 형식의 양 모델
raw per-frame prediction이 없어 새 결과를 만들지 않았다. 모델을 섞지 않고
`PENDING_RECORDED_PREDICTION`으로 남긴다.

## 테스트 및 상태

- 전체 unittest: 87 PASS, 0 FAIL, 0 SKIP
- Python compile 및 command-capability absence: PASS
- ZYZ singular pose test에서 SciPy가 non-unique Euler angle warning을 출력할 수
  있으나 최종 rotation matrix equivalence는 통과했다. Raw Euler 성분 비교는
  판정 근거로 사용하지 않는다.

| 항목 | 상태 |
|---|---|
| Offline rollout pipeline | READY |
| Prediction-only Shadow | READY |
| Mock closed-loop | READY |
| Real command integration | IMPLEMENTED_INTERFACE_ONLY_NOT_VALIDATED |
| Real robot validation | PENDING |
| Real closed-loop rollout | NOT_AUTHORIZED |

```text
Robot command: 0
Motion service/action: 0
Gripper command: 0
Home: 0
Trajectory: 0
Hold/E-stop: 0
Closed-loop real rollout: 0
```
