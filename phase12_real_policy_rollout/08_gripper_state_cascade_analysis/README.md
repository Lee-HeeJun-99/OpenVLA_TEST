# OFT Episode 4 Gripper State Cascade Analysis

## 목적과 방법

동일한 OFT vision step28560 Episode 4 action 45개를 production threshold, workspace, 5 Hz limiter, phase 및 K=5 순서를 고정하고 두 정책으로 replay했다.

- Current: `fresh=safety.accepted`; reject가 gripper knowledge를 UNKNOWN으로 무효화
- Decoupled: action reject는 후보 미실행으로 처리하고 이전 command knowledge 보존

실제 command는 없으므로 두 정책 모두 `gripper_candidate_executed=false`, measured state는 `MEASURED_UNKNOWN`, `command_issued=false`다.

## Before/after

| Metric | Current | Decoupled |
|---|---:|---:|
| Total | 45 | 45 |
| Accepted | 3 | 10 |
| Rejected | 42 | 35 |
| gripper unknown | 20 | 0 |
| gripper stale | 19 | 0 |
| translation step | 19 | 19 |
| acceleration | 19 | 19 |
| premature close | 3 | 22 |

Net cascade-generated rejects removed: **7**. Unknown/stale 39 occurrences가 곧 39개의 독립적인 추가 reject였다는 뜻은 아니다. blocker overlap과 state cascade가 있으며, UNKNOWN이 제거되자 그동안 가려졌던 premature close 22건이 나타났다.

## K=5 결과

| K index | Current accepted | Decoupled accepted |
|---:|---:|---:|
| 0 | 2/9 | 6/9 |
| 1 | 1/9 | 1/9 |
| 2 | 0/9 | 2/9 |
| 3 | 0/9 | 0/9 |
| 4 | 0/9 | 1/9 |

index 3은 decoupling 이후에도 9/9 reject다. 뒤쪽 K index 문제는 UNKNOWN cascade만으로 설명되지 않으며 실제 translation/acceleration 및 early close intent가 남는다.

## 판정

```text
GRIPPER_STATE_CASCADE_CONFIRMED_PARTIAL
Remaining OFT cause: MIXED
```

분리 정책은 bookkeeping 오류를 수정했지만 OFT action을 안전하다고 만들지는 않는다. 남은 blocker는 premature close 22, raw translation 19, translation acceleration 19다. Premature close 정책은 완화하지 않았다.

## 파일

- `results/current_policy.csv`: legacy action-level replay
- `results/decoupled_policy.csv`: 새 semantics action-level replay
- `results/blocker_comparison.csv`: blocker 전후 비교
- `results/oft_k_index_comparison.csv`: K index별 비교
- `results/oft_gripper_state_transitions_*.csv`: state transition provenance
- `results/summary.json`: machine-readable summary

## 한계

- recorded Episode 4 한 개에 대한 software causal replay다.
- accepted는 task success가 아니다.
- 실제 gripper feedback 및 command acknowledgement가 없다.
- production threshold는 변경하지 않았다.
- 실제 Robot/ROS/motion/gripper/Home/trajectory/Hold/E-stop/rollout 명령은 모두 0회다.

## 재현

```bash
cd /home/ubuntu/a0509_vla_linux_field_bundle_20260903/lhj/phase12_real_policy_rollout
python 08_gripper_state_cascade_analysis/compare_gripper_state_policy.py
```
