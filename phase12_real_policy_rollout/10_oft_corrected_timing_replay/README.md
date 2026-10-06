# OFT Corrected Timing Replay — Episode 4

## 수정 범위

Recorded prediction Shadow runner의 OFT timing과 phase mapping만 수정했다.

```text
target_step = inference_frame + chunk_index
target_time = target_step × 0.2 s
mapped_phase = phase[target_step]
```

동일 contract를 `03_shadow_mode/oft_timing_contract.py`에 단일화했다. OFT action source timestamp는 inference frame의 5 Hz time이며, action age는 `chunk_index × 0.2 s`다. Target-step의 recorded pose와 phase를 사용한다.

기존 `frame_id×1.0` 계산은 새 runtime 경로에서 제거했다. `legacy_recorded_time_sec()`는 기존 artifact의 before 결과를 재현하는 audit 전용 함수이며 새 runtime decision에는 사용되지 않는다.

## Before/after

Before는 gripper cascade 분리 후이지만 legacy timing을 사용한 10/45 결과다. After는 동일 action, threshold, workspace, phase gate에 corrected timing을 적용한 결과다.

| Metric | Before | Corrected | Change |
|---|---:|---:|---:|
| Total | 45 | 45 | 0 |
| Accepted | 10 | 13 | +3 |
| Rejected | 35 | 32 | -3 |
| Raw translation step | 19 | 19 | 0 |
| Translation acceleration | 19 | 0 | -19 |
| Premature gripper close | 22 | 22 | 0 |
| Stale/action-age | 0 | 0 | 0 |

Acceleration blocker 19건은 legacy chunk time discontinuity와 결합된 safety calculation artifact였다. Correct 0.2초 연속 timeline에서는 rate limiter와 supervisor의 acceleration 계약이 일치하여 이 blocker가 사라졌다. 그러나 blocker overlap 때문에 accepted 증가는 3개뿐이다.

## K index

| K | Before accepted | Corrected accepted | Corrected reject rate | Translation | Premature close |
|---:|---:|---:|---:|---:|---:|
| 0 | 6/9 | 6/9 | 33.3% | 1 | 2 |
| 1 | 1/9 | 3/9 | 66.7% | 4 | 3 |
| 2 | 2/9 | 2/9 | 77.8% | 4 | 5 |
| 3 | 0/9 | 1/9 | 88.9% | 4 | 6 |
| 4 | 1/9 | 1/9 | 88.9% | 6 | 6 |

K3/K4 reject 집중은 timing 수정 후에도 유지된다. Acceleration artifact는 제거됐지만 후반 index의 translation magnitude와 close intent는 그대로다.

## Early close

```text
first close candidate: target step 2, 0.4 s
reference close command: step 26
first valid grasp_close: step 27
lead vs reference: 24 steps, 4.8 s
```

Timing/phase 수정 후에도 early close는 동일하다. 따라서 runtime timing bug가 model close intent를 만든 것은 아니다.

## 최종 판정

```text
MODEL_EARLY_CLOSE_DOMINANT_AFTER_TIMING_FIX
```

Timing bug는 acceleration reject 19건을 만들었으므로 반드시 수정해야 하는 실제 runtime 결함이었다. 하지만 수정 후 가장 큰 잔여 blocker는 premature close 22건과 raw translation 19건이다. 다음 단계는 추가 timing 완화가 아니라 OFT checkpoint/gripper head, normalization 및 여러 recorded episode에서 early-close 재현을 분석하는 것이다. Sequential K5 real rollout 전 phase gate는 유지해야 한다.

## 파일

- `results/episode4_oft_corrected_shadow.jsonl`: command-disabled corrected replay 원본
- `results/before_after.csv`: blocker 전후 비교
- `results/action_timing_corrected.csv`: target step/time/phase/action-age
- `results/k_index_summary.csv`: K index별 전후 결과
- `results/summary.json`: 최종 판정

## 안전 및 한계

- Production safety threshold/config는 변경하지 않았다.
- accepted는 task success가 아니다.
- Episode 4 한 개의 recorded offline 결과다.
- 실제 Robot command, motion, gripper, Home, trajectory, Hold/E-stop 및 rollout은 모두 0회다.

## 재현

```bash
cd /home/ubuntu/a0509_vla_linux_field_bundle_20260903/lhj/phase12_real_policy_rollout
python 03_shadow_mode/prediction_shadow_runtime.py \
  --model oft \
  --input ../phase10_planner_based_shadow_mode/06_shadow_collection/offline_recorded_episode4/oft_vision_step28560_local/samples.jsonl \
  --output 10_oft_corrected_timing_replay/results/episode4_oft_corrected_shadow.jsonl \
  --command-mode disabled
python 10_oft_corrected_timing_replay/compare_corrected_replay.py
```

