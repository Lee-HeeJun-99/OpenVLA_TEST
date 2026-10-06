# OFT Episode 4 K=5 Timing and Phase Alignment Audit

## 결론

```text
Final classification: MIXED
Dominant finding: MODEL_EARLY_CLOSE_DOMINANT_WITH_RUNTIME_MAPPING_DEFECTS
```

모델의 early-close가 지배적이지만 runtime의 timestamp/phase mapping에도 수정이 필요한 결함이 있다. Reference phase label에도 command/state 전환 사이 1-step coarse boundary가 있다. 어느 하나만으로 전체 현상을 설명할 수 없다.

## 입력과 계약

- OFT vision step28560, K=5
- inference frames: 0, 5, ..., 40
- 5 Hz target execution: `target_step = inference_frame + chunk_index`
- target time: `target_step × 0.2 s`
- production threshold와 workspace는 변경하지 않음
- 실제 command는 0회

## 1. Gripper timing

이전의 “first close candidate = frame 0”은 inference request 단위 표현이었다. Action 단위로 정확히 풀면 다음과 같다.

| Event | Step/time |
|---|---:|
| First OFT close candidate | inference frame 0, K index 2 → target frame 2, 0.4 s, closedness 0.84375 |
| Episode 4 reference close command | step 26, 약 5.2 s |
| Reference closed state | step 27 |
| First mapped `grasp_close` phase | step 27, 약 5.4 s |
| First inference-grid row labelled `grasp_close` | frame 30 |

따라서 OFT는 reference command보다 24 steps/4.8 s, target-step `grasp_close`보다 25 steps/5.0 s 빠르다. 첫 chunk는 `[0.4219, 0.6953, 0.8438, 0.8984, 0.9258]`로 뒤쪽 horizon일수록 close intent가 급격히 증가한다.

## 2. K=5 timing과 phase alignment

기존 recorded Shadow runner는 OFT에 다음 시간을 사용했다.

```text
legacy: frame_id × 1.0 + chunk_index × 0.2
correct: frame_id × 0.2 + chunk_index × 0.2
```

OFT inference row 자체가 0,5,10... frame이므로 legacy 식은 chunk 사이를 5초로 취급한다. Episode 후반 최대 timestamp 오차는 32초다. 이 오류는 action age within chunk에는 영향을 덜 주지만 chunk boundary velocity/acceleration 및 전체 timeline 해석을 왜곡한다. 실제 rollout 전에 반드시 수정·회귀검증해야 한다.

또한 legacy output은 한 chunk의 5 actions에 inference frame phase를 모두 붙였다. Target-step phase로 재매핑하면 45개 중 10개의 phase가 바뀐다. 그러나 premature close 총수는 coarse와 target-step mapping 모두 22개다. 서로 다른 boundary action이 교체될 뿐이며 frame 2의 early close는 여전히 `hold/alignment` 구간이다. 따라서 phase mapping 오류가 조기 close의 주원인은 아니다.

## 3. Reference/dataset timing

Episode 4 scripted reference는 close command를 step 26에 넣고, stored closed state와 `grasp_close` phase는 step 27부터 시작한다. 즉 command와 phase/state label 사이 1-step coarse boundary가 있다.

저장된 scripted reference trajectory 10개의 close command step은 다음과 같다.

```text
27, 28, 25, 26, 24, 25, 29, 26, 26, 26
median = 26
```

Checkpoint의 원래 training corpus는 이 bundle에 step-level episode로 존재하지 않아 직접적인 training distribution이라고 주장할 수 없다. 위 수치는 stored reference trajectories이며 training samples로 간주하지 않았다. 그래도 OFT target step 2는 reference 범위 24~29보다 22 steps 이상 빠르다.

## 4. K index별 action 특성

| K | closedness mean | translation mean | implied velocity | reject | Translation blocker | Accel blocker | Premature close |
|---:|---:|---:|---:|---:|---:|---:|---:|
| 0 | 0.595 | 2.38 mm | 11.92 mm/s | 3/9 | 1 | 0 | 2 |
| 1 | 0.716 | 3.57 mm | 17.83 mm/s | 8/9 | 4 | 8 | 3 |
| 2 | 0.793 | 5.88 mm | 29.41 mm/s | 7/9 | 4 | 3 | 5 |
| 3 | 0.859 | 7.22 mm | 36.11 mm/s | 9/9 | 4 | 5 | 6 |
| 4 | 0.908 | 7.34 mm | 36.70 mm/s | 8/9 | 6 | 3 | 6 |

뒤쪽 K index reject는 한 요소가 아니다.

- closedness가 K index에 따라 단조 증가한다.
- translation mean도 2.38→7.34 mm로 증가한다.
- K3/K4 implied velocity는 20 mm/s 기준을 크게 넘는다.
- K3는 premature/translation/acceleration이 모두 중첩되어 9/9 reject다.

Correct continuous 5 Hz acceleration은 `action_timing.csv`에 별도로 기록했다. 위 “Accel blocker”는 기존 decoupled SafetyPipeline이 실제 출력한 blocker이므로, legacy chunk-boundary time 오류의 영향을 받는다. 두 값을 혼동하면 안 된다.

## 질문에 대한 답

### 1. 왜 frame 0부터 close intent를 보이는가?

Frame 0 이미지에서 생성된 첫 K=5 chunk의 k2부터 closedness가 0.7을 넘는다. Target phase로 올바르게 매핑해도 frame 2는 아직 task 초반이다. Reference close는 step 26 부근이므로 모델 출력 자체의 조기 close가 확인된다. 단순 phase-label 오류가 아니다.

### 2. K=5 후반 action이 왜 더 많이 reject되는가?

후반 index에서 gripper closedness, translation magnitude와 implied velocity가 동시에 증가한다. K3/K4는 phase 밖 close intent 6/9, translation blocker 각각 4/9와 6/9이며 acceleration도 겹친다. 따라서 `K5_LATE_INDEX` 효과는 실제 action distribution과 관련된다.

### 3. 모델과 runtime 중 무엇을 수정해야 하는가?

둘 다 필요하다.

1. **Runtime 필수 수정:** `frame_id×1.0` 시간식을 5 Hz target-step 시간으로 변경하고 chunk action마다 `frame+index` phase를 사용해야 한다. 수정 전에는 rollout timing 검증을 통과할 수 없다.
2. **Model/contract 검증:** 올바른 mapping에서도 step 2 close가 남으므로 sequential K5를 그대로 실제 gripper에 전달하면 안 된다. Phase gate를 유지하고, 여러 episode에서 재현한 뒤 checkpoint/gripper-head calibration 또는 학습 데이터 timing을 재검증해야 한다.
3. **Reference label 보정:** close command step 26과 phase/state step 27의 차이를 명시해 one-step boundary를 허용하는 평가 규칙을 사용할 수 있다. 이것은 4.8초 early close를 설명하지 못한다.

## 산출물

- `results/action_timing.csv`: 45 action target-step mapping과 dynamics
- `results/gripper_timing.csv`: gripper/phase 상세
- `results/k_index_summary.csv`: K index별 통계와 blocker
- `results/phase_gripper_summary.csv`: mapped phase별 closedness 분포
- `results/reference_close_timing.csv`: 10개 stored reference timing
- `results/summary.json`: machine-readable 결론

## 제한

- Episode 4 단일 model prediction 분석이다.
- stored reference는 optimal policy 또는 measured ground truth가 아니다.
- training corpus timing은 bundle에서 직접 검증하지 못했다.
- 실제 rollout 성공률을 의미하지 않는다.
- Robot/motion/gripper/Home/trajectory/Hold/E-stop/real rollout 명령은 모두 0회다.
