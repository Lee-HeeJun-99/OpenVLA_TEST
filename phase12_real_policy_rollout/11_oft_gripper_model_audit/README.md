# OFT gripper model audit

## 결론

최종 분류는 **`MIXED`**다. 직접 확인된 주원인은
`CHECKPOINT_EARLY_CLOSE_BEHAVIOR`, 부원인은 `K5_HORIZON_BIAS`다.
`GRIPPER_NORMALIZATION_CONTRACT_PROBLEM`은 코드와 checkpoint statistics로
배제했다. 다만 step 28560이 실제 학습한 `mixed480` 원본 row archive가 이
호스트에 없으므로, 학습 label timing 자체에 대한 최종 판정은
`UNRESOLVED_FOR_EXACT_MIXED480_CORPUS`로 남긴다.

이는 실제 로봇 실패 판정이 아니라 저장 영상에 대한 prediction-only 감사다.

## 1. Gripper output contract

추적된 흐름은 다음과 같다.

1. dataset loader는 각 observation `t`에서 연속 label
   `[t,t+1,t+2,t+3,t+4]`를 만들며 terminal에서는 마지막 gripper 상태로
   padding한다.
2. action statistics mask의 7번째 값은 `false`다. 따라서 gripper label은
   motion 6축과 달리 q01/q99 affine normalization 대상이 아니다.
3. step 28560은 `bounded_gripper=true`, L1 regression head다. action head는
   마지막 출력에 sigmoid를 적용한다.
4. inference의 `_unnormalize_actions()`도 mask가 false인 gripper를 그대로
   통과시킨다.
5. server와 Phase 10/11 adapter도 이 값을 `gripper_closedness`로 그대로
   기록한다.

따라서 의미는 연속 closedness `[0,1]`, `0=open`, `1=closed`다. tanh나
추가 clamp는 적용되지 않으며, 이 checkpoint에서 gripper affine
de-normalization 오류는 관찰되지 않았다. 학습 loss는 motion L1과 별도로
gripper Smooth-L1을 계산하고 weight 3.0을 적용한다.

주요 근거:

- `prismatic/vla/datasets/a0509_dataset.py`: K=5 future slicing 및 mask 기반 정규화
- `prismatic/models/action_heads.py`: bounded gripper sigmoid
- `prismatic/extern/hf/modeling_prismatic.py`: mask 기반 역정규화
- `models/oft_mixed480_step28560/a0509_training_config.json`
- `models/oft_mixed480_step28560/dataset_statistics.json`

## 2. Training label audit와 provenance 제한

정확한 `mixed480` training config에는 manifest SHA-256만 남아 있고 그
manifest에 대응하는 원본 480-episode row corpus는 로컬에서 찾지 못했다.
따라서 [training_gripper_by_k.csv](results/training_gripper_by_k.csv)는 다음
두 소스를 명시적으로 분리한다.

- `LOCAL_OFT200_LABEL_CORPUS_PROXY_NOT_EXACT_MIXED480`: 로컬 198 episode,
  8,875 train rows. checkpoint 학습 corpus가 아닌 구조 검증 proxy다.
- `STORED_REAL_REFERENCE_EPISODES_1_TO_10_NOT_TRAINING`: 저장 reference 10개,
  452 rows. 역시 학습 데이터가 아니다.

OFT200 proxy에서 close step 중앙값은 26(범위 18–29)이며 K0→K4 closed 비율은
0.420→0.509다. 저장 reference 10개는 중앙값 26(범위 24–29), K0→K4는
0.420→0.509다. 이 증가는 미래 horizon이 실제 close transition을 더 많이
포함하는 정상적인 label 구조로 설명된다. 하지만 exact mixed480에서도 같은
크기였다고 단정할 수 없다.

checkpoint statistics 자체의 gripper mean은 0.4908, min/max는 0/1이고 mask는
false다. 이 global statistic만으로 K별 timing은 복원할 수 없다.

## 3. Checkpoint output audit

Phase 11의 유효한 `oft_run2` 저장 결과 20 condition×episode pair, 192 chunks
(960 action)을 조사했다. 별도 Phase 10 real recorded Episode 4도 포함했다.

- Phase 11: 20 pair 중 17 pair에서 reference보다 먼저 `>=0.7`이 발생했다.
- early pair의 lead 중앙값: 16 step, 3.2 s.
- 모든 `lighting_low` 5 pair에서 step 1–2에 첫 close가 발생했다.
- Phase 10 Episode 4: step 2(0.4 s), reference step 26(5.2 s), lead 24 step/4.8 s.
- Phase 11 baseline Episode 4는 step 25로 거의 일치했다. 즉 early close는
  Episode 4 ID만의 고정 현상이 아니라 입력 condition에도 강하게 의존한다.

전체 Phase 11 prediction의 K별 closedness 평균과 `>=0.7` 비율은 다음과 같다.

| K | mean | close ratio |
|---:|---:|---:|
| 0 | 0.607 | 0.531 |
| 1 | 0.652 | 0.594 |
| 2 | 0.702 | 0.641 |
| 3 | 0.744 | 0.698 |
| 4 | 0.785 | 0.734 |

K가 뒤로 갈수록 closedness가 커지는 명확한 output bias가 있다. 미래 label
chunk에도 구조적 K 증가가 있지만, exact mixed480 label이 없으므로 checkpoint
증가량과 정확히 대조할 수 없다.

## 4. 질문별 답변

1. **학습 label 자체가 K 후반 close를 빨리 요구하는가?** Future chunk 구조상
   transition 근처에서는 그렇다. 로컬 proxy에서도 증가하지만 exact mixed480
   크기는 미확인이다.
2. **normalization/denormalization이 잘못됐는가?** 아니다. gripper mask가
   false여서 affine 변환을 양방향 모두 건너뛴다.
3. **checkpoint가 label/reference보다 일찍 예측하는가?** 저장 reference 대비
   Phase 11 17/20 pair와 Phase 10 Episode 4에서 그렇다. exact training label
   대비 여부는 mixed480 원본 부재로 확정할 수 없다.
4. **K 증가에 따라 closedness가 커지는가?** checkpoint output에서는 단조
   증가한다. proxy labels에도 더 완만한 증가가 있다.
5. **real rollout 전에 수정/재학습해야 하는가?** sequential-K5 실제 rollout을
   승인하기 전에 exact mixed480 label archive 감사와 checkpoint calibration 또는
   retraining을 권고한다. 현재 premature-close phase gate는 유지해야 하며
   production threshold를 완화하지 않았다.

## 5. 산출물과 재현

```bash
python3 phase12_real_policy_rollout/11_oft_gripper_model_audit/audit_oft_gripper.py
python3 -m unittest phase12_real_policy_rollout/11_oft_gripper_model_audit/test_audit_oft_gripper.py
```

- `results/training_gripper_by_k.csv`
- `results/episode_gripper_timing.csv`
- `results/checkpoint_vs_reference.csv`
- `results/summary.json`

## 6. 안전 상태

Production safety threshold는 변경하지 않았다. Robot command, motion
service/action, gripper command, Home, trajectory, Hold/E-stop, real rollout은
모두 0회다.
