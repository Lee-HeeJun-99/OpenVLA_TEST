# Training label vs OFT prediction

판정: `INSUFFICIENT_TRAINING_DATA`.

제공된 실제 데이터는 1 episode: blue cube / corrective_recovery / 10 Hz / 56 steps.
Checkpoint mixed480 학습에 포함됐음을 증명하는 manifest는 미확인입니다.

## Training label timing

action[6]은 0=open, 1=closed. 최초 >=0.7은 step 28, nominal 2.8 s입니다.
control.gripper_closedness_target은 step 29부터 close입니다. 1 row 차이는
observation_t→t+1 action alignment와 일치할 가능성이 있으나 생성 코드와 대조는 미완료입니다.
해당 step phase가 move_grasp이므로 경과 시간만으로 premature라고 판단할 수 없습니다.
Wall timestamp 경과는 7.446 s입니다.
sim_time_s는 6회 비단조이므로 경과 계산에 사용하지 않았습니다.

## K=5 comparison

Terminal padding은 loader처럼 마지막 gripper 값을 유지합니다. 전체 t를 동일 가중치로 집계합니다.
Training K0→K4 ratio: 0.5000→0.5714。
10 Hz training horizon은 0.4 s, 5 Hz prediction horizon은 0.8 s입니다.
이 표의 K 대응은 index 비교이며 동일 물리 horizon의 paired comparison은 아닙니다.

| Source | K | Training close ratio | Prediction close ratio | Prediction − training |
|---|---:|---:|---:|---:|
| PHASE10_EPISODE4 | 0 | 0.5000 | 0.4444 | -0.0556 |
| PHASE10_EPISODE4 | 1 | 0.5179 | 0.5556 | +0.0377 |
| PHASE10_EPISODE4 | 2 | 0.5357 | 0.7778 | +0.2421 |
| PHASE10_EPISODE4 | 3 | 0.5536 | 0.8889 | +0.3353 |
| PHASE10_EPISODE4 | 4 | 0.5714 | 0.8889 | +0.3175 |
| PHASE11_OFT_RUN2 | 0 | 0.5000 | 0.5312 | +0.0312 |
| PHASE11_OFT_RUN2 | 1 | 0.5179 | 0.5938 | +0.0759 |
| PHASE11_OFT_RUN2 | 2 | 0.5357 | 0.6406 | +0.1049 |
| PHASE11_OFT_RUN2 | 3 | 0.5536 | 0.6979 | +0.1443 |
| PHASE11_OFT_RUN2 | 4 | 0.5714 | 0.7344 | +0.1629 |

## Early-close comparison

Phase10 Episode4 prediction 최초 close는 step2 / 0.4 s입니다.
이번 training label 2.8 s와 시간차는 2.4 s, 5 Hz 환산 12 step입니다.
기존 4.8 s는 Episode4 자체 Scripted Reference Command 5.2 s와의 차이입니다.
두 값은 서로 다른 비교입니다. Raw 28−2 step은 Hz가 달라 사용하지 않습니다.
전체 episode 시간차는 results/early_close_comparison.csv에 저장했습니다.

## 해석과 다음 작업

K 후반의 close label 비율은 증가하며 prediction 증가폭은 관측상 더 큽니다.
다만 blue corrective / orange real 차이, horizon 차이, episode 길이·hold 시간 차이 때문에
checkpoint가 training bias를 증폭했다는 인과 결론은 낼 수 없습니다.
제공 데이터의 nominal episode는 0, corrective는 1로 demonstration 비교는 미실시입니다.
Exact mixed480 corpus와 resampling 기록을 확보하고 동일 observation의 checkpoint 비교로
calibration/retraining 필요성을 판단해야 합니다. 현재 phase safety gate는 유지합니다.

## Reproduction and safety

`python3 phase12_real_policy_rollout/12_train_label_vs_oft_prediction/analyze_train_vs_prediction.py`

입력 hash와 prediction provenance는 summary.json에 저장했습니다. 신규 inference는 없습니다.
Production safety threshold 변경 없음. Robot/motion/gripper/Home/trajectory/Hold/E-stop/real rollout 모두 0회.
