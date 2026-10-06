# 추가 학습 데이터 재감사

원본 single-episode 분석은 보존하고 이번 결과는 results_added_data에 저장했습니다.

고유 episode 11개, 539 steps. 모두 10 Hz.
corrective_recovery 10개, nominal 1개. SHA-256 중복 0개.
첫 close는 2.0–3.2초, 중앙값 2.5초입니다.
Nominal은 step32/3.2초, corrective 중앙값은 2.45초입니다.

| Source | K | Training close ratio | OFT close ratio | 차이 |
|---|---:|---:|---:|---:|
| PHASE10_EPISODE4 | K0 | 48.794% | 44.444% | -4.35%p |
| PHASE10_EPISODE4 | K1 | 50.835% | 55.556% | +4.72%p |
| PHASE10_EPISODE4 | K2 | 52.876% | 77.778% | +24.90%p |
| PHASE10_EPISODE4 | K3 | 54.917% | 88.889% | +33.97%p |
| PHASE10_EPISODE4 | K4 | 56.957% | 88.889% | +31.93%p |
| PHASE11_OFT_RUN2 | K0 | 48.794% | 53.125% | +4.33%p |
| PHASE11_OFT_RUN2 | K1 | 50.835% | 59.375% | +8.54%p |
| PHASE11_OFT_RUN2 | K2 | 52.876% | 64.062% | +11.19%p |
| PHASE11_OFT_RUN2 | K3 | 54.917% | 69.792% | +14.88%p |
| PHASE11_OFT_RUN2 | K4 | 56.957% | 73.438% | +16.48%p |

Training은 실제 label이며 reference proxy와 구분했습니다. Terminal은 loader처럼 마지막
gripper 상태를 유지합니다. 통계는 전체 timestep 가중치 기준입니다.
Training K5는 0.4초, prediction K5는 0.8초 horizon이므로 K별 차이는 기술적 비교입니다.
Phase10 Episode4 첫 close 0.4초는 이번 training 중앙값 2.5초보다 2.1초 빠르지만,
동일 영상·시작 상태 비교가 아니므로 조기 close 원인의 인과 증거로 사용할 수 없습니다.

## 확인된 사실과 판정

Primary finding: `DESCRIPTIVE_K5_CLOSE_AMPLIFICATION_OBSERVED`
Causal attribution: `UNRESOLVED` / `CAUSAL_ATTRIBUTION_UNRESOLVED`
Dataset representativeness: `LIMITED_SAMPLE_OF_TRAINING_CORPUS`
기존 `INSUFFICIENT_TRAINING_DATA`는 표본 대표성 제한으로 보존하며 단독 결론으로 사용하지 않습니다.

Training label 자체에도 K 후반 close 비율 증가가 존재합니다.
K0→K4 증가폭은 training +8.2%p,
OFT Episode4 +44.4%p,
OFT Phase11 +20.3%p입니다.
OFT prediction은 같은 방향의 증가를 보이며 증가폭이 training보다 큽니다.
따라서 prediction에서 K 후반 close 편향이 더 강하게 관찰됩니다.

현재 데이터는 checkpoint가 training bias를 증폭했을 가능성을 지지합니다.
다만 exact mixed480 학습 포함 여부와 resampling이 미확인이고,
training은 10 Hz / 0.4초 horizon, prediction은 5 Hz / 0.8초 horizon이며,
동일 observation 기반 matched comparison이 아니므로 인과적 증폭으로 확정할 수 없습니다.
Nominal 1개로 demonstration type 일반화를 할 수 없습니다.

## Rollout recommendation

Real rollout: `DEFERRED_FOR_MODEL_BEHAVIOR_REVIEW`.
K 후반 close 편향과 early-close behavior가 offline에서 반복적으로 확인됐습니다.
현재 rollout은 새로운 원인 규명보다 기존 현상의 물리적 재확인에 그칠 가능성이 높아,
exact training split 감사와 동일 입력 checkpoint 비교를 먼저 수행하는 것을 권고합니다.
모델 수정·검증 이후 real rollout은 최종 physical validation으로 필요합니다.
현재 phase safety gate를 유지하며 재학습 여부는 후속 모델 검토로 결정합니다.
Production threshold 변경 없음. 신규 inference·robot command 모두 0회.
