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

판정: INSUFFICIENT_TRAINING_DATA. Label K 증가와 더 큰 prediction K 증가가 관찰됐지만
정확한 mixed480 포함 여부·resampling, 동일 task/input 비교가 미확인입니다.
Nominal 1개로 demonstration type 일반화를 할 수 없습니다.
재학습 결정 전 exact training split 및 같은 입력의 checkpoint별 prediction을 확보해야 합니다.
Production threshold 변경 없음. 신규 inference·robot command 모두 0회.
