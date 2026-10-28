# 입력 전처리와 평균 은닉 특징 보정의 성분별 재평가

## 요약

- 추가 학습·영상 전체 추론·로봇 실행 없이 저장된 225쌍을 재사용했다. 물리 명령 0회.
- 밝기 정렬(B)은 픽셀 MSE를 줄였지만 이동·회전·그리퍼 예측 차이는 모두 악화했다.
- 여백 추가(C)는 보정 없이 세 행동 성분의 차이를 모두 줄였다.
- A의 조건별 평균 보정은 이동 32.69%, 회전 38.92%, 그리퍼 66.21% 감소였다.
- C의 평균 보정 추가 효과는 이동 4.25%, 그리퍼 10.99% 감소지만 **회전은 15.20% 악화**였다.
- C 에피소드 1에서는 평균 보정이 세 성분 모두 악화했다. 보정의 일률적 우수성을 주장할 수 없다.

## 1. 실험 및 자료 감사

실행 Git branch `lhj-research`, commit `f8246eb3781b5c0a7d2c32d1a22dfe0b532cb6a4`. dirty worktree는 `audit.json`에 저장했고 기존 rollout 변경은 건드리지 않았다.
정책 `oftplus_h5_vision`, `oft_mixed480_step28560_merged`, 지시 `Pick up the orange cube.`, K=5, action dim=7,
보정 위치 `action_hidden_states.input`(관측당 1×35×4096), bounded continuous gripper, FiLM 사용·proprio 미사용이 실제 설정과 일치했다.
dataset key `a0509_sim_cube_pick`; q01/q99/mask는 `audit.json` 및 통계 snapshot에 그대로 저장했다.

에피소드별 관측 수는 1~5 순서로 **46, 47, 44, 45, 43**이다. fold별 학습 관측 수는 **179,178,181,180,182**다.
원본 관측 쌍 manifest, A/B/C 각 domain의 전체 특징과 행동, action head 가중치, 통계와 기존 LOO 코드는 모두 존재했다.
따라서 필수 조건 중 미실행 조건은 없다. 먼저 저장 행동만 재집계(`cached_*`), 이후 고정 헤드 재계산 및 평균 보정을 실행했다.
민감/잔여 분해·gradient·저차원 민감 부분공간·진행도/phase 보정·조건 간 보정 전이는 실행하지 않았다.

현재 pair는 저장된 joint-relative replay 대응이다. 역사적 Real EE pose는 planned/commanded이며 measured feedback은 없다.
225쌍을 정확히 같은 물리 EE 상태의 측정 ground truth로 간주할 수 없다.

## 2. 실제 입력 처리

모든 조건에서 동일한 225 pair_id를 사용했다. 원본 source 영상과 paired 영상은 Real/Sim 모두 1280×720이다.
조건 매핑은 A=`P0_current_paired_image`, B=`P5_brightness_match_sim_to_real`, C=`P4_letterbox_224`다.

- **A**: paired RGB 영상 무변환. 단 Phase6 condition materialization 과정에서 양쪽 모두 JPEG quality 95로 재인코딩했다.
- **B**: Real 무변환, **Sim만** `gain=mean(Y_real)/max(mean(Y_sim),1e-6)`로 PIL `ImageEnhance.Brightness` 적용.
  Y=0.2126R+0.7152G+0.0722B, RGB/255 전체 영상 평균이다. 곱셈 gain이지 additive shift가 아니며 포화/uint8 양자화가 있다.
  양쪽 모두 JPEG quality 95로 저장. 대표 pair의 gain은 0.671114062다. 과거 설명의 “shift” 표현과 실제 코드의 multiplicative gain 차이를 명시한다.
- **C**: Real/Sim 모두 PIL thumbnail BICUBIC으로 **224×126**, RGB black `(0,0,0)`의 224×224 canvas 중앙에 배치.
  left/right 0, top/bottom 각각 **49 px**. 그 뒤 JPEG quality 95. 여백은 정책 내부 center crop에 의해 일부 잘린다.

### 정책 내부의 추가 처리 — 세 조건에 공통

`extract_vla_features.py`는 `center_crop=True`로 OFTRuntime을 생성한다.
A/B처럼 224×224가 아니면 TensorFlow JPEG encode/decode(기본 quality 95), Lanczos3 antialias resize 224×224, round/clip uint8을 수행한다.
C는 이미 224×224이므로 이 조건부 resize/JPEG 단계를 건너뛴다. **이 차이도 기존 경로 그대로 보존**했다.
이후 모두 중심 **면적 비율 0.9**(가로·세로 비율 sqrt(0.9)) crop, TensorFlow crop_and_resize의 bilinear로 224×224 복원.
Prismatic fused processor는 224×224 bicubic antialias resize/224 crop 및 두 backbone별 고정 mean/std Normalize를 적용한다.
center-crop 0.9는 폭을 0.9배로 자르는 의미가 아니다.

따라서 C의 효과를 순수한 도메인 차이 제거라고 부르지 않는다. 객체 크기·위치·가시 영역·padding 비율뿐 아니라 내부 경로도 변한다.

`representative_raw_paired_and_model.png`는 대표 pair의 raw source, paired 원본, A/B/C 실제 기록된 model tensor를 나란히 보여준다.
실제 입력 tensor는 `representative/model_input_tensors.npz`에 보존했다. 시각화는 고정 mean/std 역변환 후 PNG용 clip/round만 적용하며,
영상별 min-max 재정규화는 없다. 원본·조건 입력 PNG도 `representative/`에 별도 보존했다.

## 3. 동일 헤드 경로와 기준값 검증

헤드는 고정하고 gradient를 비활성화했다. 원래 추론과 동일한 **BF16, batch=1**로 Real/Sim 모두 재계산했다.
검사 전 설정한 허용 오차는 normalized max abs 1e-6, 역정규화 action max abs 1e-8이다.
**3조건×2domain×225 = 1,350개 모두 최대 차이 0.0**으로 저장 행동 및 저장 head output과 정확히 일치했다.
보정 없음과 보정 있음은 동일한 이 헤드/역정규화 경로를 사용한다.

과거 수치를 임의로 맞추지 않았다:

| 경로 | 종합 chunk 평균 L2 |
|---|---:|
| 원래 paired 입력의 저장 BF16 Real/Sim | 0.370262063 |
| 과거 코드처럼 원래 paired 입력 Real만 FP32 헤드 재계산 vs 저장 BF16 Sim | 0.370244941 |
| 이번 A: JPEG 재인코딩한 Phase6 P0, 양쪽 BF16 재계산 | 0.370203438 |

두 번째 경로는 보고된 0.370245를 6자리에서 재현했다. 기존 FP32 probe의 첫 관측은 normalized 최대 차이 0.00251442,
decoded 최대 차이 0.00221199였고 BF16 probe는 0이었다. 이번 주 분석 결과에 FP32/원래 paired baseline을 섞지 않았다.
기존 global_mean 보고값 0.125657과 이번 A 평균 보정 0.125363233 역시 입력/정밀도/재계산 기준이 달라 동일 실행 결과로 표기하지 않는다.

## 4. 보정 및 지표

각 조건 c=A,C에서 test episode e를 제외한 4개 episode의 **관측별** `(h_sim-h_real)`를 FP64로 원소별 평균했다.
`h_real_test + delta[c,-e]`를 동일 BF16 헤드에 주입했다. 주입 때만 원래 연산 dtype으로 변환한다.
test episode의 Sim 특징/행동은 delta 추정에 들어가지 않으며 Sim 행동은 최종 비교 기준으로만 사용한다.
A delta를 C에 전이하지 않았다. 관측 수가 다른 episode를 균등 가중하지 않고 지정된 T_-e 전체 관측을 균등 가중했다.
fold 목록은 `folds.json`, FP64 delta 배열은 `mean_correction_vectors.npz`에 저장했다.

각 관측에서 K=5의 매 step별 이동3 L2, 회전3 L2, 그리퍼 abs, 7D L2를 계산해 K 평균 후 225 관측 평균했다.
5개 test fold는 각각 225 pair를 중복·누락 없이 한 번씩 포함한다. 독립 검증은 `independent_fold_audit.json` 참조.
평균 감소율은 전체 평균 전후의 비율이며 관측별 감소율을 평균하지 않았다.

역정규화: `where(mask, 0.5*(a_norm+1)*(q99-q01+1e-8)+q01, a_norm)`.
mask=[true,true,true,true,true,true,false]로 **그리퍼는 q01/q99 재변환하지 않는다**. 모든 표는 같은 decoded 출력 스케일이다.

단위/표현: 데이터 collector source의 xyz 위치 차이는 **m**, 회전은 `q_next * inverse(q_previous)`의 shortest rotvec **rad**,
그리퍼는 **무차원 continuous closedness [0,1] (0 open,1 closed)**다. 회전 지표는 rotvec 좌표의 유클리드 차이이지 SO(3) geodesic이나 실제 자세 추종 오차가 아니다.
Sim collector는 world-frame delta; Real은 저장된 base/planned pose 유래이고 양쪽 물리 좌표 정합은 미확인이다.
따라서 지표는 같은 모델의 두 입력 간 예측 불일치이지 검증된 실제 로봇 pose 오차가 아니다.
기존 multi-metric 코드의 gripper threshold 0.5는 `ASSUMED_THRESHOLD`로 표시되어 있어 **열기/닫기 일치율은 계산하지 않았다**.

## 5. 논문용 표 1 — 입력 변환, 보정 없음

| 입력 조건 | 픽셀 MSE¹ | 이동 L2 (m) | 회전 L2 (rad) | 그리퍼 절대 차이 |
| --- | --- | --- | --- | --- |
| A | 0.086255597 | 0.004272973 | 0.004357588 | 0.369431044 |
| B | 0.026059177 | 0.004513420 | 0.005326350 | 0.376012356 |
| C | 0.050448888 | 0.001774022 | 0.001531518 | 0.101699992 |

¹ 픽셀 MSE는 **실제로 기록된 backbone 1 입력 tensor를 고정 Normalize 역변환한 RGB [0,1] 스케일**에서 산출했다.
BF16 양자화가 포함된다. backbone 2 및 정책 입력 전 condition JPEG MSE는 별도 CSV 열이다.
PNG에 사용하는 clipping/uint8 round는 MSE 계산에 사용하지 않는다. 서로 다른 해상도의 pre-policy JPEG MSE와 모델 입력 MSE를 혼용하지 않았다.
C에서는 공통 검은 여백이 MSE를 줄이는 효과도 있다.

## 6. 논문용 표 2 — 같은 입력 조건 안의 평균 보정

| 입력 조건 | 평균 보정 | 이동 L2 (m) | 회전 L2 (rad) | 그리퍼 절대 차이 |
| --- | --- | --- | --- | --- |
| A | 없음 | 0.004272973 | 0.004357588 | 0.369431044 |
| A | 조건별 5-fold LOO | 0.002876286 | 0.002661825 | 0.124836012 |
| C | 없음 | 0.001774022 | 0.001531518 | 0.101699992 |
| C | 조건별 5-fold LOO | 0.001698703 | 0.001764365 | 0.090520582 |

## 7. 상세 효과와 에피소드별 결과

양의 감소율=개선, 음의 감소율=악화. 종합 7D L2는 혼합 단위의 기존 진단 지표이며 아래 세 성분 평균의 합이 아니다.

| 조건 | 이동 감소율 | 회전 감소율 | 그리퍼 감소율 | 종합 7D L2 전 → 후 | 종합 감소율 |
| --- | --- | --- | --- | --- | --- |
| A | +32.69% | +38.92% | +66.21% | 0.370203438 → 0.125363233 | +66.14% |
| C | +4.25% | -15.20% | +10.99% | 0.101993157 → 0.090847054 | +10.93% |

| 입력 | test episode | 관측 수 | 이동 감소율 | 회전 감소율 | 그리퍼 감소율 | 7D 감소율 |
| --- | --- | --- | --- | --- | --- | --- |
| A | episode_000001 | 46 | +38.11% | +50.21% | +81.01% | +80.93% |
| A | episode_000002 | 47 | +26.68% | +29.17% | +73.91% | +73.74% |
| A | episode_000003 | 44 | +29.06% | +36.17% | +48.02% | +48.05% |
| A | episode_000004 | 45 | +39.14% | +46.82% | +82.86% | +82.68% |
| A | episode_000005 | 43 | +32.18% | +38.80% | +52.95% | +52.94% |
| C | episode_000001 | 46 | -42.00% | -44.29% | -68.49% | -68.03% |
| C | episode_000002 | 47 | +5.22% | -17.55% | +8.86% | +8.77% |
| C | episode_000003 | 44 | +26.96% | +1.43% | +37.10% | +37.03% |
| C | episode_000004 | 45 | +3.77% | -26.10% | +10.94% | +10.70% |
| C | episode_000005 | 43 | +22.84% | +19.12% | +41.30% | +41.16% |

에피소드별 절대 평균과 절대 변화량은 `episode_metrics.csv`, `correction_effects.csv`에 있다.
A는 모든 episode에서 세 성분 평균이 개선됐다. C는 이동/그리퍼 4/5 episode 개선, 회전 2/5 개선이다.
C episode 1은 이동 -42.00%, 회전 -44.29%, 그리퍼 -68.49%로 전부 악화했다.
분모가 작은 값의 비율만 강조하지 않도록 절대값도 모두 저장했다. 예를 들어 C 이동의 전체 절대 감소는 0.000075319 m(0.075319 mm)에 불과하다.

## 8. 연구 질문별 답

1. **픽셀 MSE가 줄면 각 행동 성분 차이도 줄어드는가? 아니다.** B는 A보다 MSE가 약 69.79% 낮지만 이동 5.63%, 회전 22.23%, 그리퍼 1.78% 악화했다.
   C는 A보다 MSE가 약 41.51% 낮으면서 세 성분은 개선됐다. MSE 순위 B<C<A와 행동 차이 순위 C<A<B는 다르다.
   이는 세 처리 조건에 대한 기술적 비교이지 보편적인 MSE-행동 인과관계나 통계적 유의성 주장이 아니다.
2. **종합 차이 감소가 주로 그리퍼 변화와 관련됐는가? 수치적으로 그렇다.** A에서 7D L2 0.370203과 gripper abs 0.369431이 가깝다.
   A 평균 보정 후에도 각각 0.125363과 0.124836이다. C의 전/후 종합 변화도 gripper 변화와 매우 가깝다.
   `numeric_scale_dominance.csv`의 제곱 성분 분해는 이 수치적 scale 지배를 보여주지만, 혼합 단위의 물리적 중요도나 mean L2의 가산 기여율로 해석하지 않는다.
3. **이동·회전에서도 개선되는가?** C 전처리 자체 및 A 평균 보정에서는 둘 다 개선됐다.
   다만 C 뒤 평균 보정은 이동 소폭 개선, 회전 악화이므로 종합 지표 개선을 전 성분 개선으로 일반화할 수 없다.
4. **기준 입력의 평균 보정 추가 효과는?** A→A_mean은 이동 32.69%, 회전 38.92%, 그리퍼 66.21% 감소했다.
   세 성분 모두 episode별 평균에서 5/5 개선이지만, 5개의 상관된 episode만으로 통계적·배포 일반화를 주장하지 않는다.
5. **여백 이후에도 추가 효과가 있는가?** C_mean은 C보다 이동 4.25%, 그리퍼 10.99%, 7D 10.93% 감소지만 회전 15.20% 악화했다.
   추가 효과는 제한적·성분 의존적이며 C 자체가 A_mean보다 이동/회전/그리퍼 모두 낮다. 보정이 전처리보다 우수하다는 결론은 지지되지 않는다.
6. **episode에 따라 다른가?** 그렇다. 특히 C episode 1의 전 성분 악화와 episode 2/4의 회전 악화를 숨기지 않는다.
7. **논문에서 가능한 주장:** 이 저장된 5개 episode의 paired 관측에서 입력 변환과 조건별 평균 특징 주입은 서로 다른 성분별 결과를 보였고,
   낮은 pixel MSE만으로 낮은 action disagreement를 예측할 수 없었다. A에서는 평균 보정의 다성분 개선,
   C에서는 추가 효과 감소 및 회전 trade-off가 관찰됐다.
   **불가능한 주장:** 실제 task 성공률·실제 제어 오차 개선, 배포 가능한 adapter, 새 환경/조명/장면 일반화,
   정식 Real-Sim pose 정합, 보정의 보편적 우월성, 독립 프레임 가정의 p-value/유의성, 그리퍼 정확도/물리적 안전성.

## 9. 재현과 산출물

실행 코드는 이 실험 폴더의 `analyze.py`, `precision_probe.py`, `legacy_baseline_probe.py`, `report.py`, `test_analysis.py`에 있다.
모든 결과는 새 `results_001/`에 저장했고 과거 실험 파일·config·모델 가중치를 덮어쓰지 않았다.
재현 명령은 상위 `REPRODUCE.md`를 참조한다. GPU head-only 계산은 RTX 3090, torch 2.2.0+cu121, CUDA 12.1에서 실행됐다.
학습/전체 backbone 추론/ROS/실제 로봇 실행은 없다.
