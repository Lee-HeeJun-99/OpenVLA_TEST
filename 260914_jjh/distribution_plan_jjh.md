# Real-Sim Token Distribution 분석 계획

## 목적

Real과 Sim 이미지 데이터를 동일한 image encoder에 통과시킨 뒤, token 또는 latent feature distribution의 차이를 분석한다.

핵심 질문은 다음과 같다.

- Real과 Sim observation이 encoder latent space에서 얼마나 떨어져 있는가?
- 그 차이가 단순한 distribution shift로 설명 가능한가?
- distribution shift 보정만으로 sim-to-real gap을 줄일 가능성이 있는가?
- 이 접근이 Real 데이터를 대량으로 다시 수집하지 않고도 Sim 기반 adaptation에 사용할 수 있는가?

이번 단계에서는 scene을 photoreal하게 미세 조정하는 것보다, Real/Sim 이미지를 같은 encoder로 처리했을 때 latent/token space에서 어떤 차이가 생기는지를 먼저 확인한다.

## 기준 데이터

Real 기준 데이터:

```text
/home/ubuntu/robot_ws/src/doosan-robot2/raw_dataset_oft/episodes/episode_000001
```

Real 이미지:

```text
/home/ubuntu/robot_ws/src/doosan-robot2/raw_dataset_oft/episodes/episode_000001/images/primary
```

Sim 기준 데이터:

```text
/home/ubuntu/a0509_vla_linux_field_bundle_20260903/outputs/sim2real_analysis/raw_dataset_oft_episode_000001_home_relative_joint_replay_images
```

Sim 이미지:

```text
/home/ubuntu/a0509_vla_linux_field_bundle_20260903/outputs/sim2real_analysis/raw_dataset_oft_episode_000001_home_relative_joint_replay_images/images/primary
```

Sim replay report:

```text
/home/ubuntu/a0509_vla_linux_field_bundle_20260903/outputs/sim2real_analysis/raw_dataset_oft_episode_000001_home_relative_joint_replay.json
```

## 전제 조건

- Real과 Sim은 같은 task를 수행한다.
- instruction은 동일하게 유지한다.

```text
Pick up the orange cube.
```

- target color는 `orange`이다.
- Real과 Sim 모두 5Hz frame sequence를 기준으로 한다.
- Real과 Sim 모두 left eye-in-hand camera 관측을 사용한다.
- Real과 Sim 이미지 해상도는 `1280x720`이다.
- Sim은 Real trajectory를 HOME-relative joint delta로 replay한다.
- joint_6의 Real/Sim 표현 차이는 정상으로 간주한다.
- EEF/TCP absolute world pose 수치는 좌표계 차이가 있으므로 직접 비교 기준으로 사용하지 않는다.

## 분석 산출물 구조

분석 결과는 아래 폴더에 저장한다.

```text
/home/ubuntu/a0509_vla_linux_field_bundle_20260903/outputs/token_distribution_analysis
```

예상 구조:

```text
outputs/token_distribution_analysis/
  features/
    real_episode_000001_tokens.pt
    sim_episode_000001_tokens.pt
    real_episode_000001_pooled.pt
    sim_episode_000001_pooled.pt
  metrics/
    summary.json
    frame_pair_metrics.csv
    phase_metrics.json
  plots/
    pca_pooled_features.png
    umap_pooled_features.png
    token_mean_shift.png
    frame_000000_patch_distance.png
    frame_000097_patch_distance.png
    frame_000119_patch_distance.png
  report.md
```

## Plan 1. Encoder와 token 추출 지점 결정

목표:

- 사용할 image encoder를 확정한다.
- encoder에서 어떤 representation을 뽑을지 정한다.

후보:

- OpenVLA/OFT vision encoder patch token
- pooled image feature
- CLS token 또는 equivalent pooled token
- action 직전 VLA hidden state

1차 분석에서는 원인 해석이 쉬운 아래 두 가지를 우선한다.

- patch token 전체
- pooled image feature

이후 필요하면 action 직전 hidden state나 action output까지 확장한다.

진행사항:

- 아직 시작 전.

## Plan 2. 동일 전처리 파이프라인 구성

목표:

- Real과 Sim 이미지가 완전히 같은 preprocess를 거치도록 한다.
- resize, crop, normalization, dtype, device 차이로 인한 가짜 distribution gap을 제거한다.

확인할 항목:

- image resize size
- crop 여부
- normalization mean/std
- channel order
- RGB/BGR
- dtype
- batch dimension
- eval mode

현재 runtime 참고값:

```text
image_resize_size = 224
center_crop_enabled = false
center_crop_scale = 0.9
```

단, crop은 disabled이므로 `center_crop_scale`은 현재 직접 적용되지 않는다.

진행사항:

- 아직 시작 전.

## Plan 3. Real/Sim frame alignment 정의

목표:

- 전체 episode distribution 비교와 frame-pair 비교를 동시에 수행한다.
- phase별 차이를 볼 수 있게 frame을 구간으로 나눈다.

비교 단위:

- 전체 frame distribution
- 같은 frame index pair
- phase별 distribution

대표 frame:

```text
000000: 초기 HOME/hold
000097: gripper close / attach 시점
000119: lift 후반
```

phase 후보:

- hold
- alignment / approach
- descent_to_grasp
- grasp close
- lift

진행사항:

- 아직 시작 전.

## Plan 4. Feature 추출 스크립트 작성

목표:

- Real/Sim 이미지 폴더를 입력받아 같은 encoder feature를 저장한다.
- 추출 결과를 재사용할 수 있도록 `.pt` 또는 `.npz`로 저장한다.

필요 기능:

- 이미지 리스트 정렬
- frame index 매칭
- instruction optional input 처리
- encoder load
- preprocess
- patch token 저장
- pooled feature 저장
- metadata 저장

예상 출력:

```text
features/real_episode_000001_tokens.pt
features/sim_episode_000001_tokens.pt
features/real_episode_000001_pooled.pt
features/sim_episode_000001_pooled.pt
```

진행사항:

- 아직 시작 전.

## Plan 5. Distribution metric 계산

목표:

- Real과 Sim feature distribution의 차이를 수치화한다.

1차 metric:

- pooled feature cosine distance
- frame-pair L2 distance
- token mean difference
- token std difference
- covariance difference
- MMD
- Wasserstein distance

추가 metric:

- phase별 평균 distance
- frame별 distance time-series
- patch별 평균 distance

진행사항:

- 아직 시작 전.

## Plan 6. 시각화

목표:

- Real/Sim feature가 latent space에서 어떻게 분리되는지 시각적으로 확인한다.
- 어느 이미지 영역에서 token gap이 큰지 확인한다.

시각화:

- PCA pooled feature plot
- UMAP pooled feature plot
- frame별 distance curve
- token mean shift bar/heatmap
- patch token distance heatmap

대표 heatmap 대상:

```text
frame_000000
frame_000097
frame_000119
```

진행사항:

- 아직 시작 전.

## Plan 7. Latent shift 보정 가능성 확인

목표:

- 단순 distribution alignment로 Real/Sim feature gap이 줄어드는지 확인한다.

후보 방법:

- mean shift correction
- variance normalization
- whitening
- CORAL
- simple linear mapping

처음에는 학습 없는 방법부터 확인한다.

검증:

- 보정 전/후 pooled feature distance 비교
- 보정 전/후 MMD 비교
- 보정 전/후 PCA/UMAP overlap 비교

진행사항:

- 아직 시작 전.

## Plan 8. Action 영향 확인

목표:

- latent shift 보정이 실제 policy action에 어떤 영향을 주는지 확인한다.

분석:

- Real image action
- Sim image action
- corrected Sim latent action
- action delta 비교
- gripper command 변화 확인

주의:

- 1차 token distribution 분석 후 진행한다.
- action까지 보면 decoder/policy 영향이 섞이므로 초반에는 feature 분석과 분리한다.

진행사항:

- 아직 시작 전.

## Plan 9. 결과 보고서 작성

목표:

- 분석 결과를 사람이 읽을 수 있는 형태로 정리한다.
- 어떤 gap이 task-relevant한지 판단한다.

보고서 항목:

- 사용한 encoder/checkpoint
- 사용한 preprocess
- 사용한 frame 수
- 전체 distribution metric
- phase별 metric
- 대표 frame heatmap
- latent shift 보정 전/후 metric
- 해석
- 다음 실험 제안

진행사항:

- 아직 시작 전.
