# Real-Sim Token Distribution 분석 진행 결과

이 파일은 `distribution_plan_jjh.md`의 각 계획 단락에 대응하는 실제 진행 결과를 기록한다.

## 공통 기준

Real 기준 episode:

```text
/home/ubuntu/robot_ws/src/doosan-robot2/raw_dataset_oft/episodes/episode_000001
```

Sim 기준 episode:

```text
/home/ubuntu/a0509_vla_linux_field_bundle_20260903/outputs/sim2real_analysis/raw_dataset_oft_episode_000001_home_relative_joint_replay_images
```

출력 예정 위치:

```text
/home/ubuntu/a0509_vla_linux_field_bundle_20260903/outputs/token_distribution_analysis
```

## Plan 1. Encoder와 token 추출 지점 결정

상태:

- 1차 확인 완료.

결과:

- 현재 번들에서 기준으로 삼을 1순위 encoder는 `runtime_state/oft_mixed480_step28560_merged`의 OpenVLA vision backbone이다.
- 구조:
  - `model_type = openvla`
  - `vision_backbone_id = dinosiglip-vit-so-224px`
  - `use_fused_vision_backbone = True`
  - 내부 TIMM backbone:
    - `vit_large_patch14_reg4_dinov2.lvd142m`
    - `vit_so400m_patch14_siglip_224`
  - image resize strategy:
    - `resize-naive`
- 비교용 후보:
  - `models/base_openvla_7b`: base OpenVLA 7B. generic/pretrained vision latent gap 확인용.
  - `runtime_state/oft_mixed480_step28560_merged`: 현재 실행/분석 기준 OFT merged 모델. 실제 정책이 보는 latent gap 확인용.
  - `models/oft_mixed480_step28560/vision_backbone--28560_checkpoint.pt`: OFT step 28560 vision backbone checkpoint.
  - `models/vanilla_s1_balanced_step8130`: adapter와 processor만 확인됨. 단독 full encoder weight 후보로는 우선순위 낮음.
- 1차 분석 기준:
  - primary encoder는 `runtime_state/oft_mixed480_step28560_merged`.
  - token 추출은 fused backbone의 patch-level token과 pooled/global feature를 우선 대상으로 한다.
  - base OpenVLA encoder 비교는 optional ablation으로 둔다.

## Plan 2. 동일 전처리 파이프라인 구성

상태:

- 완료.

결과:

- 전처리 확인/시각화 스크립트 추가:
  - `/home/ubuntu/a0509_vla_linux_field_bundle_20260903/sim2real_analysis/04_features/prepare_oft_encoder_inputs.py`
- 사용한 모델 전처리 설정:
  - `/home/ubuntu/a0509_vla_linux_field_bundle_20260903/runtime_state/oft_mixed480_step28560_merged/preprocessor_config.json`
  - `image_resize_strategy = resize-naive`
  - `use_fused_vision_backbone = true`
  - input stream:
    - DINOv2 stream: `3x224x224`, mean/std = ImageNet 계열
    - SigLIP stream: `3x224x224`, mean/std = `[0.5, 0.5, 0.5] / [0.5, 0.5, 0.5]`
  - 최종 encoder input:
    - `[1, 6, 224, 224]`
- 전처리된 이미지 시각화 저장:
  - `/home/ubuntu/a0509_vla_linux_field_bundle_20260903/outputs/token_distribution_analysis/preprocessed_inputs/contact_sheet_dinov2.png`
  - `/home/ubuntu/a0509_vla_linux_field_bundle_20260903/outputs/token_distribution_analysis/preprocessed_inputs/contact_sheet_siglip.png`
  - 개별 frame:
    - `/home/ubuntu/a0509_vla_linux_field_bundle_20260903/outputs/token_distribution_analysis/preprocessed_inputs/real/dinov2`
    - `/home/ubuntu/a0509_vla_linux_field_bundle_20260903/outputs/token_distribution_analysis/preprocessed_inputs/real/siglip`
    - `/home/ubuntu/a0509_vla_linux_field_bundle_20260903/outputs/token_distribution_analysis/preprocessed_inputs/sim/dinov2`
    - `/home/ubuntu/a0509_vla_linux_field_bundle_20260903/outputs/token_distribution_analysis/preprocessed_inputs/sim/siglip`
- 대표 frame:
  - `000000`, `000030`, `000060`, `000090`, `000112`
- paired frame count:
  - Real 120장 / Sim 120장

## Plan 3. Real/Sim frame alignment 정의

상태:

- 1차 완료.

결과:

- Real/Sim 모두 `000000.jpg`부터 `000119.jpg`까지 120개 frame이 존재한다.
- 현재 1차 기준은 same index pairing이다.
- 향후 metric 단계에서 전체 episode distribution과 frame-pair distance를 같이 계산한다.

## Plan 4. Feature 추출 스크립트 작성

상태:

- 완료.

결과:

- 기존 feature extractor 사용:
  - `/home/ubuntu/a0509_vla_linux_field_bundle_20260903/sim2real_analysis/04_features/extract_vla_features.py`
- 실행 모델:
  - `model = oft`
  - checkpoint:
    - `/home/ubuntu/a0509_vla_linux_field_bundle_20260903/runtime_state/oft_mixed480_step28560_merged`
  - instruction:
    - `Pick up the orange cube.`
- smoke test:
  - Real 2 frame 추출 성공.
- 전체 feature 추출 결과:
  - Real 120 records:
    - `/home/ubuntu/a0509_vla_linux_field_bundle_20260903/outputs/token_distribution_analysis/features/real_episode_000001/feature_manifest.json`
  - Sim 120 records:
    - `/home/ubuntu/a0509_vla_linux_field_bundle_20260903/outputs/token_distribution_analysis/features/sim_episode_000001/feature_manifest.json`
  - 총 `.npz` feature file:
    - 240개
- 저장 tensor shape:
  - `vision_backbone.input`: `[1, 6, 224, 224]`
  - `vision_backbone.output`: `[1, 256, 2176]`
  - `projector.output`: `[1, 256, 4096]`
  - `action_hidden_states.input`: `[1, 35, 4096]`
  - `action_head.output`: `[1, 5, 7]`

## Plan 5. Distribution metric 계산

상태:

- 완료.

결과:

- metric 계산 스크립트 추가:
  - `/home/ubuntu/a0509_vla_linux_field_bundle_20260903/sim2real_analysis/10_analysis/compute_token_distribution_metrics.py`
- 입력:
  - Real feature manifest:
    - `/home/ubuntu/a0509_vla_linux_field_bundle_20260903/outputs/token_distribution_analysis/features/real_episode_000001/feature_manifest.json`
  - Sim feature manifest:
    - `/home/ubuntu/a0509_vla_linux_field_bundle_20260903/outputs/token_distribution_analysis/features/sim_episode_000001/feature_manifest.json`
- 출력:
  - `/home/ubuntu/a0509_vla_linux_field_bundle_20260903/outputs/token_distribution_analysis/metrics/summary.json`
  - `/home/ubuntu/a0509_vla_linux_field_bundle_20260903/outputs/token_distribution_analysis/metrics/frame_pair_metrics.csv`
- paired frame count:
  - 120
- `vision_backbone.output` pooled distribution:
  - `mean_shift_l2 = 73.70889950227772`
  - `mean_shift_cosine_distance = 0.12249472372732839`
  - `std_shift_l2 = 22.95354974604635`
  - `mmd2_biased = 0.4031993010537791`
  - frame-pair pooled cosine distance mean:
    - `0.15473036639858434`
  - frame-pair pooled L2 mean:
    - `87.53407226679126`
- `projector.output` pooled distribution:
  - `mean_shift_l2 = 8.503429903515912`
  - `mean_shift_cosine_distance = 0.07209695522289783`
  - `std_shift_l2 = 2.6894444546536804`
  - `mmd2_biased = 0.3979356379846253`
  - frame-pair pooled cosine distance mean:
    - `0.10400031654125691`
  - frame-pair pooled L2 mean:
    - `9.963901820080556`
- high-gap frames:
  - vision pooled cosine top frames:
    - `000075`, `000076`, `000081`, `000091`, `000094`
  - projector pooled cosine top frames:
    - `000081`, `000083`, `000082`, `000080`, `000089`
- 1차 해석:
  - projector 이후 cosine gap이 `0.1225 -> 0.0721`로 줄어든다.
  - frame-pair 평균 cosine도 `0.1547 -> 0.1040`로 줄어든다.
  - 따라서 projector가 일부 Real/Sim visual gap을 흡수하는 경향이 있다.
  - high-gap frame이 grasp 접근/접촉 전후 구간에 몰려 있어, 물체-EEF 상대 위치와 contact 전후 장면 차이가 latent gap을 키울 가능성이 있다.

### Gripper transition mismatch 구간 제외 재계산

문제 확인:

- frame `000097` 부근에서 급격한 latent 변화가 관찰됨.
- 원인 후보를 이미지로 확인한 결과, Real과 Sim의 gripper open width 및 closing transition timing이 다름.
- Real은 gripper가 닫히는 중간 상태가 여러 frame에 걸쳐 수집되었지만, Sim은 open 상태에서 close 상태로 갑자기 바뀌는 형태에 가까움.
- 따라서 해당 구간을 그대로 포함하면 Real/Sim domain gap이 아니라 gripper replay/postprocess mismatch가 distribution metric에 섞일 수 있음.

해결 방식:

- 사용자가 지정한 frame `000087`~`000096`을 분석 대상에서 제외.
- 원본 feature 파일은 유지하고, metrics/plots만 제외 조건으로 재생성.
- metric 스크립트에 `--exclude-frame-range 087:096` 옵션 추가.

제외 후 출력:

- `/home/ubuntu/a0509_vla_linux_field_bundle_20260903/outputs/token_distribution_analysis/metrics_exclude_087_096/summary.json`
- `/home/ubuntu/a0509_vla_linux_field_bundle_20260903/outputs/token_distribution_analysis/metrics_exclude_087_096/frame_pair_metrics.csv`

제외 후 paired frame count:

- `110`

제외 후 `vision_backbone.output`:

- `mean_shift_l2 = 72.5429`
- `mean_shift_cosine_distance = 0.118143`
- `mmd2_biased = 0.403350`
- frame-pair pooled cosine mean:
  - `0.150379`
- frame-pair pooled cosine max:
  - `0.214854`

제외 후 `projector.output`:

- `mean_shift_l2 = 8.5299`
- `mean_shift_cosine_distance = 0.068784`
- `mmd2_biased = 0.404602`
- frame-pair pooled cosine mean:
  - `0.099830`
- frame-pair pooled cosine max:
  - `0.160381`

제외 전/후 비교:

- `vision_backbone.output` frame-pair cosine mean:
  - `0.154730 -> 0.150379`
- `projector.output` frame-pair cosine mean:
  - `0.104000 -> 0.099830`
- transition mismatch 제거 후 평균 gap은 소폭 감소함.
- 하지만 high-gap 자체는 여전히 남아 있으므로, frame `087`~`096`의 gripper mismatch가 전체 gap의 유일한 원인은 아님.

## Plan 6. 시각화

상태:

- 완료.

결과:

- 시각화 스크립트 추가:
  - `/home/ubuntu/a0509_vla_linux_field_bundle_20260903/sim2real_analysis/10_analysis/plot_token_distribution.py`
- 출력:
  - `/home/ubuntu/a0509_vla_linux_field_bundle_20260903/outputs/token_distribution_analysis/plots/frame_pair_distance_curves.png`
  - `/home/ubuntu/a0509_vla_linux_field_bundle_20260903/outputs/token_distribution_analysis/plots/pca_pooled_vision_backbone_output.png`
  - `/home/ubuntu/a0509_vla_linux_field_bundle_20260903/outputs/token_distribution_analysis/plots/pca_pooled_projector_output.png`
  - `/home/ubuntu/a0509_vla_linux_field_bundle_20260903/outputs/token_distribution_analysis/plots/top_projector_gap_frames_contact_sheet.png`
  - `/home/ubuntu/a0509_vla_linux_field_bundle_20260903/outputs/token_distribution_analysis/plots/plot_manifest.json`
- 확인 내용:
  - frame distance curve는 초반 HOME/hold 구간보다 approach/grasp 구간에서 Real/Sim gap이 커지는 패턴을 보인다.
  - `vision_backbone.output` gap이 `projector.output` gap보다 전반적으로 크다.
  - PCA에서는 Real과 Sim이 시간 진행 방향은 유사하게 따라가지만, 같은 frame pair가 일정한 방향으로 분리되어 있다.
  - 이는 장면이 완전히 랜덤하게 섞이는 형태보다, domain-level shift 성격이 있음을 시사한다.
  - top projector gap frame contact sheet 기준으로 가장 큰 gap은 `000080`~`000089` 주변 grasp 접근/접촉 전후에서 나타난다.

추가 세부 시각화:

- 세부 시각화 스크립트 추가:
  - `/home/ubuntu/a0509_vla_linux_field_bundle_20260903/sim2real_analysis/10_analysis/plot_token_gap_details.py`
- 출력:
  - `/home/ubuntu/a0509_vla_linux_field_bundle_20260903/outputs/token_distribution_analysis/plots/details/phase_cosine_boxplot.png`
  - `/home/ubuntu/a0509_vla_linux_field_bundle_20260903/outputs/token_distribution_analysis/plots/details/smoothed_frame_gap_with_phases.png`
  - `/home/ubuntu/a0509_vla_linux_field_bundle_20260903/outputs/token_distribution_analysis/plots/details/phase_metrics.json`
  - `/home/ubuntu/a0509_vla_linux_field_bundle_20260903/outputs/token_distribution_analysis/plots/details/token_l2_heatmap_vision_backbone_output_000000.png`
  - `/home/ubuntu/a0509_vla_linux_field_bundle_20260903/outputs/token_distribution_analysis/plots/details/token_l2_heatmap_vision_backbone_output_000075.png`
  - `/home/ubuntu/a0509_vla_linux_field_bundle_20260903/outputs/token_distribution_analysis/plots/details/token_l2_heatmap_vision_backbone_output_000081.png`
  - `/home/ubuntu/a0509_vla_linux_field_bundle_20260903/outputs/token_distribution_analysis/plots/details/token_l2_heatmap_vision_backbone_output_000089.png`
  - `/home/ubuntu/a0509_vla_linux_field_bundle_20260903/outputs/token_distribution_analysis/plots/details/token_l2_heatmap_vision_backbone_output_000112.png`
  - `/home/ubuntu/a0509_vla_linux_field_bundle_20260903/outputs/token_distribution_analysis/plots/details/token_l2_heatmap_projector_output_000000.png`
  - `/home/ubuntu/a0509_vla_linux_field_bundle_20260903/outputs/token_distribution_analysis/plots/details/token_l2_heatmap_projector_output_000075.png`
  - `/home/ubuntu/a0509_vla_linux_field_bundle_20260903/outputs/token_distribution_analysis/plots/details/token_l2_heatmap_projector_output_000081.png`
  - `/home/ubuntu/a0509_vla_linux_field_bundle_20260903/outputs/token_distribution_analysis/plots/details/token_l2_heatmap_projector_output_000089.png`
  - `/home/ubuntu/a0509_vla_linux_field_bundle_20260903/outputs/token_distribution_analysis/plots/details/token_l2_heatmap_projector_output_000112.png`
- phase별 pooled cosine mean:
  - `home_hold`:
    - vision `0.0866`, projector `0.0396`
  - `approach`:
    - vision `0.1520`, projector `0.0954`
  - `grasp_contact`:
    - vision `0.1993`, projector `0.1500`
  - `lift`:
    - vision `0.1683`, projector `0.1300`
- 세부 해석:
  - gap은 `home_hold < approach < grasp_contact` 순서로 증가한다.
  - lift에서는 grasp_contact보다 줄지만 home/approach보다 여전히 크다.
  - token L2 heatmap은 gap이 모든 patch에 균일하지 않고 특정 patch 영역에 집중됨을 보여준다.
  - 따라서 현재 gap은 단순 전체 색감/밝기 차이만으로 보기 어렵고, 물체-EEF 상대 위치, contact 근처 기하 차이, shadow/table edge/robot appearance 차이가 encoder token에 국소적으로 들어간 것으로 해석된다.

### Gripper transition mismatch 구간 제외 후 시각화

출력:

- `/home/ubuntu/a0509_vla_linux_field_bundle_20260903/outputs/token_distribution_analysis/plots_exclude_087_096/frame_pair_distance_curves.png`
- `/home/ubuntu/a0509_vla_linux_field_bundle_20260903/outputs/token_distribution_analysis/plots_exclude_087_096/pca_pooled_vision_backbone_output.png`
- `/home/ubuntu/a0509_vla_linux_field_bundle_20260903/outputs/token_distribution_analysis/plots_exclude_087_096/pca_pooled_projector_output.png`
- `/home/ubuntu/a0509_vla_linux_field_bundle_20260903/outputs/token_distribution_analysis/plots_exclude_087_096/top_projector_gap_frames_contact_sheet.png`
- `/home/ubuntu/a0509_vla_linux_field_bundle_20260903/outputs/token_distribution_analysis/plots_exclude_087_096/details/phase_cosine_boxplot.png`
- `/home/ubuntu/a0509_vla_linux_field_bundle_20260903/outputs/token_distribution_analysis/plots_exclude_087_096/details/smoothed_frame_gap_with_phases.png`
- `/home/ubuntu/a0509_vla_linux_field_bundle_20260903/outputs/token_distribution_analysis/plots_exclude_087_096/details/phase_metrics.json`

제외 후 phase별 pooled cosine mean:

- `home_hold`:
  - vision `0.0866`, projector `0.0396`
- `approach`:
  - vision `0.1520`, projector `0.0954`
- `grasp_contact`:
  - count `9`
  - vision `0.1965`, projector `0.1502`
- `lift`:
  - count `23`
  - vision `0.1666`, projector `0.1291`

해석:

- frame `087`~`096` 제거 후에도 `grasp_contact` 구간의 projector gap은 가장 높다.
- 급격한 gripper close mismatch는 제거했지만, gripper/cube가 가까운 구간의 latent gap은 여전히 유지된다.
- 따라서 후속 Plan 7 latent shift 보정에서는 `metrics_exclude_087_096`와 `plots_exclude_087_096`를 기준으로 진행하는 것이 더 안전하다.

## Plan 7. Latent shift 보정 가능성 확인

상태:

- 완료.

결과:

- 보정 방식:
  - frame `000087`~`000096` 제외 기준 사용.
  - pooled latent feature 기준으로 `shift = real_mean - sim_mean` 계산.
  - `sim_corrected = sim + shift` 적용.
  - `vision_backbone.output`, `projector.output` 각각 평가.
- 스크립트:
  - `/home/ubuntu/a0509_vla_linux_field_bundle_20260903/sim2real_analysis/10_analysis/evaluate_latent_shift_correction.py`
- 출력:
  - `/home/ubuntu/a0509_vla_linux_field_bundle_20260903/outputs/token_distribution_analysis/shift_correction_exclude_087_096/shift_summary.json`
  - `/home/ubuntu/a0509_vla_linux_field_bundle_20260903/outputs/token_distribution_analysis/shift_correction_exclude_087_096/frame_pair_before_after.csv`
  - `/home/ubuntu/a0509_vla_linux_field_bundle_20260903/outputs/token_distribution_analysis/shift_correction_exclude_087_096/frame_pair_before_after_shift.png`
  - `/home/ubuntu/a0509_vla_linux_field_bundle_20260903/outputs/token_distribution_analysis/shift_correction_exclude_087_096/pca_before_after_shift_vision_backbone_output.png`
  - `/home/ubuntu/a0509_vla_linux_field_bundle_20260903/outputs/token_distribution_analysis/shift_correction_exclude_087_096/pca_before_after_shift_projector_output.png`
- paired frame count:
  - `110`

`vision_backbone.output` 결과:

- shift norm:
  - `72.5429`
- distribution mean shift:
  - before L2 `72.542869`
  - after L2 `0.0`
  - before cosine `0.118143`
  - after cosine `0.0`
- MMD:
  - before `0.403350`
  - after `0.046156`
- frame-pair cosine:
  - before mean `0.150379`
  - after mean `0.050054`
  - improvement mean `0.100325`
- frame-pair L2:
  - before mean `86.1395`
  - after mean `47.2422`
  - improvement mean `38.8974`

`projector.output` 결과:

- shift norm:
  - `8.5299`
- distribution mean shift:
  - before L2 `8.529868`
  - after L2 `0.0`
  - before cosine `0.068784`
  - after cosine `0.0`
- MMD:
  - before `0.404602`
  - after `0.055993`
- frame-pair cosine:
  - before mean `0.099830`
  - after mean `0.040267`
  - improvement mean `0.059563`
- frame-pair L2:
  - before mean `9.9329`
  - after mean `5.1638`
  - improvement mean `4.7691`

Phase별 `projector.output` cosine:

- `home_hold`:
  - before `0.0396`
  - after `0.0433`
  - improvement `-0.0037`
- `approach`:
  - before `0.0954`
  - after `0.0320`
  - improvement `0.0634`
- `grasp_contact`:
  - before `0.1502`
  - after `0.0581`
  - improvement `0.0921`
- `lift`:
  - before `0.1291`
  - after `0.0545`
  - improvement `0.0746`

해석:

- 단일 global latent shift만으로도 frame-pair cosine과 MMD가 크게 줄어든다.
- 이는 Real/Sim 차이에 일정한 domain shift 성분이 상당히 포함되어 있음을 의미한다.
- 다만 `home_hold`의 `projector.output`은 소폭 악화되므로, 하나의 shift가 모든 phase에 최적인 것은 아니다.
- 효과는 `approach`, `grasp_contact`, `lift`에서 크다.
- 따라서 다음 후보는 global shift보다 phase-conditioned shift 또는 progress-conditioned shift이다.

### Phase-conditioned latent shift correction

보정 방식:

- frame `000087`~`000096` 제외 기준 사용.
- phase별로 `shift_phase = real_phase_mean - sim_phase_mean` 계산.
- 각 frame의 phase에 해당하는 shift를 Sim pooled feature에 적용.
- phase 정의:
  - `home_hold`: `000000`~`000013`
  - `approach`: `000014`~`000077`
  - `grasp_contact`: `000078`~`000095`, 단 `000087`~`000095` 제외 후 9 frame만 사용
  - `lift`: `000096`~`000119`, 단 `000096` 제외 후 23 frame 사용
- 스크립트:
  - `/home/ubuntu/a0509_vla_linux_field_bundle_20260903/sim2real_analysis/10_analysis/evaluate_phase_shift_correction.py`
- 출력:
  - `/home/ubuntu/a0509_vla_linux_field_bundle_20260903/outputs/token_distribution_analysis/phase_shift_correction_exclude_087_096/phase_shift_summary.json`
  - `/home/ubuntu/a0509_vla_linux_field_bundle_20260903/outputs/token_distribution_analysis/phase_shift_correction_exclude_087_096/frame_pair_before_after.csv`
  - `/home/ubuntu/a0509_vla_linux_field_bundle_20260903/outputs/token_distribution_analysis/phase_shift_correction_exclude_087_096/frame_pair_before_after_phase_shift.png`
  - `/home/ubuntu/a0509_vla_linux_field_bundle_20260903/outputs/token_distribution_analysis/phase_shift_correction_exclude_087_096/pca_before_after_phase_shift_vision_backbone_output.png`
  - `/home/ubuntu/a0509_vla_linux_field_bundle_20260903/outputs/token_distribution_analysis/phase_shift_correction_exclude_087_096/pca_before_after_phase_shift_projector_output.png`

Global shift 대비 결과:

- `vision_backbone.output`:
  - global after frame cosine mean:
    - `0.050054`
  - phase after frame cosine mean:
    - `0.028238`
  - global after MMD:
    - `0.046156`
  - phase after MMD:
    - `0.009806`
- `projector.output`:
  - global after frame cosine mean:
    - `0.040267`
  - phase after frame cosine mean:
    - `0.021368`
  - global after MMD:
    - `0.055993`
  - phase after MMD:
    - `0.015292`

Phase별 `projector.output` cosine:

- `home_hold`:
  - `0.0396 -> 0.0048`
- `approach`:
  - `0.0954 -> 0.0291`
- `grasp_contact`:
  - `0.1502 -> 0.0076`
- `lift`:
  - `0.1291 -> 0.0154`

해석:

- phase-conditioned shift는 global shift보다 모든 phase에서 더 강하게 gap을 줄인다.
- global shift에서 살짝 악화됐던 `home_hold`도 phase shift에서는 크게 개선된다.
- 이는 Real/Sim gap이 하나의 고정 shift라기보다 task progress/phase에 따라 다른 shift 성분을 가진다는 뜻이다.
- 단, phase 경계에서 보정값이 불연속으로 바뀌므로 실제 policy 적용 후보로는 smooth progress-conditioned shift가 더 자연스럽다.

### Progress-conditioned smooth latent shift correction

보정 방식:

- frame `000087`~`000096` 제외 기준 사용.
- frame-local window 기반으로 부드러운 shift를 계산.
- 기본 window:
  - `11`
- 각 frame `t`에 대해 주변 frame 평균으로 계산:
  - `shift(t) = mean(real[t-5:t+5]) - mean(sim[t-5:t+5])`
  - 실제 구현은 제외 frame 이후 남은 frame index 기준으로, frame number 거리가 window half 이내인 sample들을 사용.
- `sim_corrected[t] = sim[t] + shift(t)`
- 스크립트:
  - `/home/ubuntu/a0509_vla_linux_field_bundle_20260903/sim2real_analysis/10_analysis/evaluate_progress_shift_correction.py`
- 출력:
  - `/home/ubuntu/a0509_vla_linux_field_bundle_20260903/outputs/token_distribution_analysis/progress_shift_correction_exclude_087_096/progress_shift_summary.json`
  - `/home/ubuntu/a0509_vla_linux_field_bundle_20260903/outputs/token_distribution_analysis/progress_shift_correction_exclude_087_096/frame_pair_before_after.csv`
  - `/home/ubuntu/a0509_vla_linux_field_bundle_20260903/outputs/token_distribution_analysis/progress_shift_correction_exclude_087_096/frame_pair_before_global_phase_progress.png`
  - `/home/ubuntu/a0509_vla_linux_field_bundle_20260903/outputs/token_distribution_analysis/progress_shift_correction_exclude_087_096/shift_norm_by_frame.png`
  - `/home/ubuntu/a0509_vla_linux_field_bundle_20260903/outputs/token_distribution_analysis/progress_shift_correction_exclude_087_096/pca_before_after_progress_shift_vision_backbone_output.png`
  - `/home/ubuntu/a0509_vla_linux_field_bundle_20260903/outputs/token_distribution_analysis/progress_shift_correction_exclude_087_096/pca_before_after_progress_shift_projector_output.png`

Global/Phase/Progress 비교:

- `vision_backbone.output` frame-pair cosine mean:
  - before:
    - `0.150379`
  - global shift after:
    - `0.050054`
  - phase shift after:
    - `0.028238`
  - progress shift after:
    - `0.005153`
- `vision_backbone.output` MMD:
  - before:
    - `0.403350`
  - progress shift after:
    - `0.000124`
- `vision_backbone.output` frame-pair L2 mean:
  - before:
    - `86.1395`
  - progress shift after:
    - `14.8025`

- `projector.output` frame-pair cosine mean:
  - before:
    - `0.099830`
  - global shift after:
    - `0.040267`
  - phase shift after:
    - `0.021368`
  - progress shift after:
    - `0.003553`
- `projector.output` MMD:
  - before:
    - `0.404602`
  - progress shift after:
    - `0.000090`
- `projector.output` frame-pair L2 mean:
  - before:
    - `9.9329`
  - progress shift after:
    - `1.4811`

Progress shift norm:

- `vision_backbone.output`:
  - mean `83.8704`
  - min `61.7034`
  - max `98.9353`
- `projector.output`:
  - mean `9.7363`
  - min `6.6506`
  - max `11.0840`

해석:

- progress-conditioned shift는 global/phase shift보다 더 강하게 Real/Sim frame-pair gap을 줄인다.
- shift norm이 frame 진행에 따라 변하므로, Real/Sim gap은 task progress에 따라 연속적으로 변하는 성격이 있다.
- 단, 이 결과는 같은 episode의 주변 Real frame을 사용해 shift를 계산했기 때문에 보정 가능성의 상한에 가깝다.
- 따라서 이 결과를 "real 데이터 없이 바로 가능"으로 해석하면 안 된다.
- 올바른 다음 검증은 소량 Real calibration 또는 leave-one-frame/leave-one-episode 방식으로 shift를 추정하고, 보정에 사용하지 않은 frame/episode에서 효과가 유지되는지 확인하는 것이다.

## Plan 8. Action 영향 확인

상태:

- 대기 중.

결과:

- 아직 없음.

## Plan 9. 결과 보고서 작성

상태:

- 대기 중.

결과:

- 아직 없음.
