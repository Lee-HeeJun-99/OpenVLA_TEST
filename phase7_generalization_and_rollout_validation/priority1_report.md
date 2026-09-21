# Phase7 Priority1 Report

## 1. 연구 배경

Phase1-Phase6에서는 `oftplus_h5_vision` checkpoint step 28560 기준 5 episode / 225 Real-Sim paired frame에서 Observation Gap, Representation Gap, Policy-Relevant Gap, Action Gap을 분석했다. 핵심 출발점은 전체 representation distance가 아니라 실제 action prediction을 바꾸는 component를 찾는 것이다.

## 2. 기존 Phase 1부터 Phase 6 결과 요약

기존 결과는 P4 letterbox preprocessing, progress/phase-conditioned hidden correction, policy-sensitive low-rank basis가 offline Action Gap을 크게 줄일 수 있음을 보였다. 단 이 결과는 모두 동일 offline vision-policy track에 한정되며 current ROS `oftplus_h5_proprio` deployment 결과가 아니다.

## 3. 기존 결과의 한정성

기존 dataset은 5개 episode와 225 pair에 한정된다. Real pose는 measured feedback가 아니라 commanded/planned 계열로 기록된 것으로 취급한다. GT action, executed action, measured motion, Real task success는 검증되지 않았으므로 이번 재평가는 Real/Sim policy prediction disagreement로만 해석한다.

## 4. Audit 결과

- Policy variant: `oftplus_h5_vision`
- Checkpoint: `runtime_state/oft_mixed480_step28560_merged`
- Checkpoint step: 28560
- Instruction: `Pick up the orange cube.`
- Use proprio: false
- Pair count: 225
- Action chunk length: 5
- Action dimension: 7
- Priority2 started: false

세부 audit 산출물은 `00_audit/`에 저장했다.

## 5. 발견된 오류와 미해결 사항

현재 Priority1에서 치명적 계산 중단 사유는 발견하지 않았다. 다만 다음은 미해결이다.

- Gripper binary threshold는 code-level GT threshold가 아니라 `0.5` assumed threshold다.
- Stored action은 policy decoded output으로 확인했지만, planner / commanded / executed / measured action과의 정확도 비교는 불가하다.
- Correction method 전체에 대해 L1/RMSE/Huber를 재계산하려면 corrected action chunk 또는 action-head 재실행 산출물이 더 필요하다.
- P4+C2 camera 결합 결과는 shift 해상도 차이 때문에 물리적 non-additivity 확정 증거로 쓰면 안 된다.

## 6. Action 정의와 normalization

`OFTRuntime.predict()`의 `response['actions']`를 `policy_decoded_action`으로 사용했다. Model raw action은 `_unnormalize_actions()`를 통과하며, dataset statistics의 `q01/q99`와 mask를 사용하는 구조다. Action index는 다음처럼 정리했다.

```text
0,1,2: translation
3,4,5: rotation-like action components
6: gripper
```

Rotation representation의 물리 단위와 geodesic 해석은 아직 검증되지 않았으므로 이번 결과에서는 component disagreement로 보고한다.

## 7. L1, L2 norm, RMSE, Huber, cosine 결과

P0-P6 조건에 대해 저장된 decoded action chunk로 multi-metric을 재계산했다.

```text
frame_metrics rows: 1575
episode_metrics rows: 35
conditions: P0_current_paired_image, P1_resize_224_direct, P2_center_crop_square_resize_224, P3_center_crop_0p875_resize_224, P4_letterbox_224, P5_brightness_match_sim_to_real, P6_contrast_match_sim_to_real
```

조건별 1위 요약:

```text
action_l2_chunk_mean: P4_letterbox_224 (0.101993)
action_l1_chunk_mean: P4_letterbox_224 (0.106149)
action_rmse_chunk_mean: P4_letterbox_224 (0.038550)
action_huber0.05_chunk_mean: P4_letterbox_224 (0.004191)
action_cosine_chunk_mean: P2_center_crop_square_resize_224 (0.026256)
translation_l2_chunk_mean: P4_letterbox_224 (0.001774)
rotation_l2_chunk_mean: P4_letterbox_224 (0.001532)
gripper_abs_chunk_mean: P4_letterbox_224 (0.101700)
gripper_binary_disagreement_rate: P4_letterbox_224 (0.078222)
```

## 8. Translation, rotation, gripper 분리 결과

Translation, rotation, gripper는 별도 파일로 분리했다.

```text
01_multimetric_reanalysis/translation_metrics.csv
01_multimetric_reanalysis/rotation_metrics.csv
01_multimetric_reanalysis/gripper_event_metrics.csv
03_gripper_analysis/
```

주요 action L2 순위는 gripper absolute gap 순위와 거의 동일하다. 따라서 기존 Action Gap은 gripper dimension의 영향을 크게 받는 것으로 해석해야 한다.

## 9. Gripper timing 분석

Chunk 내부 gripper transition index를 Real/Sim 각각 계산했다. 단 이것은 policy prediction chunk 내부 disagreement이며, 실제 gripper close timing error가 아니다. GT gripper state가 없으므로 accuracy, precision, recall, false-open, false-close는 계산하지 않았다.

## 10. Episode-level 통계

Episode-level summary, episode-cluster bootstrap, paired method comparison을 `02_statistical_validation/`에 저장했다. Episode 수가 5개뿐이므로 frame-level 독립 표본처럼 해석하지 않고, episode-level paired difference와 개선 episode 수를 우선 근거로 사용한다.

## 11. Metric별 방법 순위

`01_multimetric_reanalysis/metric_robustness_summary.csv`에 metric별 condition rank를 저장했다. P4 letterbox는 action L1, L2, RMSE, Huber, translation, rotation, gripper continuous gap에서 일관되게 최상위로 나타났다.

## 12. Metric이 바뀌어도 유지되는 결론

- P4 letterbox는 주요 decoded-action disagreement metric에서 가장 강했다.
- P5 brightness matching은 observation/stat alignment 목적과 달리 action disagreement 개선에는 약하거나 악화되는 경향을 유지했다.
- 전체 action metric은 gripper component에 크게 민감하다.

## 13. Metric에 따라 달라지는 결론

- Translation/rotation만 보면 P0와 P1/P5/P6 사이의 차이는 전체 action L2만큼 크지 않다.
- Gripper binary disagreement는 assumed threshold에 의존하므로 continuous gripper gap과 분리해서 봐야 한다.
- Cosine distance는 action norm과 방향성 해석이 섞이므로 L1/L2/RMSE와 동일한 결론 지표로 과장하지 않는다.

## 14. 실행 완료 항목

- Phase1-Phase6 artifact audit
- Action definition / normalization audit
- L1, L2 norm, RMSE, Huber, cosine 재평가
- Translation / rotation / gripper 분리
- Gripper chunk timing disagreement 분석
- Episode-level statistics
- Metric별 condition ranking
- Priority2/Priority3/Priority4 protocol skeleton 생성

## 15. 실행하지 못한 항목

- GPU full-forward 재실험
- Camera shift sweep
- Letterbox 내부 ablation
- Global alignment full-forward
- 새 object/light/camera condition 데이터 수집
- Shadow mode / closed-loop rollout
- Real task success 분석

## 16. Priority 2에 필요한 데이터와 GPU 조건

Priority2에는 image-to-action full-forward 재실행이 필요하다. 특히 camera shift sweep, letterbox padding decomposition, correction method별 L1/RMSE/Huber 재평가는 GPU 또는 기존 corrected action chunk 재생성이 필요하다.

## 17. 해석 가능한 결론

현재 데이터 범위에서는 P4 letterbox가 여러 action disagreement metric에서 가장 robust한 preprocessing condition이다. 또한 observation-level 통계 정렬이 action-level improvement를 보장하지 않는다는 Phase6 결론은 metric audit 후에도 유지된다.

## 18. 주장하면 안 되는 결론

- P4가 Real robot success를 개선한다고 주장할 수 없다.
- Gripper gap을 gripper error 또는 false close/open으로 부르면 안 된다.
- 현재 결과를 `oftplus_h5_proprio` ROS deployment 결과로 해석하면 안 된다.
- 5 episode LOO 또는 P0-P6 재평가를 unseen layout / unseen lighting / rollout generalization으로 주장하면 안 된다.
