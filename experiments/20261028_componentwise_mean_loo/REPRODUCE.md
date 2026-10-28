# 재현 명령

이 폴더는 2026-10-28의 offline 재평가 전용이다. ROS·robot client·학습·전체 모델 추론을 사용하지 않는다.
기존 Phase4~7 및 physical rollout 파일은 수정하지 않는다.

실행 기준 commit: `f8246eb3781b5c0a7d2c32d1a22dfe0b532cb6a4`, branch `lhj-research`.
dirty worktree와 checkpoint/config/hash는 `results_001/audit.json`에 기록했다.

```bash
cd /home/ubuntu/a0509_vla_linux_field_bundle_20260903/lhj
PY=/home/ubuntu/a0509_vla_linux_field_bundle_20260903/environment/a6000_ubuntu22_py310/bin/python
EXP=experiments/20261028_componentwise_mean_loo
# results_002는 존재하지 않는 새 폴더여야 한다. results_001을 덮어쓰지 않는다.
OUT=$EXP/results_002

# 선택: 첫 샘플의 FP32/BF16 수치 차이 재현 (GPU 권장)
$PY "$EXP/precision_probe.py"

# 1. 추론 없는 저장 예측/실제 입력 tensor의 재집계
$PY "$EXP/analyze.py" --stage cached --output "$OUT"

# 2. 전체 저장 예측 일치 검사 → 통과할 때만 조건별 평균 LOO
# GPU 사용 필요. 원래 BF16 batch=1을 유지하고 CPU/FP32로 자동 대체하지 않는다.
$PY "$EXP/analyze.py" --stage head --output "$OUT"

# 3. 과거 0.370245 기준값 차이의 별도 FP32 재현
$PY "$EXP/legacy_baseline_probe.py" "$OUT"

# 4. 연구 보고서·provenance·독립 fold 감사
$PY "$EXP/report.py" "$OUT"

# 5. metrics 단위 테스트 및 results_001이 있으면 artifact 검증
$PY -m unittest discover -s "$EXP" -p 'test_*.py' -v
```

필수 결과:

- `paper_table1.csv`, `paper_table2.csv`: 요청한 논문용 두 표
- `observation_metrics.csv`: 5개 condition/method 조합 × 225쌍 = 1,125행
- `episode_metrics.csv`: 25행, `overall_metrics.csv`: 5행
- `correction_effects.csv`: 같은 입력의 보정 전후 절대/상대 변화
- `folds.json`, `mean_correction_vectors.npz`: A/C 각각 5-fold train/test ID와 FP64 원소별 평균
- `replayed_uncorrected_actions.npz`, `corrected_actions.npz`: 5×7 행동 배열
- `head_validation.json`, `head_replay_validation_per_observation.csv`: 1,350개 원래 예측 일치 검사
- `cached_*`: head 재계산보다 먼저 산출한 추론 없는 결과
- `representative_raw_paired_and_model.png`: raw/paired/실제 model input 비교
- `representative/model_input_tensors.npz`: 기록된 실제 normalized 입력 (1×6×224×224)
- `source_inventory.csv`, `audit.json`, `additional_source_provenance.json`, `snapshots/`: 출처 및 설정
- `legacy_precision_explanation.json`: 0.370245의 별도 재현. primary 결과에 혼합하지 않는다.
- `RESULTS_REPORT.md`: 해석·제약·개선/악화 전체 보고

픽셀 MSE의 주 지표는 기록된 backbone 1 tensor를 고정 mean/std로 역변환한 float RGB 공간이다.
정규화 tensor의 MSE, JPEG RGB MSE, 시각화 PNG의 양자화된 MSE를 섞지 않는다.
별도 `pixel_mse_condition_jpeg`, `pixel_mse_model_rgb_branch2`도 저장했다.

기존 데이터 전체와 GPU 환경이 필요하다. 저장된 features가 없는 새 입력에 대한 adapter가 아니다.
