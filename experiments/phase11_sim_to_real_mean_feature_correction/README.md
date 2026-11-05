# Sim → Real 평균 은닉 특징 보정

저장된 225쌍의 `action_hidden_states.input` (1×35×4096)을 사용해 5개 에피소드 Leave-One-Episode-Out 보정을 평가한다. A와 B는 기존 Phase6 입력 및 특징을 사용한다. C1은 Real에 `mean(h_sim-h_real)`을, C2는 Sim에 `mean(h_real-h_sim)`을 더한 뒤 step 28560의 고정 행동 헤드를 실행한다. 모든 조건을 같은 실행 환경에서 다시 계산한다.

```bash
cd /home/ubuntu/a0509_vla_linux_field_bundle_20260903/lhj
PY=/home/ubuntu/a0509_vla_linux_field_bundle_20260903/environment/a6000_ubuntu22_py310/bin/python
$PY experiments/phase11_sim_to_real_mean_feature_correction/run.py
$PY experiments/phase11_sim_to_real_mean_feature_correction/report.py
$PY -m unittest discover -s experiments/phase11_sim_to_real_mean_feature_correction -p 'test_*.py' -v
```

`run.py`는 새 `results_001`이 이미 있으면 중단한다. 재실행하려면 결과 경로를 코드에서 새로운 이름으로 바꾼다. 기존 실험 결과를 수정하지 않는다. 실행 환경, 재현 한계, 수치는 `results_001/RESULTS_REPORT.md`에 기록한다.
