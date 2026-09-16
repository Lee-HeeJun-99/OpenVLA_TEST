# Phase 6 Full-Forward Feasibility Audit

[Status]
UNVERIFIED / FULL-FORWARD REQUIRED

[Checked]
- Existing extractor: `/home/ubuntu/a0509_vla_linux_field_bundle_20260903/sim2real_analysis/04_features/extract_vla_features.py`
- Runtime: `/home/ubuntu/a0509_vla_linux_field_bundle_20260903/runtime/openvla-oft/vla-scripts/serve_a0509_oft.py`
- Policy checkpoint: `/home/ubuntu/a0509_vla_linux_field_bundle_20260903/runtime_state/oft_mixed480_step28560_merged`
- Variant: `oftplus_h5_vision`

[Result]
The current session cannot run full image -> action inference because CUDA is unavailable.

Observed runtime errors:

```text
nvidia-smi: couldn't communicate with the NVIDIA driver
RuntimeError: CUDA GPU is required
CUDA_ERROR_NO_DEVICE
```

[Decision]
Do not infer environment attribution from surrogate upstream mappings.
All Phase 6 policy-impact metrics require local GPU execution of:

```bash
/home/ubuntu/a0509_vla_linux_field_bundle_20260903/lhj/phase6_environment_attribution/scripts/run_full_forward_feature_extraction.sh

/home/ubuntu/a0509_vla_linux_field_bundle_20260903/environment/a6000_ubuntu22_py310/bin/python \
  /home/ubuntu/a0509_vla_linux_field_bundle_20260903/lhj/phase6_environment_attribution/scripts/phase6_summarize_full_forward.py
```

[What Is Ready]
- Condition images for 225 pairs and seven preprocessing/photometric conditions are prepared under:
  `/home/ubuntu/a0509_vla_linux_field_bundle_20260903/lhj/phase6_environment_attribution/01_preprocessing/condition_inputs`
- Observation-side metrics are computed in:
  `/home/ubuntu/a0509_vla_linux_field_bundle_20260903/lhj/phase6_environment_attribution/environment_factor_table.csv`

[Limitation]
Current observation-only numbers cannot be used as evidence that a factor changes VLA action prediction.
