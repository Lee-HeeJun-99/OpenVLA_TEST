# Phase 6 Local GPU Run Commands

Run from any shell on the machine/session where CUDA is visible to PyTorch.

## 1. Confirm GPU

```bash
nvidia-smi
```

## 2. Prepare Condition Images

Already prepared once, but safe to regenerate:

```bash
/home/ubuntu/a0509_vla_linux_field_bundle_20260903/environment/a6000_ubuntu22_py310/bin/python \
  /home/ubuntu/a0509_vla_linux_field_bundle_20260903/lhj/phase6_environment_attribution/scripts/phase6_prepare_condition_images.py
```

## 3. Run Full-Forward Feature/Action Extraction

This runs `oftplus_h5_vision`, checkpoint step `28560`, over all condition/domain images.

```bash
/home/ubuntu/a0509_vla_linux_field_bundle_20260903/lhj/phase6_environment_attribution/scripts/run_full_forward_feature_extraction.sh
```

Outputs:

```text
/home/ubuntu/a0509_vla_linux_field_bundle_20260903/lhj/phase6_environment_attribution/01_preprocessing/full_forward_features
```

## 4. Summarize Phase 6 Full-Forward Metrics

```bash
/home/ubuntu/a0509_vla_linux_field_bundle_20260903/environment/a6000_ubuntu22_py310/bin/python \
  /home/ubuntu/a0509_vla_linux_field_bundle_20260903/lhj/phase6_environment_attribution/scripts/phase6_summarize_full_forward.py
```

Key outputs:

```text
/home/ubuntu/a0509_vla_linux_field_bundle_20260903/lhj/phase6_environment_attribution/environment_factor_table.csv
/home/ubuntu/a0509_vla_linux_field_bundle_20260903/lhj/phase6_environment_attribution/policy_sensitive_attribution_frame_metrics.csv
/home/ubuntu/a0509_vla_linux_field_bundle_20260903/lhj/phase6_environment_attribution/full_forward_summary.json
```

## 5. Current Interpretation Rule

Until step 3 and 4 complete, Phase 6 is observation-side only.

Do not interpret preprocessing/photometric improvements as policy improvements before `action_gap` is filled in `environment_factor_table.csv`.
