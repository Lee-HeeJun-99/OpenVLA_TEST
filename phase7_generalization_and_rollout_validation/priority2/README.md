# Phase7 Priority2

Status:

```text
COMPLETED_PRIORITY2
PRIORITY3_STARTED: false
PRIORITY4_STARTED: false
```

## Scope

Priority2 used Priority1 outputs as the factual baseline and performed:

- correction action chunk availability audit
- action-head-only hidden alignment baselines
- matched camera shift input preparation
- letterbox decomposition input preparation
- deployability labeling
- final evidence / metric / limitation reports

No new environment data, Shadow Mode, closed-loop rollout, or Real robot motion was started.

## Reproduce

Use the bundle Python environment:

```bash
cd /home/ubuntu/a0509_vla_linux_field_bundle_20260903
/home/ubuntu/a0509_vla_linux_field_bundle_20260903/environment/a6000_ubuntu22_py310/bin/python \
  lhj/phase7_generalization_and_rollout_validation/scripts/priority2_alignment_and_ablation.py \
  --bundle-root /home/ubuntu/a0509_vla_linux_field_bundle_20260903 \
  --output-dir /home/ubuntu/a0509_vla_linux_field_bundle_20260903/lhj/phase7_generalization_and_rollout_validation/priority2 \
  --seed 20260918
```

## Key Outputs

- `final_phase7_priority2_report.md`
- `final_status.json`
- `final_evidence_matrix.csv`
- `final_metric_table.csv`
- `limitations.md`
- `validation_report.md`
- `recommended_priority3_actions.md`
- `01_correction_multimetric_audit/`
- `02_matched_camera_shift/`
- `03_letterbox_ablation/`
- `04_global_vs_policy_alignment/`

## GPU Required Items

The current environment could not communicate with the NVIDIA driver. The following inputs are prepared but require full-forward extraction on a CUDA-capable setup:

- `02_matched_camera_shift/condition_inputs/`
- `03_letterbox_ablation/condition_inputs/`

Use `oftplus_h5_vision`, checkpoint step 28560, instruction `Pick up the orange cube.`, and the same extraction path used in Phase6.

## Interpretation Limits

- `04_global_vs_policy_alignment/` is action-head-only on stored `action_hidden_states.input`.
- It is not image-to-final-action full-forward.
- Matched camera and letterbox action numbers were not generated in Priority2.
- Real robot performance remains unverified.
