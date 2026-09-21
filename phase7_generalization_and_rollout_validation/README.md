# Phase7 Generalization and Rollout Validation - Priority1

## Scope

Priority1 only:

- Phase1-Phase6 audit
- Action definition and normalization audit
- Multi-metric reanalysis from stored policy decoded action chunks
- Translation / rotation / gripper separated metrics
- Episode-level statistics
- Gripper disagreement analysis
- Protocol stubs for future data/GPU/robot phases

No GPU full-forward, new data collection, or robot motion was executed by this Priority1 script.

## Reproduce

```bash
cd /home/ubuntu/a0509_vla_linux_field_bundle_20260903
python lhj/phase7_generalization_and_rollout_validation/scripts/priority1_audit_and_metrics.py \
  --bundle-root /home/ubuntu/a0509_vla_linux_field_bundle_20260903 \
  --output-dir /home/ubuntu/a0509_vla_linux_field_bundle_20260903/lhj/phase7_generalization_and_rollout_validation \
  --seed 20260918
```

## Key outputs

- `priority1_report.md`
- `priority1_status.json`
- `00_audit/audit_report.md`
- `01_multimetric_reanalysis/frame_metrics.csv`
- `02_statistical_validation/report.md`
- `03_gripper_analysis/report.md`
- `unresolved_issues.md`
- `recommended_priority2_actions.md`

## Known limits

- Metrics are Real/Sim policy prediction disagreement, not GT action error.
- Gripper binary threshold is assumed as 0.5 for disagreement analysis.
- Existing 225-pair dataset has no measured Real robot motion.
- Current ROS proprio deployment is out of scope.
