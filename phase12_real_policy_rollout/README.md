# Phase 12 Real policy rollout

Current state: `OFFLINE_ROLLOUT_PIPELINE_READY / REAL_MOTION_VALIDATION_PENDING`.

This directory contains canonical/action/safety contracts, a hard command gate,
Null/Mock sinks, TCP provenance abstractions, recorded Shadow and mock closed-loop.
It does not contain an authorized motion launcher or usable Real sink. Run offline
tests only:

```bash
/home/ubuntu/a0509_vla_linux_field_bundle_20260903/environment/a6000_ubuntu22_py310/bin/python -m unittest discover -s /home/ubuntu/a0509_vla_linux_field_bundle_20260903/lhj/phase12_real_policy_rollout/tests -v
```

Recorded prediction-only Shadow:

```bash
PY=/home/ubuntu/a0509_vla_linux_field_bundle_20260903/environment/a6000_ubuntu22_py310/bin/python
P12=/home/ubuntu/a0509_vla_linux_field_bundle_20260903/lhj/phase12_real_policy_rollout
P10=/home/ubuntu/a0509_vla_linux_field_bundle_20260903/lhj/phase10_planner_based_shadow_mode
$PY "$P12/03_shadow_mode/prediction_shadow_runtime.py" --model openvla --command-mode disabled --input "$P10/06_shadow_collection/offline_recorded_episode4/openvla_step8130_retry/samples.jsonl" --output /tmp/phase12_openvla_shadow.jsonl
$PY "$P12/03_shadow_mode/prediction_shadow_runtime.py" --model oft --command-mode disabled --input "$P10/06_shadow_collection/offline_recorded_episode4/oft_vision_step28560_local/samples.jsonl" --output /tmp/phase12_oft_shadow.jsonl
```

GPU model runs must be sequential. Existing Real `runtime.launch.py`, ActionAdapter
and DoosanBridge are not part of this offline path.
