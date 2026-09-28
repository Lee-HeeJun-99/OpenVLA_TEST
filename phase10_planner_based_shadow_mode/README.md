# Phase 10 — Planner-based Real Shadow Mode

Phase 10 is a new pipeline. It does not rewrite Phase 8 or treat failed Phase 9 policy trajectories as Planner ground truth.

```text
Observation Distribution
→ Representation Distribution
→ Policy-Relevant Representation
→ Planner-relative Action Error
→ Real Closed-loop Success
```

Current overall status: `PARTIALLY_COMPLETED`. Audit, canonical action conversion, logger, safety gate, dual recorded-input adapters and offline fixture smoke tests are complete. No Real Shadow Mode or Real rollout has run.

## Mode boundary

- `recorded-input shadow`: implemented; same saved images feed both models.
- `online shadow`: not implemented in the clean repository; intended only for latency checks.
- `closed-loop`: not executed and requires a separate explicit approval.

`shadow_mode_runner.py` has no ROS import or publisher. AI executed-action fields are null and `ShadowCommandGate` raises on every attempted AI publish.

## Offline tests

```bash
cd /home/ubuntu/a0509_vla_linux_field_bundle_20260903/lhj
./phase10_planner_based_shadow_mode/scripts/run_offline_smoke.sh
```

The test uses two existing episode 4 frames and explicitly marked fixture model responses. It validates the pipeline only and does not produce research results.

## Recorded-input replay against model servers

The following is prediction-only and cannot command the robot:

```bash
python phase10_planner_based_shadow_mode/06_shadow_collection/shadow_mode_runner.py \
  --episode-dir /path/to/recorded/planner/episode \
  --output-dir /path/to/new/output_dir \
  --trial-id baseline_trial_001 \
  --condition-id baseline \
  --layout-id layout_001 \
  --openvla-server-url http://127.0.0.1:8764 \
  --oft-server-url http://127.0.0.1:8765
```

OFT is inferred every fifth 5 Hz frame and returns K=5. OpenVLA is inferred every frame and returns K=1.

## Timestamp alignment

```bash
python phase10_planner_based_shadow_mode/08_time_alignment/align_recorded_samples.py \
  --samples /path/to/shadow_output/samples.jsonl \
  --output /path/to/aligned_samples.csv
```

## Required before Real collection

1. Freeze/review the external Real controller source revision.
2. Integrate measured TCP/joint/gripper feedback and preserve its source clock.
3. Verify Home, gripper Home, workspace/joint/velocity limits, hold acknowledgement and hardware E-stop.
4. Wire critical logger failure and state/communication timeout to planner/robot hold.
5. Confirm model servers/checkpoints and `Pick up the orange cube.` normalization.
6. Perform a planner-only dry run under operator review.
7. Obtain explicit user approval before any Real command.

Phase 9 files are diagnostic only and must never be passed to Real replay tooling.

