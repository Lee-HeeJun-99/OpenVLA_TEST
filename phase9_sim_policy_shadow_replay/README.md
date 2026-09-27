# Phase 9 Sim Policy Trajectory To Real Shadow Replay

Purpose: create a Sim-policy reference trajectory for Shadow Mode. The real robot should later replay a reviewed trajectory while the policy output from real images is logged only.

This phase does not prove real-world policy success and does not execute the real robot by itself.

## Inputs

- Fixed cube layout JSON used to display the current Isaac Sim block placement.
- `oftplus_h5_vision` server, checkpoint step 28560.
- Isaac Sim closed-loop validation report.

## Output Flow

1. Convert a manual layout JSON to a VLA validation layout manifest.
2. Run Isaac Sim closed-loop VLA validation on that fixed layout.
3. Export the validation report to dataset-shaped steps.
4. Export a real replay candidate action JSONL for safety review.

## Commands

Use the current layout JSON you used to open Isaac Sim. Example:

```bash
BUNDLE=/home/ubuntu/a0509_vla_linux_field_bundle_20260903
PHASE=$BUNDLE/lhj/phase9_sim_policy_shadow_replay
LAYOUT=$BUNDLE/outputs/sim2real_analysis/trajectory010_from_episode004_layout.json

/usr/bin/python3 \
  $PHASE/scripts/make_vla_rollout_inputs.py \
  --manual-layout-json "$LAYOUT" \
  --output-dir "$PHASE/00_inputs" \
  --episodes 1 \
  --target-color orange \
  --instruction "Pick up the orange cube."
```

Start the OFT server separately with `oftplus_h5_vision`:

```bash
BUNDLE=/home/ubuntu/a0509_vla_linux_field_bundle_20260903
cd "$BUNDLE"
scripts/serve.sh oft 8765 0 127.0.0.1
```

In a second terminal, run the fixed-layout Isaac Sim policy rollout:

```bash
BUNDLE=/home/ubuntu/a0509_vla_linux_field_bundle_20260903
PHASE=$BUNDLE/lhj/phase9_sim_policy_shadow_replay
ISAAC=/home/ubuntu/Downloads/isaac-sim-standalone-5.1.0-linux-x86_64
export A0509_PROJECT_ROOT=$BUNDLE/validation/a0509_project

$BUNDLE/environment/a6000_ubuntu22_py310/bin/python \
  $BUNDLE/isaac_sim/run_oft_validation.py \
  --server-url http://127.0.0.1:8765 \
  --isaac-root "$ISAAC" \
  --scene "$BUNDLE/validation/a0509_project/USD/version_1/a0509_version_1_scaled_85x60.usda" \
  --home-config "$BUNDLE/isaac_sim/config/a0509_home_pose_real_joint6_aligned.json" \
  --variant oftplus_h5_vision \
  --statistics "$BUNDLE/runtime_state/oft_mixed480_step28560_merged/dataset_statistics.json" \
  --label phase9_fixed_layout_sim_policy \
  --output-dir "$PHASE/01_sim_policy_rollout" \
  --seed 20260922 \
  --episodes 1 \
  --actions-per-inference 1 \
  --max-policy-steps 120 \
  --record-hz 5 \
  --render-hz 5 \
  --camera-width 1280 \
  --camera-height 720 \
  --uncapped-simulation \
  --lock-eef-orientation \
  --layout-manifest "$PHASE/00_inputs/vla_layout_manifest.json"
```

After the report is created, replace `REPORT` with the printed JSON path:

```bash
BUNDLE=/home/ubuntu/a0509_vla_linux_field_bundle_20260903
PHASE=$BUNDLE/lhj/phase9_sim_policy_shadow_replay
REPORT=$PHASE/01_sim_policy_rollout/phase9_fixed_layout_sim_policy_k1_seed20260922_1ep.json

/usr/bin/python3 \
  $BUNDLE/sim2real_analysis/10_analysis/export_validation_rollout_trajectory.py \
  --validation-report "$REPORT" \
  --output-root "$PHASE/01_sim_policy_rollout/exported_dataset" \
  --domain sim_policy_rollout \
  --target-color orange

/usr/bin/python3 \
  $PHASE/scripts/export_real_replay_candidate_from_vla_report.py \
  --validation-report "$REPORT" \
  --output-dir "$PHASE/02_real_replay_candidate" \
  --episode-index 0 \
  --drop-rotation
```

## Safety Status

The generated `real_replay_candidate_actions.jsonl` is not an approved robot command. Review workspace limits, TCP frame convention, per-step delta size, gripper timing, and Sim IK failures before any real execution.

## Rotation-Locked Rerun

For the current Shadow Mode reference trajectory work, use the real-joint6-aligned home config and lock the EEF orientation during rollout:

```bash
BUNDLE=/home/ubuntu/a0509_vla_linux_field_bundle_20260903
PHASE=$BUNDLE/lhj/phase9_sim_policy_shadow_replay
ISAAC=/home/ubuntu/Downloads/isaac-sim-standalone-5.1.0-linux-x86_64
export A0509_PROJECT_ROOT=$BUNDLE/validation/a0509_project

$BUNDLE/environment/a6000_ubuntu22_py310/bin/python \
  $BUNDLE/isaac_sim/run_oft_validation.py \
  --server-url http://127.0.0.1:8765 \
  --isaac-root "$ISAAC" \
  --scene "$BUNDLE/validation/a0509_project/USD/version_1/a0509_version_1_scaled_85x60.usda" \
  --home-config "$BUNDLE/isaac_sim/config/a0509_home_pose_real_joint6_aligned.json" \
  --variant oftplus_h5_vision \
  --statistics "$BUNDLE/models/oft_mixed480_step28560/dataset_statistics.json" \
  --label phase9_fixed_layout_sim_policy_k5_rotlock \
  --output-dir "$PHASE/01_sim_policy_rollout" \
  --seed 20260922 \
  --episodes 1 \
  --actions-per-inference 5 \
  --max-policy-steps 500 \
  --stagnation-policy-steps 25 \
  --policy-hold-steps 12 \
  --record-hz 5 \
  --render-hz 5 \
  --camera-width 1280 \
  --camera-height 720 \
  --motion-speed-scale 1.7126963727221802 \
  --uncapped-simulation \
  --lock-eef-orientation \
  --layout-manifest "$PHASE/00_inputs/vla_layout_manifest.json" \
  --record-video \
  --video-fps 5
```
