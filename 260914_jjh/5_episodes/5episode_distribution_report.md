# 5-Episode Real/Sim Token Distribution Analysis

Date: 2026-09-15 KST

## Input Pairing

- Analysis root: `/home/ubuntu/a0509_vla_linux_field_bundle_20260903/outputs/token_distribution_analysis/5_episodes`
- Real source: `/home/ubuntu/a0509_vla_linux_field_bundle_20260903/data/real_world/raw_dataset_oft/episodes`
- Sim source: `/home/ubuntu/a0509_vla_linux_field_bundle_20260903/outputs/sim2real_analysis`
- Pair-preserving input folder: `paired_inputs`
- Total paired frames: `225`

Episode frame pairs:

- `episode_000001`: `46`
- `episode_000002`: `47`
- `episode_000003`: `44`
- `episode_000004`: `45`
- `episode_000005`: `43`

Real/Sim frame pairing was preserved. No independent deduplication was applied, because this analysis assumes same-timestep Real/Sim image comparison.

## Encoder

- Encoder path: existing bundle OFT/OpenVLA feature extractor
- Script: `sim2real_analysis/04_features/extract_vla_features.py`
- Feature outputs:
  - `features/real/feature_manifest.json`
  - `features/sim/feature_manifest.json`
- Saved feature levels:
  - `vision_backbone.output`
  - `projector.output`
- Instruction: `Pick up the orange cube.`

## Distribution Metrics

Output:

- `metrics/summary.json`
- `metrics/frame_pair_metrics.csv`

Overall results:

- `vision_backbone.output`
  - frame cosine mean: `0.141749`
  - frame L2 mean: `85.1580`
  - distribution mean-shift L2: `63.2623`
  - MMD: `0.273608`
- `projector.output`
  - frame cosine mean: `0.092312`
  - frame L2 mean: `10.2502`
  - distribution mean-shift L2: `8.36836`
  - MMD: `0.339874`

## Episode-Level Trend

Projector pooled cosine mean:

- `episode_000001`: `0.071829`
- `episode_000002`: `0.079840`
- `episode_000003`: `0.096804`
- `episode_000004`: `0.109822`
- `episode_000005`: `0.104934`

Interpretation:

- Episode 1 and 2 are closer between Real and Sim.
- Episode 4 and 5 show larger Real/Sim gap.
- The gap likely changes with object placement, robot/table visibility, gripper-object relation, and camera-frame composition.

## Phase-Level Trend

Projector pooled cosine mean:

- `hold`: `0.032876`
- `alignment`: `0.056368`
- `lift`: `0.110268`
- `descent_to_grasp`: `0.113158`
- `grasp_close`: `0.134261`

Interpretation:

- The largest gap appears around contact/gripper phases.
- This matches the previous single-trajectory observation that gripper motion/contact timing is a sensitive source of latent distribution shift.
- `hold` is closest because scene dynamics are minimal.

## Real-to-Sim Progress Shift

Output:

- `real_to_sim_progress_shift/real_to_sim_progress_shift_summary.json`
- `real_to_sim_progress_shift/frame_pair_real_to_sim_before_after.csv`
- `real_to_sim_progress_shift/real_to_sim_progress_shift_vectors.npz`

Direction:

- `real_corrected = real + (sim_local_mean - real_local_mean)`

Overall before/after:

- `vision_backbone.output`
  - frame cosine mean: `0.141749 -> 0.031199`
  - frame L2 mean: `85.1580 -> 40.7191`
  - MMD: `0.273608 -> 0.001732`
- `projector.output`
  - frame cosine mean: `0.092312 -> 0.014645`
  - frame L2 mean: `10.2502 -> 4.0928`
  - MMD: `0.339874 -> 0.001493`

Interpretation:

- Progress-conditioned Real-to-Sim shift strongly moves Real latent features toward the Sim distribution.
- This is currently an offline latent-space sanity check.
- Policy deployment still requires deciding where to inject the correction: full projector tokens, policy input embedding, or a later action-facing hidden state.

## Generated Visualizations

- `plots/frame_pair_distance_curves.png`
- `plots/pca_pooled_vision_backbone_output.png`
- `plots/pca_pooled_projector_output.png`
- `plots/top_projector_gap_frames_contact_sheet.png`
- `real_to_sim_progress_shift/frame_pair_real_to_sim_before_after.png`
- `real_to_sim_progress_shift/real_to_sim_shift_norm_by_frame.png`
- `real_to_sim_progress_shift/pca_real_to_sim_progress_shift_vision_backbone_output.png`
- `real_to_sim_progress_shift/pca_real_to_sim_progress_shift_projector_output.png`
- `episode_phase_summary/episode_gap_bar.png`
- `episode_phase_summary/phase_gap_bar.png`
- `episode_phase_summary/episode_phase_projector_gap_heatmap.png`

## Next Step

The next practical step is to select one policy-facing correction point and run an offline replay-style policy inference test:

1. Load Real images.
2. Encode them with the same OFT/OpenVLA encoder.
3. Apply Real-to-Sim progress shift to the selected latent representation.
4. Feed corrected latent features to the Sim-trained policy path.
5. Compare predicted actions against uncorrected Real features and Sim replay references.
