# PHASE 1 Audit Status

Date: 2026-09-15 KST

This table is a first-pass audit. `VERIFIED` here means verified from currently available files/logs/code/offline calculations, not necessarily physically remeasured in the lab.

| Item | Evidence | Status | Impact | Action |
|---|---|---|---|---|
| Real environment history | Existing logs mention table/workspace changes, but no definitive measurement timeline found yet in first pass. | UNVERIFIED | High | Continue searching logs/images/metadata; do not treat old Z offsets as ground truth. |
| 5-episode environment consistency | Contact sheet `real_sim_episode_sample_contact_sheet.jpg`; Real images show same camera viewpoint/table/background/robot-side structure across episodes. | VERIFIED for visual consistency; geometry still UNVERIFIED | High | Use as same visual condition for now, but do not claim measured table height consistency. |
| Real workspace geometry | Layout metadata gives intended block centers; physical measured table/workspace values not verified. | UNVERIFIED | High | Need measurement source or controlled calibration before geometry claims. |
| Sim workspace geometry | Replay logs show applied manual layouts and cube positions at z `0.8605289459228516` m. Exact table/robot geometry from USD not yet extracted. | EXISTING RESULT | High | Extract USD prim transforms/dimensions next. |
| Robot Base | Sim replay logs use robot prim `/World/a0509/base`; Real base reference not physically verified. | Sim VERIFIED from logs; Real UNVERIFIED | High | Audit Real robot/base frame source from runtime/robot state. |
| Tool/TCP/EEF frames | Sim app args use `end_effector_frame='tool0'`; Real metadata has `tcp_pose`/`end_effector_pose`. Equivalence not verified. | UNVERIFIED | High | Compare frame definitions and transforms before interpreting EEF pose gap. |
| Camera stream | Real collection used `/zed/zed_node/rgb/color/rect/image` in earlier logs; Sim replay logs use primary camera `/World/a0509/link_6/tool0/left_Camera`, 1280x720. | EXISTING RESULT | High | Verify Real left/right and preprocessing path from collection/runtime code. |
| Camera intrinsic | Sim replay app records camera config in runtime, but first pass did not extract full intrinsics from USD. Real intrinsics not verified. | UNVERIFIED | High | Extract camera USD attributes and Real camera info if available. |
| Camera extrinsic | Sim camera prim path known; transform not yet extracted. Real extrinsic/mounting not verified. | UNVERIFIED | High | Extract Sim T_base_camera; locate Real calibration/mount reference. |
| Preprocessing | Feature extraction uses `load_rgb` and OFT runtime with `center_crop=True`; runtime config for Real policy has separate preprocessing settings. Exact equivalence not verified. | EXISTING RESULT | Medium/High | Audit `load_rgb`, OFT runtime image processor, and Real runtime preprocessing. |
| Control frequency | Real timestamps show effective ~5 Hz for five episodes. Sim replay configured `joint_replay_rate_hz=5.0`, physics 60 Hz, render 30 Hz. | VERIFIED for configured/source sampling; Sim physical timing partially VERIFIED | Medium | Compare source_step_index/phase/gripper event timing and Sim capture timing. |
| Replay | Logs show joint replay using Real `steps.jsonl`, home-relative, snap-before-capture, attach cube on close. | VERIFIED for implementation path/config | High | Audit replay code semantics: relative joint convention, cube attach behavior, gripper timing. |
| 225-frame pairing | `build_5episode_paired_inputs.py` pairs by Real `step_index` and Sim `source_step_index`; no missing pair in manifest. | VERIFIED for index pairing | High | Physical-state correspondence still needs joint/EEF/gripper/image-event validation. |
| Feature extraction | `extract_vla_features.py` hooks `vision_backbone` and `projector`, saves full tensors, uses instruction `Pick up the orange cube.` | VERIFIED for code path and output existence | Medium | Need tensor shape report and runtime preprocessing audit. |
| Metrics | `compute_token_distribution_metrics.py` computes pooled cosine/L2, token mean metrics, MMD on paired frames. | VERIFIED for implementation | Medium | Metrics valid conditional on pairing/preprocessing validity. |
| Phase labels | Phase labels come from Real `planner_phase` copied into pairing manifest; phase counts consistent across episodes except alignment length. | VERIFIED for label source | Medium | Do not infer causal phase effect yet. |
| Progress correction | Existing implementation computes progress-conditioned pooled Real-to-Sim shift on all paired frames and evaluates on same frames. | VERIFIED for implementation; EXISTING RESULT for numbers | High | Treat latent reduction as in-sample only. |
| Correction leakage | Same data used for shift estimation and evaluation. | VERIFIED leakage for generalization claims | High | Leave-One-Episode-Out correction required before generalization claims. |

## First-Pass Decision

Existing 5-episode baseline is usable as an `EXISTING RESULT` for audit and exploratory analysis, but not yet a final verified baseline. The largest immediate validity risks are:

1. Real physical geometry/history is not verified.
2. Real/Sim camera extrinsic/intrinsic correspondence is not verified.
3. Same-index pairing is verified, but same physical state correspondence needs deeper checks.
4. Progress correction is in-sample and must be rebuilt with episode-held-out evaluation.

## Files Generated In This Audit Pass

- `episode_consistency_summary.json`
- `episode_consistency_summary.csv`
- `real_sim_episode_sample_contact_sheet.jpg`
- `sim_replay_runtime_evidence.json`
- `sim_replay_runtime_evidence.csv`
- `audit_status.md`


## Additional Audit Evidence - 2026-09-15 KST

### Sim USD Authored Geometry Extract

Evidence file: `sim_usd_geometry_extract.json`

Key authored values from `a0509_version_1_scaled_85x60.usda`:

- Scene sha256: `7a63e0428be4a3d9124710bdaa8ef775e279292235266102d4d3b3f0122becc1`
- Table top authored transform:
  - translate: `[0.744757527499, 0.000488163954463, 0.818029]`
  - scale: `[0.6, 1.2, 0.05]`
- Default cube authored z:
  - `0.861529 m`
- Replay-applied cube z from logs/layouts:
  - `0.8605289459228516 m`
- Left camera authored local transform under `/World/a0509/link_6/tool0/left_Camera`:
  - translate: `[-0.060222793661, -0.10842505344, 0.064837748142]`
  - rotateXYZ: `[155.0000187483, 0, 0] deg`
  - focal length: `2.12`
  - aperture: `[5.376, 3.024]`
  - clipping range: `[0.05, 100]`

Status:

- Direct authored USD values: `VERIFIED`
- Full world transforms / physical dimensions after references/composition: `UNVERIFIED`

### Replay Correspondence Metrics

Evidence files:

- `pair_correspondence_summary.json`
- `pair_correspondence_metrics.csv`

Findings:

- Real `joint_position_rad[:6]` and replay `source_joint_position_rad` match exactly:
  - max diff: `0.0 rad`
- Sim target vs actual joint after snap is effectively exact:
  - max abs error across episodes: about `1.16e-7 rad`
- Replay method is confirmed as home-relative joint replay:
  - Sim target = Sim HOME + Real joint delta from first Real step.
- Cube attach occurs once per episode around gripper close:
  - episode 1: source step 28
  - episode 2: source step 29
  - episode 3: source step 26
  - episode 4: source step 27
  - episode 5: source step 25
- Naive Real TCP position vs Sim EEF position L2 is around `0.813 m`.

Interpretation:

- Joint-index correspondence is `VERIFIED`.
- Direct Real TCP pose vs Sim EEF pose comparison is not valid without coordinate/frame audit.
- Same-index pairing is valid as source-step replay correspondence, but not yet verified as identical physical TCP/EEF pose correspondence.

### Preprocessing / Runtime Mismatch Risk

Evidence:

- Offline feature extraction uses bundle OFT runtime with `center_crop=True` in `extract_vla_features.py`.
- Real ROS runtime config `runtime_oft.yaml` has:
  - `use_training_image_preprocess: false`
  - `image_resize_size: 224`
  - `center_crop_enabled: false`
  - `center_crop_scale: 0.9`
- Runtime OFT node passes raw RGB image array into OFT `get_vla_action`; actual OFT processor behavior still needs direct tensor dump.

Status:

- Offline analysis preprocessing path: `VERIFIED at code/config level`
- Real deployment preprocessing equivalence: `UNVERIFIED`
- Potential impact: high for action-facing conclusions.

### Leave-One-Episode-Out Progress Shift

Evidence:

- Script: `/home/ubuntu/a0509_vla_linux_field_bundle_20260903/lhj/scripts/evaluate_loo_real_to_sim_progress_shift.py`
- Output dir: `/home/ubuntu/a0509_vla_linux_field_bundle_20260903/lhj/phase1_audit/loo_real_to_sim_progress_shift`

Result:

- `vision_backbone.output`:
  - cosine mean: `0.141749 -> 0.040102`
  - L2 mean: `85.1580 -> 46.4359`
  - MMD: `0.273608 -> 0.002791`
- `projector.output`:
  - cosine mean: `0.092312 -> 0.018920`
  - L2 mean: `10.2502 -> 4.6653`
  - MMD: `0.339874 -> 0.002052`

Interpretation:

- Episode-held-out progress shift still reduces latent discrepancy substantially.
- This is stronger than the previous in-sample correction result, but still only representation-level evidence.
- Some projector L2 frame-level cases worsen, so correction is not uniformly beneficial per frame.
- No action/policy-performance claim is allowed yet.

### Policy-Relevant Progress Shift / Offline Action Gap

Evidence:

- Script: `/home/ubuntu/a0509_vla_linux_field_bundle_20260903/lhj/scripts/evaluate_policy_relevant_progress_shift.py`
- Output dir:
  - `/home/ubuntu/a0509_vla_linux_field_bundle_20260903/lhj/phase1_action_gap/policy_relevant_progress_shift`
- Summary:
  - `policy_relevant_progress_shift_summary.json`
- Frame metrics:
  - `policy_relevant_progress_shift_frame_metrics.csv`
- Visualizations:
  - `action_gap_by_progress.png`
  - `raw_vs_hidden_shift_action_gap.png`

Verified implementation details:

- `action_head.output` is normalized action output, not final action for the first six action dimensions.
- Final `response.actions` are produced after q01/q99 unnormalization.
- Unnormalization uses action stats from:
  - `/home/ubuntu/a0509_vla_linux_field_bundle_20260903/runtime_state/oft_mixed480_step28560_merged/dataset_statistics.json`
  - key: `a0509_sim_cube_pick`
- Gripper dimension mask is `false`, so gripper is passed through without q01/q99 scaling.
- Re-running saved `action_hidden_states.input` through the saved action head reproduces final manifest actions with max absolute error:
  - mean: `0.001716`
  - max: `0.004071`

Leave-One-Episode-Out action-level result:

- Raw final action chunk mean L2:
  - `0.370262`
- Normalized action output progress shift:
  - `0.100958`
  - relative reduction: `72.73%`
- Action-hidden progress shift, rerun action head, then unnormalize:
  - `0.072325`
  - relative reduction: `80.47%`
- Raw first action L2:
  - `0.316387`
- Action-hidden shifted first action L2:
  - `0.051591`
  - relative reduction: `83.69%`

Interpretation:

- The existing 5-episode paired dataset supports a Level 2 offline claim:
  - progress-conditioned Real→Sim correction reduces final offline Action Gap on held-out episodes.
- This is stronger than representation-only alignment because corrected action-hidden tensors were passed through the action head and final action unnormalization.

Limits:

- This does not prove real-world task success improvement.
- This does not prove generalization to unseen layouts beyond the five existing episodes.
- This is not yet integrated into online runtime inference.

Status:

- OFT action normalization path: `VERIFIED`
- Offline action-gap reduction on existing 5-episode LOO split: `VERIFIED`
- Runtime-integrated correction: `PLANNED`
- Real performance effect: `UNVERIFIED`

### PHASE 2 Gap Validation / No-Collapse Check

Evidence:

- Script:
  - `/home/ubuntu/a0509_vla_linux_field_bundle_20260903/lhj/scripts/verify_hidden_shift_no_collapse.py`
- Output:
  - `/home/ubuntu/a0509_vla_linux_field_bundle_20260903/lhj/phase2_validation/hidden_shift_no_collapse/hidden_shift_no_collapse_summary.json`
- Summary:
  - `/home/ubuntu/a0509_vla_linux_field_bundle_20260903/lhj/phase2_validation/gap_validation_summary.md`

Verified result:

- Full `action_hidden_states.input` paired L2:
  - `277.423 -> 225.517`
- Full `action_hidden_states.input` cosine distance:
  - `0.393221 -> 0.271807`
- Hidden L2 worsened frames:
  - `8 / 225`
- Feature variance mean:
  - Real: `0.346409`
  - Sim: `0.360631`
  - Corrected: `0.415532`
- Pairwise structure correlation:
  - Corrected vs Real: `0.957173`
  - Corrected vs Sim: `0.829088`
  - Real vs Sim: `0.838765`
- Action gap by episode improved in all five held-out episode evaluations.
- Action worsened frames:
  - `35 / 225`

Interpretation:

- No obvious representation collapse was observed after hidden progress shift.
- Correction preserves strong frame-to-frame structure relative to Real.
- Correction improves average offline action gap, but it is not uniformly beneficial per frame.

Status:

- Hidden correction no-collapse check: `VERIFIED`
- Offline policy-relevant gap reduction: `VERIFIED`
- Causal source of gap: `UNVERIFIED`
- Current ROS runtime/deployment equivalence: `INVALID / RE-RUN REQUIRED`
- Real-world performance effect: `UNVERIFIED`

### PHASE 2 Full Offline Validation

Evidence:

- Script:
  - `/home/ubuntu/a0509_vla_linux_field_bundle_20260903/lhj/scripts/comprehensive_offline_validation.py`
- Summary:
  - `/home/ubuntu/a0509_vla_linux_field_bundle_20260903/lhj/phase2_full_validation/comprehensive_offline_validation_summary.json`
- Status table:
  - `/home/ubuntu/a0509_vla_linux_field_bundle_20260903/lhj/phase2_full_validation/comprehensive_offline_validation_status.csv`
- Report:
  - `/home/ubuntu/a0509_vla_linux_field_bundle_20260903/lhj/phase2_full_validation/full_validation_report.md`

Rerun reproducibility:

- Observation/token gap: `PASS`
- LOO representation shift: `PASS`
- Policy-relevant action shift: `PASS`
- Representation-action gap: `PASS`
- Max absolute numeric diff between original and rerun outputs: `0.0`

Data integrity:

- Real feature manifest: `PASS`
- Sim feature manifest: `PASS`
- Paired manifest: `PASS`
- Real records: `225`
- Sim records: `225`
- Paired records: `225`
- Missing features/images: `0`
- Tensor shape errors: `0`
- Action chunk shape errors: `0`

Final PHASE 2 offline decision:

- Existing 5-episode offline bundle validation is `VERIFIED`.
- Offline Level-2 action-gap reduction is `VERIFIED`.
- Current ROS deployment equivalence remains `INVALID / RE-RUN REQUIRED`.
- Real robot performance remains `UNVERIFIED / REQUIRES REAL ROBOT APPROVAL`.
- Causal source of gap remains `UNVERIFIED / REQUIRES CONTROLLED ABLATION`.

### Real / Sim Episode Metadata Consistency

Evidence:

- Real summary:
  - `real_episode_metadata_summary.json`
  - `real_episode_metadata_summary.csv`
- Sim summary:
  - `sim_episode_replay_summary.json`
  - `sim_episode_replay_summary.csv`

Real file-level consistency across 5 episodes:

- instruction: identical
- target color: identical
- camera/camera mount/image topic: identical
- collection mode: identical
- coordinate frame: identical
- workspace x/y/z values: identical
- fixed start joint: identical
- first TCP pose: identical
- first joint position: identical
- move velocity/acceleration: identical

Real caveat:

- `pose_source` is planned/commanded pose.
- `feedback_pose_samples = 0`.
- Therefore this verifies file-level commanded consistency, not measured physical tracking accuracy.

Sim replay consistency across 5 reports:

- status: `complete`
- home-relative replay: `true`
- snap-before-capture: `true`
- attach cube on close: `true`
- attach distance: `0.3 m`
- replay rate: `5 Hz`
- physics rate: `60 Hz`
- hold steps per source step: `12`
- HOME convergence: `true`

Status:

- Real commanded condition consistency: `VERIFIED`
- Sim replay condition consistency: `VERIFIED`
- Real measured physical trajectory accuracy: `UNVERIFIED`

### Camera Audit

Evidence:

- `camera_audit_summary.md`
- Real 5-episode metadata
- `sim_usd_geometry_extract.json`
- ZED wrapper config search

Findings:

- Real image stream/topic:
  - `/zed/zed_node/rgb/color/rect/image`
  - status: `VERIFIED`
- Real camera model/mount from metadata:
  - `Stereolabs ZED`
  - `eye_in_hand_vertical`
  - status: `VERIFIED at metadata level`
- Real dataset-time camera intrinsics/distortion:
  - not found
  - status: `UNVERIFIED`
- `real_camera.yaml` under Doosan visual servoing exists, but no evidence links it to this dataset.
- Sim authored camera:
  - `/World/a0509/link_6/tool0/left_Camera`
  - local translate/rotate/focal/aperture extracted
  - status: `VERIFIED for authored USD values`
- Real/Sim left/right equivalence:
  - status: `UNVERIFIED`

Impact:

- Camera is still a high-impact unresolved factor for Observation Gap attribution.
- Existing paired representation/action metrics are still valid as offline measurements, but not as camera-cause evidence.

### Coordinate Frame / TCP / EEF Audit

Evidence:

- `coordinate_frame_audit_summary.md`
- Real collection code:
  - `single_robot_simple.py`
- Sim replay code:
  - `isaac_sim/a0509_control_app.py`
  - `isaac_sim/a0509_control/lula_planner.py`

Findings:

- Real `tcp_pose` in these 5 episodes is planned/commanded pose.
- Real xyz is converted `mm -> m`.
- Real orientation fields are converted `deg -> rad` from Doosan pose fields and then interpreted by the dataset code as RPY-like values for quaternion conversion.
- Real `feedback_pose_samples = 0`.
- Sim `sim_end_effector_pose` is computed for `tool0` by Lula/Isaac kinematics.

Decision:

- Direct Real TCP pose vs Sim `tool0` pose numeric comparison is not valid as physical mismatch evidence.
- The valid current replay guarantee is joint-delta replay relative to each domain's HOME configuration.

Status:

- Real commanded pose source: `VERIFIED`
- Sim `tool0` EEF source: `VERIFIED`
- Real measured TCP tracking: `UNVERIFIED`
- Real TCP frame == Sim tool0 frame: `UNVERIFIED`
- Direct Real TCP vs Sim EEF comparison: `INVALID / RE-RUN REQUIRED`

### Preprocessing Audit

Evidence:

- `preprocessing_audit_summary.md`
- Bundle OFT preprocessing code:
  - `runtime/openvla-oft/experiments/robot/openvla_utils.py`
- Current ROS runtime code/config:
  - `/home/ubuntu/robot_ws/src/openvla_doosan_runtime/config/runtime_oft.yaml`
  - `camera_adapter_node.py`
  - `openvla_oft_inference_node.py`

Bundle offline path:

- variant: `oftplus_h5_vision`
- checkpoint: bundle `oft_mixed480_step28560_merged`
- JPEG encode/decode
- Lanczos3 resize to `224 x 224`
- center crop enabled, scale `0.9`
- fused Prismatic image processor

Current ROS OFT config:

- variant: `oftplus_h5_proprio`
- different checkpoint path
- camera adapter does no resize/crop
- `use_training_image_preprocess=false`
- `center_crop_enabled=false`

Status:

- Bundle offline preprocessing: `VERIFIED`
- Current ROS preprocessing config: `VERIFIED`
- Bundle offline vs current ROS equivalence: `INVALID / RE-RUN REQUIRED`

Impact:

- High. Existing LHJ analysis is valid for the bundle offline policy, but not automatically valid for current ROS real deployment.
