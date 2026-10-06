# Work log

## 2026-10-01 — recollected lighting re-audit

- Re-audited all 20 episode roots after lighting recollection.
- Confirmed 15/15 valid baseline-condition pairs and zero reference-route mismatch.
- Produced 684 phase-aligned observation pairs, including 228 lighting samples.
- Updated prediction-only launcher to include lighting and run OpenVLA/OFT sequentially.
- Five offline safety/integrity tests passed.
- GPU inference and feature extraction were not executed because model servers and GPU driver are unavailable in this environment.
- Robot/ROS/trajectory/gripper/Home operations: 0.

## 2026-10-01 — prediction result validation

- OpenVLA: 20 episodes, 912/912 predictions, zero inference errors, zero fixture records.
- OFT: output files exist but contain zero predictions; all 192 scheduled 1 Hz requests failed with connection refused.
- Preserved the failed OFT logs and prepared `oft_run2` as a separate rerun destination.
- Added pre-run healthcheck and per-episode fail-closed output validation.

## 2026-10-01 — actual model action analysis

- Validated OpenVLA 912/912 frame predictions and OFT 192/192 K=5 inference chunks; errors and fixtures were zero.
- Expanded OFT K=5 to aligned target steps and generated 1,368 paired model rows (684/model).
- Separated condition-induced action shift from Scripted-Reference-Command-relative error.
- Generated component, standardized, gripper semantic and OFT chunk-index summaries.
- Prepared full hidden-feature extraction for 912 images with a 20 GiB free-space guard and sequential model execution.
- Robot/ROS/trajectory/gripper/Home operations remained zero.

## 2026-10-01 — feature and policy-sensitive completion

- Validated 1,824 full feature records (7.6 GiB), with no extraction errors.
- Computed 3,420 paired layer rows across vision/projector and OFT action-hidden features.
- Enforced train Episodes 1–4 and validation Episode 5; oracle results remain separately labeled.
- Completed k=1..128 sweep; train-selected effective rank saturates at seven due to the 7-D action target.
- Train-selected projection did not consistently beat random projection, so no robustness claim is made.
- Completed episode-bootstrap intervals, phase/gripper semantic summaries and rollout-condition hypotheses.
