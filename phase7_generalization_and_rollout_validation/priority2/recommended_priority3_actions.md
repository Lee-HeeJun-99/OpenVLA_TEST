# Recommended Priority3 Actions

1. Restore CUDA/NVIDIA driver availability and run full-forward extraction for `02_matched_camera_shift/condition_inputs`.
2. Run full-forward extraction for `03_letterbox_ablation/condition_inputs`.
3. Recompute the same Priority1 metric table for those new full-forward conditions.
4. Collect new condition-held-out object position / lighting / camera data only after the above controlled image-space ablations are complete.
5. Do not start Shadow Mode or closed-loop rollout until a separate safety and ROS/proprio deployment audit is complete.
