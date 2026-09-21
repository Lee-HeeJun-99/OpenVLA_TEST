# Priority2 Full-Forward Postprocess Report

Status: `VERIFIED_FULL_FORWARD_POSTPROCESS`

- Camera frame rows: 900
- Letterbox frame rows: 2025
- Total frame rows: 2925

## Best Conditions By Action L2

- letterbox_ablation / A2_letterbox_black: 0.102333 (N=225)
- matched_camera_shift / M2_p4_shift_right_4px_matched: 0.102863 (N=225)
- letterbox_ablation / A7_top_aligned_letterbox: 0.120915 (N=225)
- letterbox_ablation / A3_letterbox_gray: 0.130814 (N=225)
- matched_camera_shift / M3_p4_shift_right_24px_control: 0.130915 (N=225)
- letterbox_ablation / A4_letterbox_mean_color: 0.132111 (N=225)
- letterbox_ablation / A6_center_crop_resize: 0.150024 (N=225)
- letterbox_ablation / A8_bottom_aligned_letterbox: 0.183629 (N=225)
- letterbox_ablation / A5_letterbox_replicated_resize_bg: 0.297483 (N=225)
- matched_camera_shift / M1_raw_shift_right_24px: 0.335711 (N=225)

These numbers are image-to-final-action full-forward results from the saved feature manifests.
They remain offline action-disagreement evidence, not real robot performance evidence.
