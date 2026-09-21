# Phase7 Priority1 Multi-Metric Reanalysis Report

Status: `VERIFIED_FOR_FULL_FORWARD_PREPROCESSING_CONDITIONS`

This report recomputes Real/Sim policy action disagreement metrics from stored `response['actions']` chunks for P0-P6 preprocessing/photometric conditions. It does not claim GT action accuracy.

## Ranking by `action_l2_chunk_mean`

- rank 1: `P4_letterbox_224` = 0.101993
- rank 2: `P2_center_crop_square_resize_224` = 0.149192
- rank 3: `P3_center_crop_0p875_resize_224` = 0.180517
- rank 4: `P1_resize_224_direct` = 0.369253
- rank 5: `P0_current_paired_image` = 0.370203
- rank 6: `P6_contrast_match_sim_to_real` = 0.371546
- rank 7: `P5_brightness_match_sim_to_real` = 0.376782

## Ranking by `action_l1_chunk_mean`

- rank 1: `P4_letterbox_224` = 0.106149
- rank 2: `P2_center_crop_square_resize_224` = 0.153796
- rank 3: `P3_center_crop_0p875_resize_224` = 0.187200
- rank 4: `P1_resize_224_direct` = 0.380740
- rank 5: `P0_current_paired_image` = 0.381495
- rank 6: `P6_contrast_match_sim_to_real` = 0.382921
- rank 7: `P5_brightness_match_sim_to_real` = 0.390074

## Ranking by `action_rmse_chunk_mean`

- rank 1: `P4_letterbox_224` = 0.038550
- rank 2: `P2_center_crop_square_resize_224` = 0.056389
- rank 3: `P3_center_crop_0p875_resize_224` = 0.068229
- rank 4: `P1_resize_224_direct` = 0.139564
- rank 5: `P0_current_paired_image` = 0.139924
- rank 6: `P6_contrast_match_sim_to_real` = 0.140431
- rank 7: `P5_brightness_match_sim_to_real` = 0.142410

## Ranking by `action_huber0.05_chunk_mean`

- rank 1: `P4_letterbox_224` = 0.004191
- rank 2: `P2_center_crop_square_resize_224` = 0.006645
- rank 3: `P3_center_crop_0p875_resize_224` = 0.008198
- rank 4: `P1_resize_224_direct` = 0.017702
- rank 5: `P0_current_paired_image` = 0.017752
- rank 6: `P6_contrast_match_sim_to_real` = 0.017819
- rank 7: `P5_brightness_match_sim_to_real` = 0.018096

## Ranking by `gripper_abs_chunk_mean`

- rank 1: `P4_letterbox_224` = 0.101700
- rank 2: `P2_center_crop_square_resize_224` = 0.148689
- rank 3: `P3_center_crop_0p875_resize_224` = 0.180058
- rank 4: `P1_resize_224_direct` = 0.368487
- rank 5: `P0_current_paired_image` = 0.369431
- rank 6: `P6_contrast_match_sim_to_real` = 0.370785
- rank 7: `P5_brightness_match_sim_to_real` = 0.376012

Key interpretation: P4 remains best among P0-P6 across the main full-action metrics available from stored action chunks. P5 brightness is not rescued by alternative L1/RMSE/Huber metrics.
