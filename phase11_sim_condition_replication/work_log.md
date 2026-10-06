# Work log

- Dataset scope: JSON and episode artifacts only. No feature tensors or model outputs.
- Baseline episode 1–5 replay JSON and image directories were copied into `dataset/episodes/baseline`; source data remain unchanged.

## 2026-10-01 policy-sensitive analysis

- Audited 20 Real condition episode roots read-only.
- Retained 10/15 pairs: five extra-object and five distractor-swap; 456 phase-progress-aligned samples.
- Excluded all five lighting-low pairs because grasp reference Z differs from the current baseline by exactly 10 mm. Reconstructed only a candidate lighting manifest because no source manifest exists.
- Computed paired observation L1/RMSE/SSIM and generated three inspected figures.
- OpenVLA/OFT inference and feature hooks were not run: both servers refused connections and the NVIDIA driver was unavailable.
- Added prediction-only launcher and five offline tests; all tests passed.
- Robot/ROS/control operations: zero.
