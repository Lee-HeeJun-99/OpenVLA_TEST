# Gripper scale audit status

Static semantics are verified, but Phase 11 model-output metrics are not available.

- Translation: metres; rotation: radians; gripper: unitless closedness.
- OFT step28560 mask excludes gripper from affine de-normalization and uses bounded sigmoid continuous closedness.
- OpenVLA step8130 statistics mask includes gripper and describes binary closedness.
- Therefore raw concatenated L2 may be numerically gripper-dominated; component metrics, training-stat standardized L2, robust-scale L2, semantic false-close/false-open and contribution ratio must be reported separately.
- `SCALE_DOMINANCE` versus `SEMANTIC_POLICY_ERROR` remains unresolved until actual Phase 11 predictions exist.
