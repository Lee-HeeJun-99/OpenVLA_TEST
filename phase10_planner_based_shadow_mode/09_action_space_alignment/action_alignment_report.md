# Action Space Alignment Report

Status: `COMPLETED_OFFLINE_ONLY`

Canonical schema:

```text
[dx_m, dy_m, dz_m, dRotVecX_rad, dRotVecY_rad, dRotVecZ_rad, gripper_closedness]
reference_frame = robot_base_world_aligned
step horizon = 0.2 s
```

The current planner dataset and model-server response use the same component order, but provenance remains distinct. The canonicalizer accepts explicit translation/rotation units, rejects non-rotation-vector representations without a conversion, converts gripper openness when declared, rejects out-of-range/nonfinite values, and records the horizon.

Comparison policy:

- OpenVLA K=1 at frame `t` is compared with Planner step `t`.
- OFT inference at `t` yields K=5 and is compared with Planner steps `t..t+4` per chunk index.
- Endpoint/integrated displacement is also compared over 1.0 s.
- Unequal horizons are never compared by direct raw-vector L2.
- Gripper continuous error and thresholded event timing/precision/recall/F1 are separate outputs.

Remaining validation: confirm the Real planner delta is computed from delivered/measured states rather than only planned poses, and verify external gripper post-processing on the reviewed runtime commit.

