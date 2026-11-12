# Prediction / target / ServoL / actual trajectory comparison

Source trial: `20261112_190404_arm_only_servo_episode`. Offline analysis only; physical calls **0**. No runtime code, thresholds or original logs changed.

## Verdict

**Not a one-to-one match.** Raw translation was correctly preserved into target generation. Rotation was modified by the existing angular limiter in 76/224 predictions; conversion of the limited rotation into target orientation was correct. Early/middle observed motion generally followed the intended direction but did not complete the full action within the next observation interval. In the terminal interval actual rotation and the virtual command trajectory diverged substantially. The operator attributes the final protective stop to a joint limit.

## Evidence coverage

- Raw predictions and controller Cartesian feedback: 224 each.
- Published stream pose messages: 2143, linked to 223 distinct inference source timestamps.
- The final accepted prediction (step223) has no corresponding stream message. Accepted target count is not actual dispatch count.
- Measured JointState trajectory: 12073 samples, joint-name reordered and converted to URDF FK flange poses.
- Batch FK was independently checked against the existing scalar FK implementation at 5 samples; position agrees within 1e-9mm, orientation within 1e-12rad.
- Source file hashes and URDF hash are preserved. No per-message vendor ACK or execution timestamps are available.

## Target generation

Translation modified: 0/224. Maximum arithmetic target-position reconstruction error 9.65e-14mm.

Rotation modified by angular limiter: 76/224. Target rotation equals `Exp(limited_world_rotvec) * R_current`, maximum reconstruction error 5.55e-14deg. This establishes correct target math, not physical target attainment.

## Controller-feedback intervals

Each interval runs from recorded CurrentPosX for step i to CurrentPosX for step i+1. These windows include retained previous control state and asynchronous publication/transport effects; they are NOT isolated single-command response tests. Ratios below are not controller gains or percentages eventually completed.

| Metric (median) | Steps0..216: 217 intervals | Steps217..222: 6 intervals |
|---|---:|---:|
| Translation direction cosine (1=same) | 0.9833 | 0.2536 |
| Rotation direction cosine (world rotvec) | 0.9977 | 0.3386 |
| Observed translation / raw translation | 0.4011 | 0.2195 |
| Observed rotation / filtered rotation | 0.4162 | 5.9847 |
| Position target error at next feedback (mm) | 1.9768 | 3.3626 |
| Orientation target error at next feedback (deg) | 0.7228 | 5.2352 |

Across all223 intervals, translation opposite-projection intervals=5; rotation opposite-projection intervals=0. Positive projection alone does not mean matching axes or magnitude, especially when direction cosine is small. The first group is called early/middle, not certified fault-free runtime.

## Terminal examples

| Step | Raw rotation deg | Filtered rotation deg | Source-linked stream first-to-last rotation deg | Controller pose change to next feedback deg |
|---:|---:|---:|---:|---:|
|217|0.850|0.850|0.639|0.588|
|219|0.773|0.773|1.761|3.705|
|220|1.068|1.012|3.442|7.264|
|221|0.805|0.805|4.779|9.329|
|222|0.111|0.111|7.027|6.955|

The 0.111deg inference cannot be claimed to have directly requested an isolated 6.955deg move. The stream generator retains pose and velocity, while fresh endpoints are computed from real controller feedback. The observation shows a strong late mismatch, not a proven mechanism or gain of 62x. Vendor interpretation, retained trajectory state, IK, and command retargeting remain causal candidates.

## Dense stream vs measured-joint FK trajectory

At each stream request time, use the most recent earlier JointState sample. Maximum joint-sample age=35.31ms. This is host-time alignment, not vendor consumption alignment. FK is computed flange, not verified controller TCP; no flange-to-TCP correction was introduced.

| Requested pose vs FK flange | Median | p95 | Maximum |
|---|---:|---:|---:|
| Translation difference mm | 2.8315 | 5.8141 | 10.1825 |
| Rotation geodesic difference deg | 0.7851 | 2.2119 | 13.1569 |

This independently supports terminal trajectory mismatch. Absolute FK/controller differences can include URDF calibration and frame/tool uncertainty, so they are not a calibrated servo-following-error measurement.

## Files and reproduction

- `prediction_vs_controller_feedback.csv`: raw/limited action, endpoint, actual displacement, direction and magnitude metrics per feedback interval.
- `measured_joint_FK_trajectory.csv`: full measured-joint computed flange trajectory.
- `stream_vs_joint_FK.csv`: all 2143 requested poses aligned to measured-joint FK.
- `summary.json`: statistics, model/URDF metadata and limitations.
- `input_sha256.json`: source integrity hashes.

Reproduce: `/usr/bin/python3 compare_servo_prediction_trajectory.py`. The output directory must not already exist. No additional robot execution is required.
