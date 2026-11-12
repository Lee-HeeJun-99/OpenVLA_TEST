# Three recorded ServoL runs — offline audit

Physical calls: 0. Original artifacts unchanged. No production code changes in this task.

Raw model rotation is a BASE rotvec (rad), NOT Euler ABC. Raw target below is reconstructed from logged current pose and raw action, using intrinsic ZYZ. Recorded filtered targets are separately preserved in targets.csv.

Pair rule requested by user: geodesic < 4deg => expression label; >= 4deg => physical jump label. A small pair angle alone does not prove that the actual rejection was caused by an Euler representation problem.

## Raw-policy target (reconstructed, before rotation delta limiter)

| Run | Step (zero-based) | Rejected | xyz mm | ABC deg | Delta ABC vs previous raw target deg | Pair geodesic deg | Distance of B to nearest singular deg |
|---|---:|---|---|---|---|---:|---:|
| 182408 | 59 | False | [394.525944, -15.728662, 673.441735] | [-47.579061, 152.578605, 115.430866] | [-2.106196, 0.201846, -2.894836] | 1.429153 | 27.421395 |
| 182408 | 60 | False | [396.189014, -15.748590, 672.417138] | [-46.743120, 151.683822, 116.236531] | [0.835941, -0.894782, 0.805665] | 0.978638 | 28.316178 |
| 182408 | 61 | False | [398.420886, -15.741052, 670.610192] | [-49.097184, 152.017994, 113.046739] | [-2.354064, 0.334172, -3.189792] | 1.608107 | 27.982006 |
| 182408 | 62 | True | [399.301319, -14.034006, 669.610681] | [-46.486969, 151.860201, 112.466947] | [2.610215, -0.157793, -0.579792] | 3.137710 | 28.139799 |
| 183554 | 63 | False | [412.937087, 8.460192, 686.253058] | [1.177176, 165.055985, 152.166576] | [-0.477860, 0.387228, -1.951725] | 1.544968 | 14.944015 |
| 183554 | 64 | False | [418.011824, 9.912142, 684.728339] | [0.402526, 165.467524, 150.056825] | [-0.774650, 0.411538, -2.109751] | 1.435051 | 14.532476 |
| 183554 | 65 | False | [421.731750, 9.619713, 683.174018] | [-0.524362, 165.902995, 147.566679] | [-0.926888, 0.435471, -2.490145] | 1.666337 | 14.097005 |
| 183554 | 66 | True | [426.457854, 12.173058, 681.872931] | [-1.071588, 166.288787, 145.359662] | [-0.547225, 0.385792, -2.207017] | 1.724672 | 13.711213 |
| 183848 | 58 | False | [417.042783, 14.564188, 687.908693] | [10.292219, 165.707325, 163.677542] | [-3.341030, 0.734866, -5.049767] | 2.134747 | 14.292675 |
| 183848 | 59 | False | [422.256833, 17.004061, 686.221076] | [11.419724, 166.069164, 163.709002] | [1.127505, 0.361839, 0.031459] | 1.155154 | 13.930836 |
| 183848 | 60 | False | [427.234312, 19.256648, 684.970832] | [16.095006, 166.223345, 167.426098] | [4.675281, 0.154181, 3.717096] | 1.397317 | 13.776655 |
| 183848 | 61 | True | [432.550657, 21.676267, 683.284247] | [11.083381, 166.965099, 160.609372] | [-5.011625, 0.741755, -6.816726] | 2.380597 | 13.034901 |

No listed B is within 1deg of 0 or +/-180deg; nearest distance is 13.035deg. This 1deg neighborhood is a reporting label, not a safety threshold.

## Actual raw gate replay

| Run | Rejected step | Raw geodesic deg | Before | After |
|---|---:|---:|---|---|
| 182408 | [62] | 4.149291218 | REJECT | REJECT |
| 183554 | [66] | 4.113097423 | REJECT | REJECT |
| 183848 | [61] | 4.119130698 | REJECT | REJECT |

All three are actual raw rotation steps above the unchanged 4deg gate. Slerp/continuous ZYZ cannot and must not erase this raw rejection. Replaying recorded inputs is not a counterfactual physical trajectory prediction.

All models report chunk_size=1; each row is a fresh inference, chunk_id=null, chunk_index=0. There is no multi-action chunk boundary.

## Stop samples

| Run | Stop request monotonic | State receive monotonic | State | First exact-zero window ready | State age at that time |
|---|---:|---|---|---|---|
| 182408 | 76322.980988310 | None | None | None | None |
| 183554 | 77014.466465018 | 77014.477830281 | {'robot_state': 1, 'success': True} | 77015.717964475 | 1.2401341939985286 |
| 183848 | 77186.804425285 | 77186.814919599 | {'robot_state': 1, 'success': True} | 77187.767448833 | 0.9525292339967564 |

182408: no post-stop robot state recorded, only one post-stop JointState sample. Stop cannot be verified from these logs.

183554 / 183848: post-stop STANDBY sample exists and follows request. Exact-zero window became available later, when the cached robot-state sample was 1.240s / 0.953s old (>0.5s). Thus stale, not before-request. Same cached state was reused during proof wait. No clock-domain mixing was found: comparisons use host monotonic receive time; ROS source stamps are used only for source continuity.

## Robot-state timeline (compressed)

Complete sampled sequence: robot_state_timeline.csv. State getter receive timestamps were not logged per prediction; inference-start times below are upper bounds on state receipt, not exact measurements. Pose receive times must not be relabeled as state receipt.

| Run | Step interval | State | Receipt upper-bound interval (monotonic) |
|---|---|---:|---|
| 182408 | 0..62 | 1 | 76292.741662781 .. 76322.523091474 |
| 183554 | 0..66 | 1 | 76986.470195551 .. 77014.088568377 |
| 183848 | 0..46 | 1 | 77160.692171286 .. 77180.109469132 |
| 183848 | 47..47 | 2 | 77180.564869551 .. 77180.564869551 |
| 183848 | 48..61 | 1 | 77181.030090666 .. 77186.436143163 |

State 1=STANDBY, 2=MOVING under the launcher contract. Sparse per-inference polling cannot establish the full continuous state trajectory.

## Changes and reproducibility

`analyze_three_servo_runs.py` added only. New analysis CSV/JSON/Markdown generated. No runtime/safety/controller changes in this task because all rejected raw rotations are real >4deg steps. Existing geodesic/Slerp/continuous-ZYZ code was reused read-only.

Reconstruction replay: every stored target segment sampled at 21 fractions using linear position/Slerp rotation; max orientation reconstruction error <1.6e-15 rad across all runs.

Source file hashes: provenance.json. Raw values, separate filtered target values, and both pair-angle calculations: targets.csv. Per-prediction raw-gate replay decisions: analysis.json.

Command: `/usr/bin/python3 analyze_three_servo_runs.py` (output directory must not already exist).
