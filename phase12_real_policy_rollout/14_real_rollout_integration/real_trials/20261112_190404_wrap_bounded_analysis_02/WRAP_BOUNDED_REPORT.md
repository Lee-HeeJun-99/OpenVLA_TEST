# 190404 C-wrap, signed lead, bounded interpolation and test draft

**Physical calls0. Original trial and all live limits/configurations unchanged.**

## 1. Timing and interpretation

First emitted C outside[-180,180]: monotonic78799.595141412, C=-180.059768463deg (source step217).

The terminal uninterrupted increase begins at sample218. Its onset lies between sample217 at78799.423470039 (0.462328deg error) and sample218 at78799.900505753 (0.902106deg error). The C crossing falls INSIDE this interval. With pose sampled only once per inference and unknown acquisition/vendor-consumption delay, the logs cannot determine which physically began first.

A weaker earlier three-positive-growth sequence exists at205..207, before crossing. This rules out claiming that all error growth first started only at the C boundary. Late error first exceeds normal p99 at step220/time78800.901082415,1.305941sec after crossing. Correlation is strong in the terminal window, causality remains unproven.

Actual CurrentPosX is compared to the last requested command at or before its receive time; no future command is used. Initial step0 precedes the first stream request and is compared to the initial pose. All angles use rotation matrices/quaternions.

### Step200..222 errors and growth rates

| Step | Pose receive monotonic | Position error mm | Growth mm/s | Rotation error deg | Growth deg/s |
|---:|---:|---:|---:|---:|---:|
| 200 | 78790.749009 | 2.015633 | -0.378480 | 0.420751 | 0.260335 |
| 201 | 78791.259847 | 1.998650 | -0.033246 | 0.690771 | 0.528582 |
| 202 | 78792.034513 | 0.955916 | -1.346044 | 0.277174 | -0.533903 |
| 203 | 78792.500613 | 2.016904 | 2.276310 | 0.590759 | 0.672784 |
| 204 | 78793.031681 | 1.674730 | -0.644314 | 0.294354 | -0.558131 |
| 205 | 78793.498068 | 1.900779 | 0.484682 | 0.365660 | 0.152891 |
| 206 | 78793.959022 | 2.584503 | 1.483281 | 0.388099 | 0.048679 |
| 207 | 78794.464065 | 0.281144 | -4.560720 | 0.885124 | 0.984124 |
| 208 | 78794.981697 | 1.379693 | 2.122260 | 0.569416 | -0.609907 |
| 209 | 78795.451259 | 2.462371 | 2.305714 | 1.091151 | 1.111108 |
| 210 | 78795.942148 | 0.688926 | -3.612721 | 0.786100 | -0.621427 |
| 211 | 78796.406850 | 1.603112 | 1.967254 | 0.765296 | -0.044767 |
| 212 | 78796.974897 | 0.619901 | -1.730863 | 0.421021 | -0.606069 |
| 213 | 78797.500977 | 2.138900 | 2.887387 | 1.099162 | 1.289044 |
| 214 | 78797.944162 | 1.950228 | -0.425717 | 0.566024 | -1.202972 |
| 215 | 78798.441188 | 1.914820 | -0.071239 | 0.444525 | -0.244452 |
| 216 | 78798.908990 | 2.191255 | 0.590922 | 0.838214 | 0.841572 |
| 217 | 78799.423470 | 2.295146 | 0.201933 | 0.462328 | -0.730612 |
| 218 | 78799.900506 | 2.843663 | 1.149845 | 0.902106 | 0.921896 |
| 219 | 78800.451981 | 3.130222 | 0.519623 | 1.695240 | 1.438206 |
| 220 | 78800.901082 | 3.956403 | 1.839631 | 3.621001 | 4.288028 |
| 221 | 78801.403602 | 3.731067 | -0.448413 | 7.304051 | 7.329171 |
| 222 | 78801.916303 | 3.486065 | -0.477864 | 11.568308 | 8.317241 |

### Signed ahead/behind projections

World translation error e=p_actual-p_requested; axis lead_i=e_i*sign(v_requested_i). Positive is ahead along that axis current requested direction; negative is behind. Zero requested axis speed is undefined, not forced to ahead/behind.

World rotation error is Log(R_actual*R_requested^-1). Its dot product with the unit requested world angular-velocity vector is signed rotational lead. Positive means geometrically ahead along that instantaneous direction, NOT vendor-confirmed tracking/latency or a proven cause.

| Step | Axis signed lead X/Y/Z mm | Translation-direction lead mm | Rotation-direction lead deg |
|---:|---|---:|---:|
| 200 | [-1.135407, -0.271627, -1.643121] | -1.588575 | 0.410054 |
| 201 | [-1.003572, -0.123248, -1.724023] | -1.955525 | 0.635491 |
| 202 | [0.488402, -0.020936, 0.821462] | 0.926797 | -0.257005 |
| 203 | [-1.066300, 0.031905, -1.711692] | -2.015804 | 0.588470 |
| 204 | [0.867523, -0.002969, 1.432520] | 1.674611 | -0.113933 |
| 205 | [-1.001994, 0.000156, -1.615231] | -1.900775 | 0.284475 |
| 206 | [-2.150147, -0.109040, 1.429907] | -1.860388 | 0.345353 |
| 207 | [0.165179, -0.149843, -0.171187] | 0.078037 | 0.390779 |
| 208 | [-0.428796, -0.150044, -1.302756] | -1.333904 | 0.393186 |
| 209 | [-1.300117, -0.336560, -2.063902] | -2.446474 | -0.600723 |
| 210 | [-0.527758, 0.303742, -0.322227] | -0.105571 | 0.692840 |
| 211 | [-1.029639, -0.057011, -1.227421] | -1.601555 | 0.719421 |
| 212 | [-0.589548, 0.066614, -0.179648] | -0.314934 | 0.356420 |
| 213 | [-0.858726, -0.042441, -1.958489] | -2.133824 | -1.044751 |
| 214 | [0.849076, 0.097991, 1.752957] | 1.939211 | -0.174140 |
| 215 | [-0.817320, -0.017437, -1.731537] | -1.914310 | 0.355265 |
| 216 | [0.984367, 0.028430, 1.957502] | 2.184401 | 0.825460 |
| 217 | [-1.064301, 0.111149, -2.030420] | -2.281914 | -0.448116 |
| 218 | [1.202304, 0.273637, 2.562422] | 2.809981 | 0.240435 |
| 219 | [1.274726, 0.239995, 2.848818] | 3.084348 | 0.926471 |
| 220 | [-2.789616, -0.642860, -2.730916] | -3.342556 | 3.438419 |
| 221 | [3.124902, 0.641240, 1.935112] | 3.371988 | 6.682540 |
| 222 | [3.278005, 0.069788, -1.184257] | 1.772460 | 11.498542 |

Terminal rotation projection is positive at218..222 and reaches+11.498508deg. Translation is not uniformly ahead: at220 all three axis projections are negative; at222 X/Y are ahead but Z is behind. Therefore a blanket statement that every component of the robot runs ahead is incorrect.

## 2. New offline generator

`servo_experimental/bounded_segment.py::CommandSegmentInterpolator`:

- Command reference: segment start is previous generated/published command; target is that same pose plus the recorded limited action.
- Position: p(u)=p_start+u*(p_target-p_start). Orientation: shortest-arc Slerp(R_start,R_target,u). No velocity state or speed integration.
- Time fraction u=clip((now-activation)/duration,0,1). Duration uses the last observed target activation interval (causal), first defaults to frozen1/control_rate_hz. Future inference duration is not used. A late target holds the endpoint; an early target preempts at current command without a position/orientation jump.
- ABC is normalized to[-180,180], evaluating signed-B equivalent forms and endpoint wraps and selecting the nearest numeric representation to the previous normalized command. Matrix reconstruction is checked. This can create an unavoidable Euler numeric wrap even though the SO(3) path is continuous; it is NOT proof of Doosan interpolation behavior.
- Net and cumulative position/rotation path budgets are bounded by limited action+1e-6mm/deg numerical tolerance. Workspace, action-age and tick-gap checks fail closed. Physical settings untouched.

### Full recorded-input replay

| Generator | Emitted offline rows | Valid segments | Translation/rotation budget violations | Translation/rotation target overshoot segments | Workspace outcome |
|---|---:|---:|---|---|---|
| original | 2143 | 223 | 11/62 | 129/208 | None |
| command | 1886 | 197 | 92/171 | 83/167 | {'reason': 'workspace_rejection', 'step': 197, 'time': 78789.251978272, 'stage': 'set_target'} |
| bounded_command_linear_slerp | 1658 | 174 | 0/0 | 0/0 | {'step': 174, 'time': 78777.511444568, 'reason': 'workspace_rejection'} |

Counts cover only the prefix before first rejection: original223 segments; proposalA197; bounded174. The bounded candidate hits unchanged workspace at step174, so it is not a completed full episode. Counts with different denominators are not a task-performance ranking.

Target overshoot is positive projection beyond the geometric endpoint; action budget separately counts net excursion or cumulative path beyond limited action. They are different metrics.

### Same terminal window200..222, seeded from prior emitted pose

| Generator | Segments | Translation/rotation budget violations | Translation/rotation overshoot segments | Workspace rejection |
|---|---:|---|---|---|
| original | 23 | 2/13 | 19/20 | None |
| command | 23 | 14/23 | 14/21 | None |
| bounded_command_linear_slerp | 23 | 0/0 | 0/0 | None |

The bounded alternative eliminates generator overshoot/budget amplification in this23-segment terminal replay. It is NOT a robot physics simulation; changed commands would change future camera feedback/policy outputs.

### Dynamic feasibility warning — do not hide this result

With a linear time law, derivative jumps remain at retarget/start/stop. Against unchanged planning limits20mm/s,20deg/s,20mm/s²,20deg/s², full-prefix offline diagnostics find136 translational-speed,357 translational-acceleration and343 rotational-acceleration violating samples. Terminal-only replay has45/42 acceleration violations. Consequently the mathematical budget fix is **NOT_PHYSICAL_READY**. No limiter was silently removed, no bound raised, and no live path changed. A future time law must honor the same physical bounds, possibly taking longer than one inference cycle.

## 3. Empirical following-error distribution: designated steps0..216

217 controller-pose samples including the initial pre-stream reference; this is the user-designated nominal subset, not independently certified fault-free. Errors use controller pose, not URDF FK, and receive-time alignment, not vendor execution timing.

| Metric | p50 | p95 | p99 | max |
|---|---:|---:|---:|---:|
| translation_error_mm | 2.016904 | 5.000695 | 7.035042 | 9.522415 |
| rotation_error_deg | 0.786100 | 2.132016 | 2.323601 | 2.832619 |

| Diagnostic candidate | Translation mm | Rotation deg | Nominal samples rejected | First terminal rejection |
|---|---:|---:|---:|---:|
| p99_rounded_up | 7.5 | 2.4 | 2/217 | 220 |
| observed_max_rounded_up | 10 | 3 | 0/217 | 220 |

Recommendation for further offline validation:10mm/3deg is a max-covering candidate;7.5mm/2.4deg is more sensitive but rejects2 historical nominal samples. Neither is installed, neither is a certified safety limit, and neither proves stopping distance. Validate on independent data and a functioning stop path before choosing. Temporal correlation precludes treating217 samples as independent trials.

## 4. Physical synthetic test DRAFT — NOT EXECUTED

`draft_servo_C_wrap_test.py` was written, syntax-checked and its pure geometry helpers unit-tested only. The physical executor was never called, no ROS node/publisher/service was created by this work.

- One +/-3deg intrinsic C motion, either positive or negative, chosen per invocation. No automatic reverse, retry, home, gripper or model inference.
- Already reviewed safe joint posture must be supplied and matched within the existing0.5deg start criterion. No invented safe pose and no automatic move to it. Current C must already be within3deg of the selected boundary; otherwise refuse.
- Separate normalized and unwrapped modes; identical SO(3) rotation is verified in pure tests. The unwrapped mode intentionally crosses the candidate Euler input range for investigation, so vendor/profile approval is mandatory.
- Eight-second analytic smoothstep profile: peak approximately0.704deg/s, acceleration0.271deg/s². Existing transport vel/acc/time remain unchanged. This test time law differs from the new linear-time replay generator to keep the diagnostic motion low-speed.
- Require reviewed controller joint limits, explicit positive joint margins, verified TCP offset, controller version/evidence, verified stop path/profile, and fresh operator/authority/stops confirmation. Default template has all approvals false and all unknown quantities null.
- Current joint proximity and nominal Jacobian/IK forecast block motion; TCP/flange transform is explicitly applied. Nominal forecast is not a vendor IK guarantee.
- Following-error guard, JointState freshness/gap checks, workspace, state and stream-stall checks inhibit dispatch. Existing HOLD(3) once on exit; ACK and fresh post-stop STANDBY/zero-velocity window must confirm. No automatic recovery after failure.

The current known Stop ACK problem means the reviewed stop prerequisite cannot simply be assumed true. The script is a draft, not authorization or a ready-to-run recommendation. No execution command is provided here.

## Files and validation

- `steps_200_222_error_growth.csv`: all requested rates and projections.
- `all_steps_error_projection.csv`: full per-step error samples.
- `bounded_commands.csv`, `bounded_segments.csv`: full-prefix candidate.
- `terminal_200_bounded_commands.csv`: same-window candidate.
- `results.json`, `input_sha256.json`: metrics, caveats and source integrity.

Added source: `servo_experimental/bounded_segment.py`, `analyze_wrap_and_bounded_replay.py`, `render_wrap_bounded_report.py`, `draft_servo_C_wrap_test.py`, `tests/test_bounded_segment.py`, and an unapproved test-contract template. Existing reference replay was imported read-only. No production launcher/limit files changed.

Final test result recorded separately in VALIDATION.txt. Draft runner was NOT invoked; only pure helpers were tested.
