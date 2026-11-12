# 190404: stream reference audit and offline alternatives

**No robot/ROS/inference calls. No live launcher or physical settings/limits changed.**

## 1. Code evidence

- `start_servo_episode_confirmed.py:265-268`: tracker initialized from live CurrentPosX once.
- `start_servo_episode_confirmed.py:328-335`: every inference constructs endpoint from newly read real controller pose, not last command.
- `02_safety/command_disabled_runtime.py`: p_target=p_actual+1000*limited_translation; R_target=Exp(limited_world_rotvec)*R_actual.
- `servo_experimental/controller.py:102-114`: set_target replaces only target/source; commanded pose and both velocities remain.
- `servo_experimental/controller.py:135-150`: error is target minus virtual commanded state; velocity slew and trapezoidal integration persist. Slerp is local midpoint integration, NOT a bounded original-start-to-final-target interpolation.
- `start_servo_episode_confirmed.py:349-353,365-381`: target replacement and ticks are serialized, but producer/stream run independently. Tick state can change before actual publication. Normally previous tick state equals last emitted request, not controller feedback or vendor-confirmed pose.
- `servo_experimental/controller.py:151-153` and launcher `:378`: each 20Hz message carries unchanged transport time=0.7, vel=[20,20], acc=[40,40]. This is not Python interpolation duration or evidence of reaching each target in 0.7sec.
- Local driver `dsr_controller2.cpp:2217-2227`: copies pos/vel/acc/time into Drfl->servol. Internal vendor retarget scheduling is not logged and cannot be inferred here.

## 2. Step215..222 reference table

Each pose is `[x,y,z; A,B,C]`, mm / intrinsic ZYZ deg. Stream start is the LAST published-request pose before the source changes, not the first already-advanced tick. Actual pose is the logged controller sample used to form the target. ABC values are representation-nonunique.

| Step | Stream start | Actual pose | Original target |
|---:|---|---|---|
| 215 | [541.7064, 43.3747, 542.3869; 112.1871, -160.5878, -178.6790] | [540.8890, 43.3573, 544.1185; 112.1064, -160.5674, -178.3118] | [542.2131, 43.3088, 541.2766; -67.6516, 160.6375, 1.0731] |
| 216 | [542.3205, 43.2842, 541.0336; 112.4611, -160.7129, -178.9416] | [541.3362, 43.3126, 542.9911; 111.9791, -160.5506, -178.5897] | [542.9194, 43.3295, 540.1493; -68.1934, 160.6195, 0.7945] |
| 217 | [543.0433, 43.3358, 539.9574; 111.9505, -160.6332, -179.1329] | [541.9790, 43.4469, 541.9878; 110.8720, -160.5252, -179.8789] | [543.3031, 43.3984, 539.1459; -68.8888, 160.5909, -0.4971] |
| 218 | [543.3418, 43.4102, 539.0063; 111.0238, -160.5852, -180.6100] | [542.1556, 43.1413, 541.5120; 109.2797, -160.4252, 178.3923] | [543.4797, 43.0928, 538.6701; -70.4827, 160.4851, -2.2279] |
| 219 | [543.5126, 43.0820, 538.5622; 109.1319, -160.4559, -182.7183] | [542.2379, 43.3220, 541.4110; 104.2952, -160.0090, 172.7377] | [544.9440, 43.9931, 538.5691; -75.5454, 160.0505, -7.8829] |
| 220 | [544.8704, 43.9565, 538.5929; 104.4416, -160.0492, -187.8974] | [542.0808, 43.3137, 541.3238; 94.7918, -158.7990, 162.7807] | [544.7869, 43.9848, 538.4819; -85.3444, 158.0233, -17.9939] |
| 221 | [544.6878, 43.9175, 538.5393; 95.7294, -158.3195, -196.9052] | [541.5629, 43.2762, 540.4744; 78.7255, -155.7915, 145.5654] | [543.1461, 43.2932, 537.6325; -100.8923, 155.5281, -34.8301] |
| 222 | [543.0701, 43.2667, 537.5740; 84.2448, -156.3464, -209.1614] | [539.7921, 43.1969, 538.7582; 62.5220, -150.8212, 127.8836] | [540.9434, 42.6905, 537.2399; -117.6701, 150.7821, -52.3300] |

| Step | Limited rotation deg | Start/actual gap deg | Start/target distance deg | Stream cumulative rotation deg |
|---:|---:|---:|---:|---:|
| 215 | 0.850052 | 0.444525 | 0.406706 | 0.715471 |
| 216 | 0.461881 | 0.838214 | 0.425216 | 0.804703 |
| 217 | 0.850052 | 0.462328 | 0.638131 | 0.977003 |
| 218 | 0.850052 | 0.884504 | 0.548670 | 1.035567 |
| 219 | 0.773336 | 1.695240 | 1.800220 | 1.808121 |
| 220 | 1.011866 | 3.621001 | 4.149600 | 3.654075 |
| 221 | 0.804589 | 7.304051 | 7.526873 | 5.169290 |
| 222 | 0.111427 | 11.568308 | 11.673956 | 7.617239 |

At step222 the 0.111427deg limited action is relative to real pose, while tracker starts 11.568308deg away. Its target is therefore 11.673956deg from the tracker start. It traverses a cumulative 7.617239deg in that source interval. This includes the initial start-to-first-tick movement and all arc segments; the earlier 7.027deg first-to-last chord excludes the first tick and is a different quantity.

This establishes an algorithmic reference mismatch and retained-momentum/overshoot contribution. It does NOT prove why the physical controller began deviating or simulate its IK, Euler interpretation, or inner loop.

## 3. Original-stream replay validation

All2143 original stream poses reproduced exactly: maximum translation difference=0mm, rotation=0deg. Time/source/actions are from the frozen run. No vendor/robot physics is simulated.

## 4. Alternatives and diff files

- **A command reference**: endpoint = last commanded pose + limited delta; virtual state/velocities retained. Following-error guard is required to prevent commanded reference drifting away from actual pose.
- **B actual reference**: endpoint AND tracker start rebased to the same recorded real pose, velocities reset. This naive alternative is explicit for comparison, NOT approved: every rebase can jump from the prior published command and velocity continuity is lost.
- `proposal_command.diff` / `proposal_actual.diff` are exact unified diffs against the included `original_reference_adapter.py`. Included adapters compile and have no ROS/transport construction. They are offline review proposals, NOT patches silently applied to the live launcher.

### Whole recorded action sequence, guards disabled for diagnosis only

| Mode | Replayed commands | Max rotation arc/action ratio | Outcome |
|---|---:|---:|---|
| original | 2143 | 68.361 | None |
| command | 1886 | 4.930 | {'reason': 'workspace_rejection', 'step': 197, 'time': 78789.251978272, 'stage': 'set_target'} |
| actual | 2143 | 85.599 | None |

A does NOT remove all amplification; target-reference consistency alone leaves the existing integrator overshoot. A reaches unchanged workspace rejection at step197. B also does NOT solve the problem: rebasing to recorded real poses contributes artificial jumps and breaks velocity continuity. Do not adopt either naive alternative as physically validated.

### Local replay seeded from the last published state before step215

| Mode | Step222 action deg | Step222 cumulative stream rotation deg | Maximum rotation arc/action ratio |
|---|---:|---:|---:|
| original | 0.111427 | 7.617239 | 68.361 |
| command | 0.111427 | 0.478525 | 4.295 |
| actual | 0.111427 | 9.538026 | 85.599 |

The alternatives reuse recorded actual poses as exogenous inputs. Once command reference changes, those real future poses would not be the same in a physical counterfactual. Hence this is a generator stress replay, not a prediction of modified robot motion.

## 5. Added checks — offline boundary only

`servo_experimental/reference_replay.py::OfflineStreamBoundary` checks BEFORE a fake publish:

1. Net translation/rotation AND cumulative translation/rotation arc must not exceed limited-action magnitude plus explicit tolerance. Cumulative arc prevents oscillation cancellation.
2. Real/command distance uses geodesic orientation and command history at the feedback receive time, never a future candidate. Stale/future feedback fails closed.
3. Fault latches future dispatch inhibition and invokes the supplied fake stop callback once if there was prior publication. Additional candidate attempts after abort publish zero commands.

No new tracking thresholds were installed in physical configuration. Tests use numerical action-budget tolerance1e-6mm/deg. Offline following-error sensitivity uses (5mm,1deg), (10mm,2deg), (20mm,4deg), (50mm,8deg); these are diagnostic values, not approved physical thresholds. The feedback age reference is the frozen existing max_action_age_sec1.2; this does not validate it as a sensor-age contract.

### Both checks enabled

All three full-run variants inhibit at step0, before a proposed 0.915319deg excursion exceeds the first limited action0.8deg. This exposes existing overshoot even before the terminal incident. Stop callback once; commands after abort0. A strict action budget does not constitute a finished smooth trajectory controller.

Local seeded replay: original inhibits at step216; A and B inhibit at step215. Thus unsafe candidates are prevented rather than silently clipped/scaled.

### Following check isolated (budget check disabled ONLY for diagnostic isolation)

| Diagnostic bounds mm/deg | First blocked step | Measured comparison rotation deg |
|---|---:|---:|
| 5/1 | 17 | 1.049209 |
| 10/2 | 75 | 2.035391 |
| 20/4 | 221 | 7.304051 |
| 50/8 | 222 | 11.568308 |

With the loose50mm/8deg diagnostic bounds, inhibition occurs at monotonic78801.943377062, about0.161sec before recorded J5 first crossed the historical-135deg reference. No stopping-distance guarantee follows: a vendor Stop ACK/latency/physical stopping model is absent.

## 6. Disposition

Current live execution path and every physical limit/configuration remain unchanged. Added checks are IMPLEMENTED AND TESTED in the offline proposal boundary, not enabled live without a selected reference/trajectory design and justified tracking tolerances. Simple reference switching alone is insufficient; endpoint-bounded interpolation/braking with continuous replans and validated following/joint protection is still needed.

Added source files: `servo_experimental/reference_replay.py`, `replay_servo_references.py`, `render_servo_reference_report.py`, `tests/test_servo_reference_replay.py`. Original trial remains unchanged.

Validation: **460 passed**,26 existing SciPy gimbal-lock warnings. New reference/guard suite10 tests. Original replay equality, named-anchor behavior, arc overshoot, following error, epoch-free timing, future-command avoidance, equivalent Euler representations, nonfinite candidate inhibition, stale feedback and post-abort no dispatch are covered.

Reproduce: `/usr/bin/python3 replay_servo_references.py` then `/usr/bin/python3 render_servo_reference_report.py`. Existing result directory must not already exist for replay. All output paths are separate from source trial.
