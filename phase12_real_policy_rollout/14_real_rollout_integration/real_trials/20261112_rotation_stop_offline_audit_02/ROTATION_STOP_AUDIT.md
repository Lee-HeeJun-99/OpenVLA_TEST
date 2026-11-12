# ServoL rotation / stop audit — offline only

Source: `../20261112_182408_arm_only_servo_episode/`. Original artifacts unchanged.
Physical commands: **0**. Safety thresholds and automatic retry policy unchanged.

## Root cause and target 63 (step 62)

| Quantity | Degrees |
|---|---:|
| Naive current-to-filtered-target Euler component difference L2 | 395.324365796 |
| Current-to-filtered-target geodesic angle | 2.465166812 |
| Raw model rotation-vector norm | 4.149291218 |
| Current-to-raw-target geodesic angle | 4.149291218 |
| Existing raw rotation hard limit | 4.0 |

Before: `raw_rotation_step_limit` rejection. After: same rejection.
The raw action is a BASE rotation vector, **not a ZYZ Euler difference**.
The recorded target was calculated after the existing rotation delta limiter.
Comparing that target to the raw gate would confuse two different quantities.
The Euler representation jump exists but did not cause this recorded rejection.

## Changes in this task

- `02_safety/action_contract.py`: intrinsic-ZYZ geodesic helper, `(R_prev.inv()*R_new).magnitude()`.
- `02_safety/command_disabled_runtime.py`: compose the raw target, measure geodesic angle and enforce the unchanged 4-degree raw gate; record angle.
- `servo_experimental/controller.py`: explicit linear-position/Slerp-rotation segment helper; tick uses the midpoint of the acceleration-bounded local segment. This is mathematically equivalent to the existing trapezoidal SO(3) update, not a new unbounded direct interpolation to each inference target. Existing velocity/acceleration state and limits remain.
- Same controller: nearest equivalent intrinsic ZYZ including signed B and 360-degree wraps. At numerical singularities (`abs(sin(B)) < 1e-8`), preserve previous A and solve C. Reconstruction must be within 1e-6 rad. Output retains the existing nearest-unwrapped convention; no new angle clipping range.
- `start_servo_episode_confirmed.py`: after a successful Stop ACK, obtain fresh read-only robot state even when an earlier fault is latched. Missing state evidence no longer silently results from the `not guard.reason` branch. Post-stop getters do not depend on the already-failed model/camera health; timeout and response validation remain. No extra motion, stop retry, or recovery.
- `servo_experimental/stop_evidence.py`: explicitly document host monotonic receive-time comparison; distinguish missing timestamps from malformed timestamps. JointState source stamps are used only for source-to-source continuity, never compared to host monotonic time.
- Added `tests/test_servo_rotation_geometry.py` and `replay_servo_rotation_audit.py`.

## Stop evidence diagnosis

Original trial: Stop ACK true, but `final_robot_state_received_monotonic` and final robot-state evidence absent. The old launcher skipped state collection because the rotation fault was already latched. This produced `invalid_stop_timestamps`. The inspected code did **not** compare ROS epoch stamps directly against `stop_requested_monotonic`.

The fix still refuses to confirm stopping without ACK, fresh post-request STANDBY, and a clean post-request exact-zero JointState window. It cannot retroactively prove that this historical trial stopped: the missing state evidence remains missing. Future dispatch stays inhibited after abort regardless of proof result.

## Replay and tests

All 63 recorded target segments were replayed without ROS, with 21 linear/Slerp samples per segment. Maximum orientation reconstruction error: **1.441162703e-15 rad**. Rejected target 63 is included for analysis only, not execution.

Reproduce from `14_real_rollout_integration`:

```bash
/usr/bin/python3 replay_servo_rotation_audit.py \
  --trial real_trials/20261112_182408_arm_only_servo_episode \
  --output real_trials/rotation_stop_replay_new

PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 /usr/bin/python3 -m pytest \
  tests ../tests ../15_valid_invalid_rollout_policy/tests -q
```

Tests cover B=0 and +/-180 neighborhoods, signed-B equivalent representations, wrap boundaries, reconstruction under 1e-6, fake jumps, genuine rotations around the unchanged 4-degree boundary, shortest-arc Slerp, and ROS epoch source stamps coexisting with monotonic receive times.

Final regression result: **436 passed**, 14 SciPy gimbal-lock warnings from canonical Euler encoding in singular test cases; rotation-matrix reconstruction assertions pass. Runtime: 11.68 seconds. No physical calls.

Pre-existing worktree edits (translation endpoint mode and SpeedL/stop protection) were preserved; repository-wide diff includes those earlier changes as well.
