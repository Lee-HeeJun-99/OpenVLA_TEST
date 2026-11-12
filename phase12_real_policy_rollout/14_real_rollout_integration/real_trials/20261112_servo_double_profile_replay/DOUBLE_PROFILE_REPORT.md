# ServoL 2x profile — offline preparation

Regression validation: 482 passed, 26 existing SciPy gimbal-lock warnings, 13.51 s.

User requested doubling velocity and acceleration. Physical commands: **0**.

| Layer | Previous | New |
|---|---|---|
| Stream translation velocity | 20 mm/s | 40 mm/s |
| Stream translation acceleration | 20 mm/s² | 40 mm/s² |
| Stream angular velocity | 20 deg/s | 40 deg/s |
| Stream angular acceleration | 20 deg/s² | 40 deg/s² |
| ServoL transport vel | [20,20] | [40,40] |
| ServoL transport acc | [40,40] | [80,80] |

The transport acceleration was already 40, so its actual doubled value is 80,
not 40. Planning and transport are distinct layers.

Stream overrides are read from `configs/servo_episode.yaml:stream_planning`.
MoveLine and shared canonical policy settings are not changed. In particular,
the upstream canonical rotation delta limiter remains as previously configured;
this change does not claim to double every model rotation delta.

20 Hz publication, transport time=0.7 s, raw action gates, workspace, source-age
and sensor watchdogs, and controller safety settings remain unchanged.

## Recorded replay

Input: 20261112_190404_arm_only_servo_episode, 224 predictions.
The candidate generator produced 10 candidates and inhibited at policy step 1,
monotonic 78692.834664467, with `stream_dynamic_limit`.
This is the same target transition as the 1x replay. Raising the limits to 2x
did not remove the retargeting discontinuity. No hardware transport was created.
Recorded actual trajectories cannot serve as counterfactual following feedback.

## Physical readiness

**BLOCKED**. Release evidence flags remain false and following-error tolerances
remain unset. The launcher refuses before ROS or IO initialization. Do not mark
these evidence flags true merely to bypass the block.

Remaining: acceleration-continuous bounded retargeting; reviewed following-error
tolerances; vendor normalized-Euler boundary verification; physical Stop ACK and
joint-protection verification. These are not established by offline tests.

## Reproduction — no robot calls

```
/usr/bin/python3 replay_live_command_segment.py --trial real_trials/20261112_190404_arm_only_servo_episode --output real_trials/<new_output_directory>
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 /usr/bin/python3 -m pytest tests ../tests ../15_valid_invalid_rollout_policy/tests -q
```
