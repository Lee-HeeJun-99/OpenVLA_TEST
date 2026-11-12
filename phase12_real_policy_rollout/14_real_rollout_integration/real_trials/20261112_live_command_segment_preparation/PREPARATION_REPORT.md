# ServoL generator integration — physical execution BLOCKED

No robot call, ROS graph query, controller setting change or physical motion was performed.

## Connected changes

- Live launcher now uses command-reference position interpolation and SO(3) Slerp.
- Velocity integration and actual-pose/command-pose anchor mixing removed from this path.
- Quintic time scaling uses existing velocity/acceleration limits, without increasing them.
- Published ABC is equivalent ZYZ in [-180,180]; this does not establish the vendor's internal joint path.
- Cumulative translation/rotation cannot exceed the limited action plus numerical tolerance.
- Discrete speed/acceleration guard rejects unsafe target transitions before publication.
- Controller feedback is independently compared with previously submitted command history; stale/unknown feedback fails closed.
- Only the stream thread installs new goals, preventing a new goal from using an unpublished intermediate pose.
- Existing 20 Hz, transport time=0.7, raw translation/rotation gates, workspace, action age, gripper mode separation and one-shot HOLD remain unchanged.
- Post-stop state is refreshed while accumulating stationary JointState evidence instead of testing a stale cached response. This does not fix a vendor stop ACK=false.

## Recorded 190404 replay

224 historical predictions available. 10 candidates generated; the next candidate at
monotonic 78692.834664467, policy step 1, was inhibited by `stream_dynamic_limit`.
This is an offline prefix, not evidence that the remaining 222 targets pass.
Quintic segments starting at rest remove individual segment overshoot, but immediate
mid-segment retargeting still cannot assume continuous velocity/acceleration.
The guard deliberately prevents that discontinuity rather than relaxing limits.

Recorded actual feedback is not counterfactual feedback for a new trajectory;
following-error performance is therefore not claimed from this replay.

## Release status

`configs/servo_episode.yaml:segment_release` is intentionally unapproved. Startup
refuses before prompts, ROS initialization or gripper IO. No approval was fabricated.

Required remaining work:

1. Acceleration-continuous bounded retargeting, with the existing source-age and action-budget constraints, and successful full replay.
2. Explicit following-error thresholds based on reviewed evidence; previous 10 mm/3 deg values remain offline candidates, not active defaults.
3. Verify normalized Euler controller behavior with the separately drafted low-speed boundary protocol, not a full episode.
4. Resolve physical Stop ACK failure and verify controller joint protection. Software proof refresh alone cannot establish these.

No full physical rollout launch command is recommended at this stage. Do not set
the release evidence flags to true merely to bypass this block.

## Reproduction (offline only)

```
/usr/bin/python3 replay_live_command_segment.py --trial real_trials/20261112_190404_arm_only_servo_episode --output real_trials/<new_directory>
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 /usr/bin/python3 -m pytest tests ../tests ../15_valid_invalid_rollout_policy/tests -q
```

The test suite uses explicit fake release evidence and fake ROS transports. It
does not grant physical approval and never actuates the robot.

Validation: 481 tests passed (26 existing SciPy gimbal-lock warnings), 13.23 s.
Modified Python modules compiled successfully. Physical commands: 0.
