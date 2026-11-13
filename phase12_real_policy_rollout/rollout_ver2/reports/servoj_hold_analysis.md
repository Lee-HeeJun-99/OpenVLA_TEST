# ServoJ hold transport analysis

## ROS queue and lag

The driver subscription is reliable with depth 20, while the hold publisher is reliable with depth 1. The controller callback calls `Drfl->servoj(...)` for every dequeued message. If publication runs faster than callback/controller consumption, up to 20 old setpoints can remain in the subscriber queue. At 100 Hz that represents a theoretical 200 ms of queued samples before transport back-pressure or replacement behavior becomes visible. Reliable delivery does not imply that the newest command is consumed with zero age.

The hold target is identical in every message, so queue age cannot create a different joint target during the 10 s stream. It can still affect behavior after publication stops: callbacks already queued may continue calling ServoJ, extending the effective hold command. Therefore the test separately records:

- absolute-deadline send jitter and sent count;
- JointState receive time, fixed-target position error, velocity and state at 100 Hz;
- the full 5 s after publication stops, before MoveStop;
- the full 3 s after MoveStop;
- any motion or state transition at the two boundaries.

The later time-exploration/replay stages must additionally measure command-to-feedback phase lag because their queued targets differ. Hold evidence alone does not verify queue latency for a moving trajectory.

## `time=0.1 s`

The local API names this DRFL argument `fTargetTime`; the Python wrapper accepts a nonnegative `time`, validates it together with velocity/acceleration, and the ROS callback forwards it unchanged to `Drfl->servoj`. The available source does not define whether firmware interprets it strictly as interpolation time, target horizon, or another ServoJ timing parameter. That semantic remains **[미확인]** until time exploration.

For ideal fixed-target hold, repeated target position equals the measured initial position. Consequently the requested displacement is zero, so changing the interpolation horizon should not change the desired endpoint. It can still matter in practice through controller re-planning, numerical target mismatch, queued-command lifetime, or behavior after stream interruption. This is why `0.1 s` is documented as a hold-only candidate and why the 5 s post-stream observation is mandatory. It does not set `servoj_time_param_verified` and must not be reused for moving ServoJ policy.

Sources: `dsr_common2/include/DRFLEx.h` (`fTargetTime`), `DSR_ROBOT2.py` validation, `dsr_controller2.cpp` ServoJ callback and depth-20 subscription.
