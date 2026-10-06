# Allowlisted read-only service contract

Only these exact name/type pairs are accepted by `read_only_state_adapter.py`:

- `/dsr01/aux_control/get_current_posx` — `GetCurrentPosx`, request `ref=0`.
- `/dsr01/system/get_robot_state` — `GetRobotState`, empty request.
- `/dsr01/system/get_robot_mode` — `GetRobotMode`, empty request.
- `/dsr01/system/get_last_alarm` — `GetLastAlarm`, empty request.
- `/dsr01/aux_control/get_control_mode` — `GetControlMode`, empty request.
- `/dsr01/aux_control/get_control_space` — `GetControlSpace`, empty request.

All other names/types, including `get_desired_posx` and every `/motion/`, set, servo, IO, force, realtime and gripper interface, are rejected before client creation.

Attempted exactly once each: the six getters above. Results: current_posx timeout, robot_state timeout, robot_mode timeout, last_alarm success, control_mode success, control_space success. A later isolated TCP retry did not create/call a service because discovery failed (`TIMEOUT_WAITING_FOR_SERVICE`).

Observed values: control mode 3 (position control); control space 1 (joint space); last alarm level/group/index 0 with empty parameters. Empty last alarm is not treated as proof of current safety. Robot state/mode, TCP, servo state, protective-stop, E-stop and Hold acknowledgement remain unavailable.

During the subsequent 30-second stability audit all three candidate getter names remained discoverable in 31/31 polls. Nevertheless required nodes were absent in 7 polls and JointState covered only 39.82% of wall duration, so the pass gate failed. Getter requests in this stability run: **0**.

## Follow-up request

After a fresh readiness-gated 10-second JointState window passed at 100 Hz, `GetCurrentPosx(ref=0)` was requested exactly once. The request timed out at 3.000606 seconds. The response contained no TCP pose. In accordance with the stop rule, `GetRobotState` and `GetRobotMode` were not requested. The next 15.054-second passive JointState probe received zero samples although endpoints remained discoverable.

No retry, controller restart, state change, motion, IO, gripper or Hold call was made.
