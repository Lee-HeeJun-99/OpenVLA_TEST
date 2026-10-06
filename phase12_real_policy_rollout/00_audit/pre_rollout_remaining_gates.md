# Remaining gates before any rollout

## Completed engineering checks

1. JointState startup transient is isolated by a fail-closed readiness gate; 3 x 60-second accepted windows passed at about 100 Hz with zero >=100 ms gap.
2. Phase 11 model/input identities and action semantics are frozen.
3. Offline gripper polarity, 1000 mm/m conversion, Doosan ZYZ conversion and sequential OFT K=5 handling are tested.
4. Command-free runtime safety supervisor covers watchdog, rate, step, velocity, acceleration, stale, duplicate, NaN/Inf, inference, communication and logger failure.
5. Logger has per-record fsync, sequence numbers, disk-space guard, crash-visible partial marker and atomically replaced completion status.

## Blocking physical/signal checks

- physical E-stop and immediate operator access;
- protective-stop, servo and robot-mode current-state signals;
- Hold acknowledgement path;
- measured TCP source stable enough for the watchdog;
- measured gripper feedback or an explicit approved open-loop gripper risk decision;
- operator-approved workspace, joint/TCP velocity, rotation velocity and acceleration limits;
- signed local-operator checklist and named motion approver.

## Blocking model/Shadow checks

- OFT health is verified live for vision step28560/K=5/center-crop/no-proprio.
- OpenVLA port 8766 is not running and must be health-checked after OFT is stopped normally by its owner; models must not coexist on the 24 GiB GPU.
- Stationary observation is blocked by missing measured TCP/current safety signals.
- Prediction-only stationary Shadow is consequently not executed.
- Existing Phase 10/11 offline reference comparisons are valid offline evidence but do not replace live Shadow.

## Required sequence

1. Local operator completes physical checklist and provides approved numeric limits.
2. Establish read-only measured TCP, robot state/mode, servo/protective-stop and Hold acknowledgement.
3. Decide/verify gripper feedback wiring.
4. Run command-free stationary observation for 10–30 seconds.
5. Run OpenVLA then OFT stationary prediction-only Shadow sequentially, with executed/delivered commands null.
6. Re-run static capability and logger integrity checks.
7. Issue a pre-rollout report. Even a PASS then requires a separate explicit motion approval.
