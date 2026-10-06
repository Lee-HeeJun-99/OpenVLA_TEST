# Doosan state-stack 30-second stability audit

Timestamp: 2026-10-02 KST. Result: `DOOSAN_STATE_STACK_UNSTABLE`.

## Initial read-only state

- Runtime HEAD `86eaa9632d651eb907332334d02f32c1461850d7`; pre-existing dirty tree preserved.
- `dsr_controller2`: active after delayed `list_controllers` response.
- `joint_state_broadcaster`: active after delayed `list_controllers` response.
- `list_hardware_components`: first 10-second attempt timed out; no successful hardware result was obtained in this run.
- Process PID visibility: relevant driver process was not visible in the command execution namespace; `NOT_VERIFIED`.
- Recent controller log candidate: `/home/ubuntu/.ros/log/ros2_control_node_2873331_1790902417863.log`.

## Passive 30-second result

| Metric | Result |
|---|---:|
| wall duration | 30.0107 s |
| JointState samples | 1,196 |
| source timestamp span | 11.9500 s |
| duration coverage | 39.82% |
| rate during received span | 99.9997 Hz |
| interval min / max / std | 0.008567 / 0.011527 / 0.000117 s |
| non-monotonic / duplicate timestamp | 0 / 0 |
| nonfinite position / velocity | 0 / 0 |
| missing canonical joint | 0 |
| effort unsupported/NaN | 1,196 (accepted as unavailable) |
| observed wire order | `joint_1, joint_2, joint_4, joint_5, joint_3, joint_6` |
| graph polls | 31 |
| three getter service down polls | 0 each |
| JointState publisher-zero polls | 0 |
| `dsr_controller2` node down polls | 7 |
| `joint_state_broadcaster` node down polls | 7 |

The received samples themselves are valid and exactly reorderable by joint name, but they cover only about 12 of 30 seconds. Both required nodes were missing from discovery in 7/31 polls. Therefore the stack did not meet continuous availability requirements even though services and publisher endpoints stayed discoverable.

The parallel `ros2 topic hz`/log-watch command produced no usable output and is not counted as evidence. No new matching controller-log line was captured for `disconnect|timeout|communication|monitor|socket|read|failed|error|exception` during that attempt. Absence of a captured line is not proof that the driver was healthy.

Per protocol, getter calls after this stability test were skipped. No automatic recovery, controller switch, restart, or hardware-state change was attempted.

## JointState-only rosbag isolation

A separate 35-second recorder with no graph polling or getter calls produced 1,902 messages. Bag receive duration was 25.119737 s. Source timestamp span was 28.179993 s; aggregate rates were 67.459 Hz source and 75.678 Hz receive. Source max gap was 3.069995 s (three gaps ≥1 s), and receive max gap was 3.072055 s (two gaps ≥1 s). Position/velocity/name errors, duplicate stamps, and non-monotonic stamps were all zero; effort was unsupported for all messages.

This earlier interpretation was superseded by the gated subscriber tests below. The bag and first probe included transient-local/discovery startup samples in their measurement window and therefore did not establish a mid-stream driver stall.

## Corrective investigation and readiness gate

`sensor_msgs/msg/JointState` has no sequence-number field. A passive probe therefore assigned a local receive index and recorded source ROS time and callback monotonic receive time. A reproduced 3.06-second event occurred only at startup:

- local index 0 → 1: source gap 9.994905 ms, receive gap 3.067522073 s
- local index 1 → 2: source gap 3.060007067 s, receive gap 2.089217 ms

Repeated tests placed these events at the first samples of a newly created subscriber. A continuous 75-second interval then delivered 7,502 samples at 100.000 Hz with no gap ≥50 ms. A 180-second test had one startup-only transient at indices 0–2 and no later large gap.

A `FeedbackReadinessGate` was added to `03_shadow_mode/read_only_state_adapter.py`:

- use `RELIABLE` + `TRANSIENT_LOCAL` subscription QoS;
- reject initial samples until at least 2 seconds and 180 samples are continuous;
- restart warm-up for any source/receive gap over 50 ms, invalid sample, duplicate, or non-monotonic timestamp;
- after READY, revoke readiness for a gap over 100 ms and require warm-up again;
- never treat NaN effort as invalid because the driver does not support it;
- canonicalize joints by name, not wire-array index.

Three independent gated 60-second runs passed:

| Run | Accepted samples | Source rate | Receive rate | Maximum receive gap | Gap ≥50 ms | Gap ≥100 ms |
|---|---:|---:|---:|---:|---:|---:|
| 1 | 6,001 | 100.000038 Hz | 99.999953 Hz | 12.245 ms | 0 | 0 |
| 2 | 6,001 | 99.999977 Hz | 99.999452 Hz | 12.165 ms | 0 | 0 |
| 3 | 6,001 | 99.999921 Hz | 99.999860 Hz | 12.626 ms | 0 | 0 |

Run 2 observed six approximately 3-second startup artifacts before readiness. The gate excluded all of them and began the 60-second acceptance interval only after continuity stabilized.

Corrected classification: `INITIAL_DDS_DISCOVERY_TRANSIENT_MITIGATED_BY_READINESS_GATE`. No sustained JointState feedback stall was observed after the gate entered `FEEDBACK_READY`.
