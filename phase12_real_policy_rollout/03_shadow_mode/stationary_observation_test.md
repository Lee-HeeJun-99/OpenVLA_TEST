# Stationary observation test

Classification: `STATIONARY_OBSERVATION_TEST_NOT_EXECUTED`.

Reason: the new 30-second prerequisite test failed. JointState was valid at 100 Hz within its active segment but covered only 39.82% of wall duration, and required nodes were missing in 7/31 polls. No getter was called after the failed gate. Camera-only records were not accepted as valid synchronized observation samples.

Recorded synchronized samples: 0. Executed action and robot-delivered command therefore have no sample rows; the adapter contract fixes both to null and `command_issued=false`.

The paragraph above is preserved as the original Gate 1 observation. It is superseded
for the current single-instance minimal-observer session by the post-relaunch result
below; it is not deleted or rewritten as if it had never occurred.
## Minimal observer stationary smoke — 2026-10-02

Classification: `STATIONARY_OBSERVATION_TEST`

Status: `PARTIAL_PASS_CAMERA_GAPS`

로봇을 움직이지 않고 다음 두 topic을 11.31초 동안 subscriber-only rosbag으로 기록했다.

- `/dsr01/joint_states`
- `/zed/zed_node/rgb/color/rect/image/compressed`

JointState:

- 1,132 messages
- source rate 100.000004 Hz
- source max gap 12.283 ms
- 50/100 ms 이상 gap 0
- position/velocity invalid 0
- missing joint 0
- observed order: `joint_1, joint_2, joint_4, joint_5, joint_3, joint_6`

Camera:

- 277 messages
- source rate 24.863541 Hz
- source max gap 516.908 ms
- source gap >=50 ms: 66
- source gap >=100 ms: 24
- image hash unique/duplicate: 277/0

JointState data plane은 통과했지만 camera source에 0.5초 이상의 gap이 있어 model Shadow 입력 안정성 기준은 아직 통과하지 못했다. ZED publisher 설정, host load 및 image transport 경로를 read-only로 추가 확인해야 한다.

Minimal observer에는 measured TCP가 없으므로 `measured_tcp=null`, `measured_tcp_status=NOT_AVAILABLE_MINIMAL_OBSERVER`로 기록했다. FK를 measured feedback으로 표현하지 않는다.

안전 필드:

```text
executed_action=null
robot_delivered_command=null
command_issued=false
```

Bag: `03_shadow_mode/stationary_observation_smoke_20261002_1629/`

Summary: `03_shadow_mode/stationary_observation_smoke_20261002_1629_summary.json`

## ZED raw/compressed follow-up — 2026-10-02

Two 30-second subscriber-only probes compared raw and compressed streams. The first
hashed every frame; the second disabled hashing to remove that observer load.

- Hash probe: raw/compressed 898/897 samples, about 29.9 Hz, identical maximum source
  gap 283.381 ms, and 43 source gaps >=100 ms on both streams.
- Low-overhead probe: raw/compressed 875/874 samples, about 29.3 Hz, identical maximum
  source gap 416.711 ms, and 32 source gaps >=100 ms on both streams.
- Raw and compressed source-gap agreement rules out JPEG transport as the sole cause.
- Removing SHA-256 did not eliminate the gaps.

Classification: `ZED_RAW_AND_COMPRESSED_SOURCE_GAPS_REPRODUCED`.

Camera continuity remains blocked; prediction-only Shadow was not started. Robot,
getter, service, model, and command operations remained zero.

## Post-relaunch integrated stationary observation — 2026-10-02

Classification: `STATIONARY_OBSERVATION_TEST`

Status: `PASS_POST_RELAUNCH`

The single-instance minimal observer was kept running and the following topics were
recorded together in one subscriber-only rosbag for 31.760 seconds:

- `/dsr01/joint_states`: 3,177 messages, 100.000029 Hz, source max gap
  10.637 ms, gaps >=50/100 ms 0/0, invalid position/velocity 0/0.
- `/zed/zed_node/rgb/color/rect/image/compressed`: 1,877 messages,
  59.145581 Hz, source max gap 50.021 ms, gaps >=100 ms 0, 1,877 unique
  image hashes and 0 duplicate hashes.

The immediately preceding 60-second raw/compressed subscriber probe also passed:
raw/compressed 3,526/3,525 messages at about 58.8 Hz, with no >=100 ms source
gap. Therefore the earlier camera-gap failure is retained as a historical observation
but is not reproduced in the current post-relaunch session.

Continuous controller-measured TCP is still unavailable. JointState-derived URDF FK
is retained as `computed_link6_flange_not_measured_tcp`; its one-pose pendant
cross-check passed, but it is not relabeled as measured TCP.

Safety fields remain:

```text
executed_action=null
robot_delivered_command=null
command_issued=false
```

No getter, model server, publisher, service/action client, motion, gripper, Home,
trajectory, Hold/E-stop, or closed-loop operation was executed. The next eligible
non-motion gate is sequential prediction-only stationary Shadow, not rollout.

Artifacts:

- `03_shadow_mode/stationary_observation_post_relaunch_20261002/`
- `03_shadow_mode/stationary_observation_post_relaunch_20261002_summary.json`
- `03_shadow_mode/post_relaunch_jointstate_60s_20261002.json`
- `03_shadow_mode/post_relaunch_zed_60s_low_overhead_20261002.json`
