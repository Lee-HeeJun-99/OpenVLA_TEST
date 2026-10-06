# ZED stream continuity audit — 2026-10-02

## Scope and safety

Subscriber-only inspection of the ZED raw and compressed RGB streams. No publisher,
service client, model inference, getter, or robot command was created or invoked.

Topics:

- `/zed/zed_node/rgb/color/rect/image`
- `/zed/zed_node/rgb/color/rect/image/compressed`

Observed image contract: 1280×720, raw `bgra8`, step 5120; compressed JPEG/BGR8.

## Results

| Probe | Stream | Samples | Source rate | Max source gap | >=100 ms gaps |
|---|---|---:|---:|---:|---:|
| SHA-256 enabled, 30 s | raw | 898 | 29.931 Hz | 283.381 ms | 43 |
| SHA-256 enabled, 30 s | compressed | 897 | 29.915 Hz | 283.381 ms | 43 |
| low overhead/no hash, 30 s | raw | 875 | 29.311 Hz | 416.711 ms | 32 |
| low overhead/no hash, 30 s | compressed | 874 | 29.327 Hz | 416.711 ms | 32 |

The raw and compressed source timestamps show the same maximum gaps and nearly the
same counts. The gaps therefore cannot be attributed only to JPEG compression or to
frame hashing in the observer. Source and local receive gaps are also closely matched,
which makes a subscriber-only DDS receive stall less likely than an upstream capture,
ZED publication, or host scheduling stall.

This does not identify a single root cause. Host load was non-trivial during the test
(`ros2_control_node` about 202% CPU, TeamViewer Desktop about 98%, and the ZED
component container about 62% in the preceding snapshot), so host scheduling remains
a live candidate. The camera continuity gate is not passed because both low-overhead
streams contain repeated >=100 ms source gaps.

## Classification

`ZED_RAW_AND_COMPRESSED_SOURCE_GAPS_REPRODUCED`

Prediction-only live Shadow remains `NOT_EXECUTED`. Before that step, repeat the
camera-only probe after reducing unrelated host load or isolating the ZED process, and
inspect ZED capture diagnostics/settings. Do not compensate by treating burst-delivered
frames as continuous 30 Hz input.

Artifacts:

- `03_shadow_mode/zed_raw_compressed_probe_20261002.json`
- `03_shadow_mode/zed_raw_compressed_low_overhead_20261002.json`
- `03_shadow_mode/probe_zed_streams.py`

## Post-relaunch follow-up

The prior gap classification is retained for provenance. In the current single-instance
minimal-observer session it was not reproduced:

| Probe | Stream | Samples | Source rate | Max source gap | >=100 ms gaps |
|---|---|---:|---:|---:|---:|
| low overhead/no hash, 60 s | raw | 3,526 | 58.796 Hz | 50.146 ms | 0 |
| low overhead/no hash, 60 s | compressed | 3,525 | 58.796 Hz | 50.146 ms | 0 |
| integrated rosbag, 31.76 s | compressed | 1,877 | 59.146 Hz | 50.021 ms | 0 |

The integrated bag also contained continuous 100 Hz JointState. Current-session
classification: `ZED_CONTINUITY_POST_RELAUNCH_PASS`. This is a session-specific
readiness result, not proof that the earlier intermittent condition can never recur.
