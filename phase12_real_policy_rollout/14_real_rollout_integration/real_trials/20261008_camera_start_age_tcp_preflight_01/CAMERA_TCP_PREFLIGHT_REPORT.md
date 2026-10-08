# Camera start-age / TCP preflight

Branch: `lhj-research`. Baseline HEAD: `ba94b54cdc38de34f493df903e63c8a1a85a9466`.

## Current-session results

- Hardware: current safe read-only scalar responses REAL / AUTO / STANDBY.
- JointState: RUNTIME_READY; fault null; post-ready 1,111 samples over 11.108 s. Maximum source gap 11.967 ms, receive gap 19.805 ms; >=100 ms events 0.
- Start pose: max error 0.140187 degrees, within existing 0.5 degrees. No reset motion.
- Model: current strict OpenVLA identity health PASS; real camera prediction exactly once; finite 7D response.
- Camera: real 1280x720 bgra8 -> RGB -> JPEG -> processor. Exact source/model image hashes and input files preserved in this directory.

| Timing | Seconds |
|---|---:|
| Receive age at inference start | 0.028760680 |
| Receive age at inference end | 0.477575621 |
| ROS source age at inference start | 0.113568783 |
| ROS source age at inference end | 0.562444925 |
| Request / response latency | 0.448814941 |

CAMERA_PREFLIGHT_PASS uses receive age at inference start <0.5 seconds, clean concurrent JointState and valid model response. Source/end ages are preserved diagnostics, not substituted into this gate. Source age is ROS-clock based; receive age is host-monotonic based. They are not interchangeable.

## Code changes and verification

Selected-frame freshness uses request-start age in the integrated LiveObservation/runner handoff, flange-only Shadow and prediction-only batch paths. End age is retained independently. The live physical runner additionally records source ages using the ROS node clock. A stale/negative receive age at request start is rejected before the live request. Latest camera stream monitoring remains independent and unchanged. JointState startup/runtime gates and production translation/rotation limits remain unchanged.

59 targeted tests passed: timing/handoff (6), single prediction hardware gate (5), JointState startup (11), camera encoding (4), live contracts (4), flange Shadow (4), integration (8), INVALID classifier (11), batch aggregation (6). No test used actual robot command services.

## Separate blockers

TCP_CONTRACT_FAIL: TCP_UNVERIFIED. Current graph has no discovered RobotState/RobotStateRt Cartesian state topic. Existing blank name getters were not repeated. CurrentPose null-pointer risk and CurrentPosx prior stability risk were not bypassed. Flange/cache/config candidates cannot establish current active TCP binding. No current pendant evidence was supplied. Required evidence is the active TCP name, matching offset, current Cartesian pose, BASE frame, units/orientation convention and confirmation time. Do not assume zero offset or equate flange with TCP.

RAW_TRANSLATION_LIMIT_DIAGNOSTIC: raw translation [1.324056, 2.633544, -7.308878] mm, norm 7.880887 mm > existing 4 mm. This limit is untouched. Raw rotation norm 0.016676774 radians (~0.9555 degrees); gripper closedness 0.001960784. No clipping/scaling.

## Final

CAMERA_PREFLIGHT_PASS. TCP_CONTRACT_FAIL. PHYSICAL_ROLLOUT_BLOCKED by unverified TCP and existing action limit; additional physical safety readiness still requires actual evidence, not inference from STANDBY.

Physical commands: 0. Prediction requests: 1. No motion/gripper/stop clients were used. Command requested/issued false; delivered/executed action null. No task trial started, so no synthetic SUCCESS/FAILURE/INVALID task outcome assigned. Existing driver, camera and GPU server sessions were not stopped or restarted.
