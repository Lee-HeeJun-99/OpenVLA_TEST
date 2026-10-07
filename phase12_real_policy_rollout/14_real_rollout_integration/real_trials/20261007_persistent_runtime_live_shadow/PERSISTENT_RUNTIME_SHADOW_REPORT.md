# Persistent runtime live Shadow — 2026-10-07

Branch `lhj-research`; starting HEAD `5739318b7c1e643568b2cab1623f219c0dd75ba9`.

## Result

`LIVE_SHADOW_PASS` is a prediction-only result, NOT motion authorization.

- Discovery to FIRST_FRESH_SAMPLE: 19.0395 s, same ROS clock header age <100 ms.
- Fixed warm-up: 10 s; four preserved >=100 ms warm-up observations (two receive stalls around 3.067 s and two source jumps around 3.060 s). No reset.
- Persistent subscriber Shadow: 69 predictions / 30.2517 s.
- Runtime >=100 ms events: 0; watchdog fault: none.
- Selected-frame camera age maximum: 0.463460 s; camera failures: 0. Actual ZED BGRA8 to RGB/JPEG path retained; image hashes are in JSONL.
- OpenVLA strict identity/processor validation and actual inference: PASS; step8130, K=1, action_dim=7, proprio=false. Lightweight identity comparisons replace repeated heavy metadata comparison.
- Translation maximum 2.921875 mm; rotation maximum 3.736513 degrees; magnitude threshold exceedances 0. Rotation maximum is higher than previous ~0.6 degree observations; this is not evidence of identical model behavior across trials.
- Motion safety rejects: 69/69, each with `tcp_failure` and `gripper_unknown_state_blocks_command`. Expected unknown-context rejection, NOT task failure or motion readiness.
- TCP context: `FK_ESTIMATED_FLANGE`; tool/TCP contract remains unverified. No zero offset assumed.
- Physical pose/gripper/Home/trajectory/Hold/Stop/rollout commands: 0.

## Implementation and lifecycle

`jointstate_runtime_startup.py` separates WAIT_DISCOVERY, WAIT_FIRST_FRESH_SAMPLE, WARMUP, RUNTIME_READY and latched RUNTIME_FAULT. Maximum first-fresh wait is 60 s. Runtime source/receive gaps must remain strictly positive and below100 ms. Invalid names/finite values and non-increasing timestamps fault the runtime. Silence is checked independently of callback arrivals.

`flange_prediction_shadow.py` uses one rclpy node, one JointState subscription and one camera subscription for startup and inference. There was no separate subprocess precheck in this path; the old rolling20 s check was replaced, not hidden. `live_observation.py` reuses the same readiness state machine for the general rollout observation adapter. No independent30 s precheck subscriber is created.

General live adapter modifications were unit-regression tested but were not executed with real motion. This trial executes the flange-only dry-run branch of `run_real_rollout.py`.

## Commands and tests

After sourcing `/opt/ros/humble/setup.bash` and `/home/ubuntu/robot_ws/install/setup.bash`:

```bash
/home/ubuntu/a0509_vla_linux_field_bundle_20260903/environment/a6000_ubuntu22_py310/bin/python run_real_rollout.py --dry-run --live --model openvla --protocol short_horizon --prediction-only-flange --output real_trials/20261007_persistent_runtime_live_shadow/live_shadow.jsonl
/usr/bin/python3 -m unittest discover -s tests -v
```

Integration-directory unittest suite: 48 PASS / 0 FAIL / 0 SKIP, including6 new readiness tests. This does not claim the whole repository test suite or physical validation.

Driver PID3061204 was retained; no driver/controller restart, setters or motion calls. Runtime100 ms and camera0.5 s thresholds unchanged.

## Next gate

Completed passive follow-up: `../20261007_161555_automatic_hardware_state_audit`. Its new subscriber received0 JointState messages in20 s while camera received888 frames. This is a separate discovery observation, not a runtime gap in the completed persistent Shadow. No affirmative hardware state/TCP/tool binding was established; getter calls0; TCP UNKNOWN and next allowed stage BLOCKED. See `tcp_precheck.json`.

Automatic subscriber-only TCP/hardware source audit follows this PASS. GetCurrentTcp/GetCurrentTool callbacks only invoke DRFL `get_tcp()`/`get_tool()` and return names, not transforms. Source-only read semantics do not prove vendor live stability or affirmative controller connection. No getter may be called without sufficient current evidence. Unknown TCP/hardware/operator confirmations keep minimum-motion BLOCKED.
