# Hardware-day runbook — command-disabled by default

## Current procedure (supersedes historical instructions below)

No command below was executed on a physical robot during development. Hardware initialization can acquire authority/servo-on. Only a local authorized operator may launch it. Never use fake evidence or fake result artifacts on hardware.

Set task-specific shell variables to verified existing paths before following this runbook: `PHASE12_ROOT`, `BUNDLE_ROOT`, `BUNDLE_PY`, `TRIAL_DIR`, `EVIDENCE_JSON`, `APPROVAL_JSON`, `APPROVED_MODEL_CONFIG`, `RECORDED_RGB`. `MODEL` is openvla or oft. Outputs must be new paths. Each stage requires a fresh approval artifact naming operator, model, protocol, explicit_motion_approval=true and approved_wall_time; operator evidence expires after 60 seconds.

### A. Driver — operator-only initialization

```bash
source /opt/ros/humble/setup.bash
source /home/ubuntu/robot_ws/install/setup.bash
ros2 launch dsr_bringup2 dsr_bringup2_rviz.launch.py mode:=real host:=192.168.0.110 port:=12345 model:=a0509
```

PASS: driver active, actual JointState continuous, approved state/TCP sources available. FAIL: missing data, stale state, alarm, unconfirmed authority/servo/stop. Do not automatically restart or enable RT transport to manufacture a status stream. Do not launch ActionAdapter/DoosanBridge command nodes alongside the runner.

### B. Camera

```bash
ros2 launch zed_wrapper zed_camera.launch.py camera_model:="${ZED_CAMERA_MODEL:?Set the physically verified camera model}"
```

The launch/argument exists in the installed wrapper; device model is deliberately not guessed. PASS: actual RGB topic, stable timestamps and fixed camera. FAIL: wrong encoding/topic, stale frames. Runner currently subscribes `/zed/zed_node/rgb/color/rect/image`.

### C. Model server — one model at a time

```bash
cd "$BUNDLE_ROOT"
./scripts/serve.sh vanilla 8766 0 127.0.0.1
```

Stop the vanilla server before OFT:

```bash
./scripts/serve.sh oft 8765 0 127.0.0.1
```

```bash
cd "$PHASE12_ROOT"
"$BUNDLE_PY" verify_live_model_server.py --model "$MODEL" --image "$RECORDED_RGB" --output "$TRIAL_DIR/model_health.json"
```

PASS: MODEL_HEALTH_PASS. The checker compares actual processor input sizes, means/stds, interpolation and resize strategy with local checkpoint processor files, RGB/dtypes/crop/instruction/K/proprio, then a prediction response schema. GPU runtime remains pending until this passes on actual servers. Fixture predictions are refused. Save the `health` object as `verified_model_health_contract` in operator evidence. Existing servers must include phase12-health-v1 metadata (bundle source paths are documented in the report).

### D. Evidence/preflight

```bash
"$BUNDLE_PY" pre_real_rollout_check.py --evidence "$EVIDENCE_JSON" --output "$TRIAL_DIR/preflight.json"
```

This CLI is evidence-only and reports MOTION_READY=false. Runner separately checks actual live observations, service types and watchdog. Evidence must identify the verified RobotState or optional RT robot_state/robot_mode topics; never treat a message definition as a published topic. `hardware_operator_confirmations` supports explicitly confirmed operation_mode/servo_enabled/authority with OPERATOR_CONFIRMED provenance. It cannot convert UNKNOWN to measured feedback.

### E. Live prediction-only dry-run

```bash
"$BUNDLE_PY" run_real_rollout.py --dry-run --live --model "$MODEL" --protocol short_horizon --preflight-evidence "$EVIDENCE_JSON" --output "$TRIAL_DIR/shadow.jsonl"
```

PASS: valid observations/predictions, no abort, command_issued=false and delivered/executed null. FAIL: any stale/unknown required signal, model contract mismatch, logger error, safety rejection or cadence conflict. This artifact never approves motion.

### F. Minimum-motion — separate physical approval REQUIRED

Prepare an approved COPY of the model config with motion_enabled=true and explicit_motion_approval=true; defaults are never auto-enabled. Confirm local E-stop access and no persons in workspace first.

```bash
"$BUNDLE_PY" run_real_rollout.py --live --model "$MODEL" --model-config "$APPROVED_MODEL_CONFIG" --protocol minimum_motion --preflight-evidence "$EVIDENCE_JSON" --approval "$APPROVAL_JSON" --session-results "$TRIAL_DIR/stages" --output "$TRIAL_DIR/minimum.jsonl"
```

One configured deterministic translation, no rotation/gripper. PASS requires command return and observed physical agreement; API completion is not independent physical evidence. Stop after this stage.

### G. Result review and separate approval

Inspect `stages/minimum_motion_result.json`: status PASS, no fault, correct units/frame, actual physical response acceptable. DRY_RUN_PASS/FAIL cannot unlock next stage. Confirm actual Hold/stop readiness separately. Do not reuse failed trial paths or overwrite evidence.

### H. Short-horizon — separate approval REQUIRED

```bash
"$BUNDLE_PY" run_real_rollout.py --live --model "$MODEL" --model-config "$APPROVED_MODEL_CONFIG" --protocol short_horizon --preflight-evidence "$EVIDENCE_JSON" --approval "$APPROVAL_JSON" --session-results "$TRIAL_DIR/stages" --output "$TRIAL_DIR/short.jsonl"
```

Default: 2 seconds, 10 actions, gripper disabled. No overlap; unresolved previous command aborts. Check short_horizon_result.json before proceeding. 5 Hz target cadence remains CADENCE_NOT_HARDWARE_VALIDATED until measured.

### I. Full-task — separate approval REQUIRED

Approve measured task geometry in configs/task_phase.yaml and gripper polarity/abort value; no guessed target is accepted. OFT also needs explicit model_behavior_review_passed approval, because early-close safety remains active.

```bash
"$BUNDLE_PY" run_real_rollout.py --live --model "$MODEL" --model-config "$APPROVED_MODEL_CONFIG" --protocol full_task --preflight-evidence "$EVIDENCE_JSON" --approval "$APPROVAL_JSON" --session-results "$TRIAL_DIR/stages" --output "$TRIAL_DIR/full.jsonl"
```

A grasp-close ACK barrier discards remaining stale chunk actions; watchdog remains active during pulses. Full-task sequence completion does not certify cube grasp. `physical_success` stays null until independently evaluated. Any fault inhibits commands and requests Hold/Stop; actual standstill is hardware evidence, not a boolean service return.

## Historical runbook (superseded; retained for audit)

Process-level fake graph validation, no robot driver:

```bash
source /opt/ros/humble/setup.bash
source /home/ubuntu/robot_ws/install/setup.bash
cd /home/ubuntu/a0509_vla_linux_field_bundle_20260903/lhj/phase12_real_policy_rollout/14_real_rollout_integration
ROS_DOMAIN_ID=231 ROS_LOCALHOST_ONLY=1 /home/ubuntu/a0509_vla_linux_field_bundle_20260903/environment/a6000_ubuntu22_py310/bin/python tests/fake_live_process.py
```

Ports8765/8766 must be free; the test never stops an existing server to obtain them. It starts fake publishers and fake HTTP servers and launches the actual runner with --dry-run --live. Fake evidence is refused for non-dry motion. Do not run tests in the real driver DDS domain. Real model health requires a reviewed `verified_model_health_contract` including the actual processor dictionary; do not copy the fake contract into production evidence.

Follow-up validation (offline fake DDS only):

```bash
source /opt/ros/humble/setup.bash
source /home/ubuntu/robot_ws/install/setup.bash
cd /home/ubuntu/a0509_vla_linux_field_bundle_20260903/lhj/phase12_real_policy_rollout/14_real_rollout_integration
ROS_DOMAIN_ID=231 ROS_LOCALHOST_ONLY=1 /usr/bin/python3 tests/dds_fake_integration.py
```

Use no real driver in test domain231. Live dry-run accepts `--dry-run --live`; non-dry motion is still blocked pending the software items in FINAL_READINESS_REPORT. Do not bypass that guard by modifying approvals.

Do not launch hardware during offline development. The following existing driver command is a future operator procedure, not an instruction executed in this task. Hardware initialization can acquire authority and servo-on even without a Move command.

## Terminal 1 — future operator-controlled driver

```bash
source /opt/ros/humble/setup.bash
source /home/ubuntu/robot_ws/install/setup.bash
ros2 launch dsr_bringup2 dsr_bringup2_rviz.launch.py mode:=real host:=192.168.0.110 port:=12345 model:=a0509
```

The minimal broadcaster-only observer launch is insufficient for the motion service interface. Do not start ActionAdapter, DoosanBridge or integrated VLA command launch alongside this runner.

## Terminal 2 — camera

Existing launch: `zed_wrapper/launch/zed_camera.launch.py`. Camera model/device must be confirmed before choosing its `camera_model` argument; no guessed launch command is supplied. Required subscribed topic: `/zed/zed_node/rgb/color/rect/image`. Missing/mismatched topic blocks observation.

## Terminal 3 — prediction server, one model at a time

```bash
cd /home/ubuntu/a0509_vla_linux_field_bundle_20260903
./scripts/serve.sh vanilla 8766 0 127.0.0.1
```

Stop that process before OFT:

```bash
./scripts/serve.sh oft 8765 0 127.0.0.1
```

Use existing healthcheck script and verify checkpoint8130 versus vision28560, preprocessing, K and action dimension. Do not mix proprio6000.

## Terminal 4 — offline tests, safe now

```bash
cd /home/ubuntu/a0509_vla_linux_field_bundle_20260903/lhj
/home/ubuntu/a0509_vla_linux_field_bundle_20260903/environment/a6000_ubuntu22_py310/bin/python -m unittest discover -s phase12_real_policy_rollout/14_real_rollout_integration/tests -v
```

`run_real_rollout.py --model openvla|oft --protocol minimum_motion|short_horizon|full_task --dry-run --recorded-input PATH --output NEW_PATH` uses recorded inputs, not a robot. Live dry-run additionally requires ROS Python dependencies, matching model server and subscriber inputs. Output paths must not already exist.

Non-dry runner is intentionally blocked; no executable real rollout command is approved here. Fresh evidence and approval JSON are necessary but not sufficient. No config auto-enables motion.

## Terminal 5 — monitoring

Observe append-only logs and operator hardware status. No monitoring command sends Hold/E-stop. Independent continuous watchdog integration and physical stop validation must be completed before this runbook can become an executable motion procedure.
