# Runbook — preparation only, real runner currently blocked

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
