# OpenVLA real GPU/input validation — 2026-10-06

Starting branch lhj-research, HEAD 587be4c97530772314729b150282799719eba2b2; initial working tree clean.

## Latest verdict

BLOCKED_TECHNICAL_VALIDATION_FAILED. Motion remains unauthorized. JointState root-cause research was not expanded in this attempt.

- OpenVLA GPU health: PASS, verify_live_model_server.py printed MODEL_HEALTH_PASS. Actual RTX3090 checkpoint vanilla_s1_balanced_step8130, openvla_token, K1/action_dim7/no proprio. RGB uint8 input, BF16 model tensor, crop-bottom0, matching trained bicubic224 dual-backbone processor and normalization/instruction contract. Exact health and sample inference latency: model_health.json. This was real GPU inference, not fixture.
- Camera preprocessing: PASS conversion/encoding/hash path, not an unconditional all-time camera freshness approval. Actual1280x720 bgra8 converted to RGB with alpha discarded and row padding honored. live_rgb.png is the captured lossless RGB sample used by the verifier; input_validation.json/camera_frames.json preserve source/RGB hashes. Runtime now also records JPEG model-input SHA256 separately. Three new conversion tests PASS.
- TCP: BLOCKED/UNVERIFIED. /doosan/current_pose and configured hardware status sources were absent in inspected graph. No getter was retried. Existing historical Tool_v1 zero-offset/one-pose FK cross-check is recorded in pre_rollout_verified_config.yaml; it does not establish that current active controller tool is unchanged. No current evidence was fabricated. Historical pose was not reused as a fresh measured value.
- JointState motion gate: FAIL.35s simultaneous subscriber observation:353 messages, first delay15.998s, max receive gap3.077163s, max source gap6.130023s. Latest age0.048742s at capture end does NOT erase recent discontinuity. Recorded names/values and source/receive timestamps: joint_samples.json.
- Camera received956 frames; first delay0.197s, max receive gap0.474935s, max source gap0.399880s, latest age0.045193s. Camera reception is confirmed; timing statistics apply to this observation window only.
- Live integrated prediction-only dry-run: BLOCKED/NOT_EXECUTED because TCP and live input readiness are not satisfied. The single real-image GPU contract inference must not be mislabeled a 30–60s integrated Shadow PASS. Watchdog/safety thresholds were not relaxed to force a run.
- Minimum-motion: BLOCKED.
- Hold/Stop, physical gripper, short-horizon, full rollout: NOT_EXECUTED.
- OFT: NOT_STARTED.

## Actual operations / changes

Read-only branch/HEAD/status, GPU/process/port/topic inspections; verified prediction-only `./scripts/serve.sh vanilla 8766 0 127.0.0.1`; camera-only `ros2 launch zed_wrapper zed_camera.launch.py camera_model:=zed2i`; 35s subscribers; HTTP health/predict verifier. Doosan driver was not restarted. Model and camera processes remain running for further approved validation; no command node was started.

Code change: live_observation.py gains tested BGRA/RGBA row-stride-aware RGB conversion and RGB/JPEG hashes; tests/test_camera_encoding.py added. Production safety thresholds/config, action units, watchdog and command gates unchanged.

All robot command, motion service/action, gripper, Home, trajectory, Hold/E-stop, AI action publication and real rollout executions by this attempt:0. executed_action=null and robot_delivered_command=null. Server sample prediction is not delivered action.

Next allowed stage: current-tool/FK evidence validation without getter retry and command-disabled prediction diagnostics. Integrated live Shadow requires valid fresh TCP/Joints and hardware-state evidence. Minimum motion remains barred until a recent continuity window, watchdog/logger and operator/E-stop/workspace gates all PASS, followed by separate explicit motion approval. No authorization artifact was generated.
