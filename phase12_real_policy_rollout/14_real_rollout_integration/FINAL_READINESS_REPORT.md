# Final readiness report

## Latest follow-up — live contract and process E2E

This section supersedes earlier follow-ups below. **131 unittest PASS / 0 FAIL / 0 SKIP**, plus **7 process scenarios PASS**: separate OpenVLA and OFT live dry-run subprocesses (10 actions each), camera loss, JointState loss, TCP loss, RobotState loss, protective-stop fault. Test graph is localhost-only domain231; HTTP outputs are explicitly fixture data, not research predictions. No physical command executed.

Completed additions:

- RobotStateMonitor consumes the actual generated RobotState schema, tracks receive age, disconnected/stop states, and refuses unknown servo/manual-auto values. Source topic is supplied explicitly, never assumed from a message definition.
- ConcurrentLogger serializes JSONL writes; 80 records from four threads had no corruption, duplicate IDs or missing sequence numbers.
- Delayed real DDS fake-service request + independent watchdog fault + exactly one Hold request + terminal ABORTED_IN_FLIGHT passed. Physical stop remains UNVERIFIED. New command after abort was rejected.
- AbortBoundary is idempotent; cancellation is not asserted for already delivered service requests. Scheduler enforces actual dispatch spacing without relaxing production rate limits.
- Both model server implementations now expose versioned preprocessing metadata from their actual image processor, RGB/uint8 ingress, bfloat16 tensor path, crop and instruction handling. Strict nested metadata comparison is wired into live observation. Servers were syntax-checked, not GPU-loaded in this task.
- Fake graph process testing found and fixed initial TCP discovery and receive-time snapshot races. Process fixtures cannot satisfy non-dry approval.

### Verified driver source limitations

`dsr_controller2.cpp` optionally publishes `/rt_topic/robot_state` and `/rt_topic/robot_mode` as Float32MultiArray from selected RT fields when `use_rt_topic_pub_` is enabled. It also publishes RobotDisconnection and RobotError. The presence of RobotState.msg does not mean the controller publishes a full RobotState topic. The optional RT publisher performs RT reads and must not be enabled here merely to make tests pass.

RobotState.actual_mode is POSITION/TORQUE, not MANUAL/AUTO. Explicit servo-enabled is not identified in these messages. The implemented fake RobotState source uses a clearly marked synthetic AUTO/servo value and cannot authorize hardware. A real verified adapter for those signals is still needed; no operator checklist is promoted to a live measured signal.

### Remaining software blockers

1. Verified real servo/manual-auto source adapter and optional RT-state integration. The monitor rejects unknowns, and unconditional non-dry guard is retained until this is resolved.
2. Actual GPU server health snapshot and preprocessing-contract verification. Payload generation and fake HTTP strict validation are implemented, not real-checkpoint runtime validated.
3. Remaining end-to-end injections: servo drop, model health mismatch, logger failure and full-stage/minimum-motion intercept on the process graph. DDS timeout/failure tests and component tests do not replace all these paths.

Live/recorded software readiness must therefore not be labeled hardware-only. **PARTIAL_REAL_COMMAND_INTEGRATION_COMMAND_DISABLED** remains the verdict. Existing runtime original and production thresholds are unchanged; two prediction-only server source files were updated with metadata. No real getter, motion, gripper, Home, trajectory, Hold/E-stop or rollout was called.

## Follow-up implementation — 2026-10-06

Added independent `hardware_watchdog.py`, interruptible gripper pulse cancellation with mandatory abort value, `task_phase.py` geometry-driven sequence detector, `oft_action_scheduler.py` single worker/no-overlap target cadence, strict `model_health.py`, and actual generated-service DDS tests. Live dry-run CLI now accepts `--live`; watchdog/scheduler/phase modules are wired into that path. Full-task geometry defaults null and must be approved. Physical grasp remains UNVERIFIED even when sequence completes.

Latest tests: 100 core + 4 readiness + 19 integration + 3 isolated DDS = **126 PASS / 0 FAIL / 0 SKIP**. DDS test used ROS_DOMAIN_ID=231 and ROS_LOCALHOST_ONLY=1, fake nodes only, no robot driver launch. Actual service discovery, generated request serialization, response parsing, delayed-response timeout, success=false and server disappearance passed. First sandbox run emitted transport permission errors; the permitted localhost-only run passed without those errors. Fake DDS requests are not physical commands.

Remaining software blockers:

- Continuous protective-stop/servo/robot-state source is not integrated; watchdog currently consumes fresh operator evidence for those facts. That is not continuous hardware feedback.
- Strict preprocessing validator exists and is tested, but existing model health payloads do not expose all required normalization/dtype/input fields and live server integration is not complete.
- The new command worker/watchdog concurrent abort path still needs end-to-end interruption and logger concurrency verification, including unresolved in-flight commands. Component tests alone do not establish that property.
- Live dry-run was not executed; recorded delegate does not test all newly wired background modules. Non-dry guard remains intentionally active.

**Verdict remains PARTIAL_REAL_COMMAND_INTEGRATION_COMMAND_DISABLED. HARDWARE_VALIDATION_ONLY_REMAINING is not supported.** No production threshold changed. All physical command counts remain zero.

2026-10-06

| Item | Result |
|---|---|
| Real command request code | IMPLEMENTED, default disabled |
| Approval isolation | PASS in mock tests |
| ACK/hold/stop transitions | PASS in fake transport; physical meaning unvalidated |
| Dry-run controller | PASS, no transport calls |
| Mock transport command+ACK | PASS |
| Fake ROS DDS server integration | NOT_EXECUTED |
| Logging / descriptive metrics | IMPLEMENTED; physical success null |
| Continuous watchdog / live abort integration | NOT_READY |
| Full-task phase and completion integration | NOT_READY |
| Hardware validation | PENDING |
| Minimum motion / real rollout | NOT_EXECUTED / NOT_AUTHORIZED |

Tests executed: integration 12 PASS, core 100 PASS, no FAIL or SKIP in those suites. These are targeted suites, not proof that every nested historical research test was discovered. Readiness-suite regression results are recorded separately by the terminal output.

All new files are under `14_real_rollout_integration`; existing runtime and production safety config were not edited. Reused modules include SafetyPipeline, canonical action conversion, gripper state-decoupling, logger, FK and recorded rollout runner. Git status identifies this directory as newly added.

Physical command execution during this work: Robot 0; motion service/action 0; gripper 0; Home 0; trajectory 0; Hold/E-stop 0; real rollout 0. Mock service requests are not physical commands. No live ROS clients were initialized by tests.

The requested `REAL_ROLLOUT_SOFTWARE_READY` verdict is not supported: the software blockers listed in README must be completed first. Non-dry runner explicitly refuses motion even after approvals. This preserves safety rather than implying that a fresh checklist supplies a continuous watchdog.

After software completion, hardware-only validation will still require E-stop/protective stop, robot mode/servo, live endpoints, physically verified gripper polarity, measured or approved FK TCP, workspace limits, stop acknowledgement, explicit per-stage approval, minimum motion, short horizon, full rollout. OFT behavior review must precede authorizing model control.
