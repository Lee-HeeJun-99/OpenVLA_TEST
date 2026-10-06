# Subscriber-only Shadow validation

Implemented command-incapable components:

- `subscriber_only_shadow.py`: same-frame K=1/K=5 response validation, SHA-256, separated clock-domain fields, null executed/delivered action, error-to-`hold_required` logging.
- `ros_subscriber_only_node.py`: passive subscription specification only. No publisher, service client, action client, Home, gripper, Hold/E-stop, or trajectory capability.
- `shadow_runtime.yaml`: all capability flags false and exact Phase 11 contracts.

The Gate 0 polarity, chunk validation, stale/duplicate checks, NaN/Inf rejection, timeout/communication/logger failure semantics remain fail-closed. In Shadow, “hold” is a log request only; no Hold call exists.

Offline result: 13 tests passed after static capability scanning, safe/unsafe startup checks, K=1/K=5 logging, null execution fields, fsync, gripper semantics, chunk ordering/staleness, joint/workspace checks, and unresolved action-convention blockers.

Live subscriber-only run: **NOT_EXECUTED** because camera, `/dsr01/joint_states`, `/doosan/current_pose`, and model servers were absent. This avoids producing a misleading empty validation.

Live recovery added a strict read-only getter allowlist and name-based JointState canonicalizer. The latter accepts NaN effort as unsupported while requiring finite position/velocity and monotonic header timestamps. Eighteen offline tests pass. Live synchronized validation remains not executed because the controller/JointState path disappeared during the getter audit and TCP was never obtained.

## Offline contract integration follow-up — 2026-10-02

The command-free pipeline now connects JointState-derived FK, OpenVLA K=1/OFT
sequential K=5, translation ×1000, world rotvec composition, gripper hysteresis, safety
inspection and fsync logging. Actual Phase 11 predictions produced six audit records.
All executed/delivered actions are null and all `command_issued` fields are false.
Phase 12 tests: 33/33 passed. Live stationary validation is still separate and pending.
