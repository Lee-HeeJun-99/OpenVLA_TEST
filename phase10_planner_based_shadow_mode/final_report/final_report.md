# Phase 10 Initial Implementation Report

## Outcome

The requested pre-Real scope is implemented and audited. The repository now contains an isolated Planner-based recorded-input Shadow Mode pipeline with canonical action alignment, integrated logging, structural AI publish blocking, OpenVLA K=1 and OFT K=5 adapters, timestamp alignment, planner-relative metric primitives, configs, protocols and status tracking.

No Real robot command, trajectory replay, gripper command or closed-loop action was executed.

## Reused assets

- Phase 8 validation, observation, representation, component/chunk, phase and sensitive-projection code as extension references.
- `sim2real_analysis/common/feature_capture.py` hook/NPZ utilities.
- Bundle model server/client and offline HTTP inference pattern.
- Sim Lula planner and recorder.
- `field/control_contract.json` for 5 Hz / OFT 1 Hz K=5 behavior.
- Existing episode 4 only as an offline pipeline fixture, preserving `planned_commanded_pose` provenance.

## New implementation

- `action_canonicalizer.py`: explicit units/frame/horizon/gripper semantics.
- `openvla_adapter.py` and `oft_adapter.py`: recorded-input prediction-only adapters.
- `feature_hook_registry.py`: semantic hook labels and cross-model comparison guard.
- `shadow_mode_runner.py`: identical saved inputs, OpenVLA every frame, OFT every fifth frame.
- `integrated_logger.py` and JSON schema.
- `safety_gate.py`: failure classification and unconditional AI publish block.
- recorded planner interface, timestamp aligner and planner-relative metric primitives.
- experiment configs, Real protocols, audit and readiness reports.

## Phase 9 block

All five inspected Phase 9 rollouts failed: one policy timeout and four consecutive-IK failures. They and the Real replay exporter are classified `DO_NOT_USE_FOR_REAL`. Nothing in Phase 10 imports or exports them.

## Offline verification

Seven checks passed across the offline test suite:

1. canonical units and horizon integration;
2. structural AI publish blocking;
3. nonfatal shadow inference timeout behavior;
4. planner timeout hold behavior;
5. two-frame recorded-input logging with OpenVLA K=1, OFT K=5 and null executed actions.
6. planner-only static rate/workspace/segment validation with zero command publishing.
7. unknown/different clock domains are not subtracted as synchronized timestamps.

This is pipeline evidence, not model-performance evidence.

## Readiness by component

| Component | Status |
|---|---|
| Audit/reuse plan/protocol gap | `COMPLETED_OFFLINE_ONLY` |
| Canonical action alignment | `COMPLETED_OFFLINE_ONLY` |
| Logger and safety gate | `COMPLETED_OFFLINE_ONLY` |
| Recorded planner loader | `COMPLETED_OFFLINE_ONLY` |
| Live Planner interface | `BLOCKED_SAFETY_REVIEW` |
| OpenVLA/OFT action adapters | `COMPLETED_OFFLINE_ONLY` |
| OpenVLA/OFT real feature run | `BLOCKED_MISSING_DATA` |
| Sim Planner paired reference | `BLOCKED_MISSING_DATA` |
| Real baseline 30 trials | `NOT_EXECUTED` |
| Environment pilots | `NOT_EXECUTED` |
| Prototype/final rollout | `NOT_EXECUTED` |

## User confirmations required before Real Shadow Mode

- Approved Real controller/planner revision and launch topology.
- Exact topics/services for measured pose, joint/gripper state, hold acknowledgement and E-stop.
- Approved Home joint vector and workspace/joint/velocity limits.
- Camera/state clock synchronization tolerance.
- Logger failure policy and whether camera/state sync failure always holds.
- OpenVLA/OFT server URLs, checkpoint identity and instruction normalization.
- Operator presence, hardware E-stop test and planner-only dry-run approval.

## Remaining blockers

The clean repository lacks a reviewed live Real planner/logger integration and measured feedback capture. The external ROS workspace contains candidate code but is dirty/untracked. No valid Phase 10 Sim Planner paired reference or dual-model Phase 10 predictions exist yet. Consequently no observation, representation, planner-relative action, environment, prototype or success-rate conclusion is reported.
