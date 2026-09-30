# Phase 10 Initial Implementation Report

## Outcome

The requested pre-Real scope is implemented and audited. The repository now contains an isolated Planner-based recorded-input Shadow Mode pipeline with canonical action alignment, integrated logging, structural AI publish blocking, OpenVLA K=1 and OFT K=5 adapters, timestamp alignment, planner-relative metric primitives, configs, protocols and status tracking.

No Real robot command, trajectory replay, gripper command or closed-loop action was executed.

The Real runtime at Git HEAD `86eaa9632d651eb907332334d02f32c1461850d7` was additionally audited from its dirty working tree. Subscriber-independent observer/recorder wrappers were added without ROS imports, publishers, service clients, or action clients. ROS graph and hardware were not contacted.

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

Nine unit tests passed across the offline test suite, including these checks:

1. canonical units and horizon integration;
2. structural AI publish blocking;
3. nonfatal shadow inference timeout behavior;
4. planner timeout hold behavior;
5. two-frame recorded-input logging with OpenVLA K=1, OFT K=5 and null executed actions.
6. planner-only static rate/workspace/segment validation with zero command publishing.
7. unknown/different clock domains are not subtracted as synchronized timestamps.
8. robot, gripper, home, trajectory, hold, E-stop and AI-publish paths are all rejected;
9. synthetic runtime schema/synchronization and observation-only executed-command handling.

This is pipeline evidence, not model-performance evidence.

## Readiness by component

| Component | Status |
|---|---|
| Audit/reuse plan/protocol gap | `COMPLETED_OFFLINE_ONLY` |
| Canonical action alignment | `COMPLETED_OFFLINE_ONLY` |
| Logger and safety gate | `COMPLETED_OFFLINE_ONLY` |
| Recorded planner loader | `COMPLETED_OFFLINE_ONLY` |
| Live Planner interface | `BLOCKED_SAFETY_REVIEW` |
| Real runtime static audit and observer wrapper | `COMPLETED_OFFLINE_ONLY` |
| Real ROS subscriber binding | `NOT_EXECUTED` |
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

Protocol stop state: `BLOCKED_REQUIRES_LOCAL_OPERATOR_AND_HARDWARE`. A dedicated planner action, measured gripper feedback, timestamped EE/executed-command observations, Doosan rotation/reference semantics, joint limits, hold acknowledgement, and physical E-stop state remain unresolved.

## Real collection/reference integration update

The Real collection workspace shares Git HEAD `86eaa9632d651eb907332334d02f32c1461850d7`. Its working tree was already dirty: `single_robot_simple.py` is modified and `collect_episode001_from_align_home.py` is untracked. Neither file nor the Real runtime was edited.

The collection route is classified `SCRIPTED_REFERENCE_TRAJECTORY`: fixed HOME/start plus alignment, descent, close and lift waypoints are calculated before motion and issued as absolute base-frame MoveJ/MoveLine commands. There is no obstacle/IK-based online replanning. The recorder interpolates the commanded task-space segment over its actual duration at 5 Hz. Consequently the comparison target is called a Reference Command, not Planner GT or measured ground truth.

Episode 4 produced 45 extracted canonical Reference Commands. All retain `pose_source=planned_commanded_pose`, `measured_feedback_available=false`, and `measured_action=null`. The extracted vectors match the existing `steps_with_actions.jsonl` exactly (`max_abs_difference=0.0`).

The collector treats Doosan degree triples as RPY, converts them to quaternions, and computes a relative rotation vector. Phase 10 reproduces this dataset convention. The inspected local driver definitions did not independently establish the controller-native Euler convention, so physical rotation interpretation remains `UNRESOLVED_ACTION_CONVENTION`; raw degree triples are never copied into rotvec fields.

## Model configuration and actual-model status

- OpenVLA: `models/vanilla_s1_balanced_step8130`, variant `openvla_token`, H=1, dataset key `a0509_sim_cube_pick`, crop-bottom fraction 0.0.
- Phase 8 OFT: `runtime_state/oft_mixed480_step28560_merged`, `oftplus_h5_vision`, K=5, no proprio, server default center crop enabled unless explicitly disabled.
- Current Real ROS OFT config: different `oftplus_h5_proprio` checkpoint at step 6000, proprio required, training preprocess disabled and runtime center crop disabled.

These policies/configurations are not merged. Both audited HTTP servers return predictions only and have no robot publishers. The local operator subsequently completed prediction-only runs: OpenVLA returned 45 H=1 predictions and OFT returned nine K=5 chunks covering 45 steps. Both had zero inference errors, zero fixture records, zero AI executed actions and zero robot-delivered commands.

Reference-command-relative results for this single episode: OpenVLA translation L1 mean 0.017200 m, L2 RMSE 0.014574 m, gripper accuracy 0.9333; OFT translation L1 mean 0.020589 m, L2 RMSE 0.017581 m, gripper accuracy 0.6444. Under verified dataset semantics (`0=open`, `1=closed`) and threshold 0.5, Reference close occurs at step 26, OpenVLA at step 23, and expanded OFT K=5 at step 1. These are offline agreement results, not closed-loop success evidence or a statistically sufficient model ranking.

## Episode 4 OFT gripper diagnosis

The final classification is `H. MULTIPLE_CONTRIBUTING_FACTORS`. OFT's raw server response is already dataset-statistics-de-normalized, while the gripper dimension is masked out of affine de-normalization and comes from a bounded sigmoid head. Phase 10 correctly preserves it as continuous closedness. The step-1 crossing is a genuine early high-closedness prediction under that model/dataset convention, and the K=5 expansion mapping is complete and free of duplicates, gaps and off-by-one shifts.

However, the current Real runtime documents and processes the same scalar with the opposite polarity: `<=0.3` means close, `>=0.7` means open, the middle band holds prior state, repeated states are suppressed, and only chunk index 0 is selected. Applying those runtime-equivalent semantics gives first close step 0 for OpenVLA and step 10 for OFT, rather than steps 23 and 1. Therefore step 1 must not be described as the Real runtime command event. The evidence does not support de-normalization, Phase 10 canonicalization, or offline chunk-alignment errors.

At dataset semantics/threshold 0.5, OpenVLA has TP/TN/FP/FN = 19/23/3/0 and F1 0.9268; OFT has 19/10/16/0 and F1 0.7037. OFT's 16 false-close steps concentrate in alignment (12) and descent (3), plus one hold step. False positives increase with chunk index: 1, 2, 4, 4 and 5 for indices 0–4.

OpenVLA latency p50/p95/p99/max was 0.3153/0.3315/0.4511/0.5435 s including warm-up. OFT was 0.2143/0.7845/1.0819/1.1563 s; excluding its first request, p95/max were 0.2233/0.2267 s. This single episode identifies one OFT warm-up request above the 1 s budget and does not establish general latency guarantees.

## Timestamp and Shadow launch

Camera/JointState source time, unstamped EE receive time, Reference Command time and model monotonic latency are kept in named clock domains. Cross-domain subtraction is forbidden. The new `shadow_runtime.launch.py` validates fail-closed configuration and launches zero nodes; it is an interface draft, not an executed ROS system. All robot, gripper, home, trajectory, motion, stop, E-stop and AI-publish switches must remain false.

Offline verification now passes 18 tests, including recorded Episode 4 extraction, exact source-action equivalence, collector rotation conversion, K=1/K=5 logging, timestamp policy, null AI execution, logger fsync, zero-node launch inspection, rejection of every unsafe configuration switch, and copied runtime gripper polarity/threshold/hysteresis/chunk/repeated-command semantics. Robot commands and state-changing ROS calls remain zero.

Final protocol state remains: Real Shadow Mode `NOT_EXECUTED`; Real closed-loop rollout `NOT_EXECUTED`; overall `BLOCKED_REQUIRES_LOCAL_OPERATOR_AND_HARDWARE`.
