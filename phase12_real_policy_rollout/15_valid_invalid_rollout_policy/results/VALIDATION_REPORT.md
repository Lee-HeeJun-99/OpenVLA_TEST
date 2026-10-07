# INVALID rollout policy validation

Date: 2026-10-07. Baseline branch: `lhj-research`.
Baseline HEAD: `8e9c3db20ba7714339ec69123cad7983c9bce8d0`.

## Result

Policy implementation: PASS. Terminal trial statuses are exclusively SUCCESS,
FAILURE, INVALID. INVALID sets `runtime_valid=false`, `task_success=null`, inhibits
new commands, preserves evidence, and is excluded from the task-success denominator.

Unit/integration/regression: **212 PASS / 0 FAIL / 0 SKIP**:

- Phase 12 core: 100
- Existing real integration: 66
- Analysis directories 07–11: 22
- Rollout readiness 13: 4
- New policy 15: 20

Isolated generated ROS service/DDS tests: **4 PASS / 0 FAIL / 0 SKIP**.
Fake live process scenarios: **13 PASS** (OpenVLA/OFT dry-run; camera, joints,
TCP, state, protective stop, servo, mode, authority, health mismatch, disconnect,
logger failure). These are fake hardware/model results, not real hardware validation.
The final regression and DDS runs followed the final implementation edits; fake
live process scenarios ran during implementation. A later edit added pre-trial
startup journaling and avoided redundant post-terminal artifact writes.

Existing gimbal-lock warnings remain warnings, not test failures.
`git diff --check`: PASS.

## Executed batch dry-run

`recorded_dry_run_02/batch_summary.json` is the final synthetic fixture run:

- Attempts: 7; VALID: 3; INVALID: 4
- SUCCESS: 2; FAILURE: 1
- Conditional fixture task-success rate: 2/3
- Fixture invalid rate: 4/7
- INVALID categories: JointState, camera, safety rejection, model input (one each)
- Target reached: 3 valid trials; max attempts: 7
- Physical commands: 0; transport calls: 0

These inputs are **synthetic**, not recorded checkpoint predictions or measured
robot task outcomes. Their ratios only validate aggregation behavior. The earlier
`recorded_dry_run_01` artifacts are retained, not overwritten.

## Changes and remaining boundaries

New: classifier, bounded batch summary/runner, fixtures, policy tests/documentation,
and preserved dry-run trial directories. Existing files changed: live observation,
runner, command sink, fake DDS and fake live process tests.

Runtime faults are distinct from task failure. Missing verified task-outcome
evidence is INVALID_TASK_OUTCOME_UNVERIFIED, never an ACK-based task success.
Prediction-only/fixture scope is excluded from the research task-performance groups.

New-trial readiness requires new fresh sample and 10-second warm-up; no failed
trial window reset. Live rearm retains the same node/subscribers. The physical
batch path is explicitly blocked pending validated recovery and operator reset.
Startup/warm-up evidence is retained separately from trial validity.

Production safety configuration: unchanged. JointState 100 ms / latest age 500 ms,
camera 0.5 s, and raw translation 4 mm limits are not relaxed.
No real driver was launched/restarted and no real motion/gripper/hold/stop was called.

Next allowed physical stage: **BLOCKED**. Existing current TCP/tool, hardware-state
and manual-safety gates remain independent requirements; this policy implementation
does not establish their PASS or authorize motion.
