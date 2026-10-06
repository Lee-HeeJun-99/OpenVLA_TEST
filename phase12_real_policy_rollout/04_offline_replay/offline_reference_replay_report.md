# Offline recorded-action contract integration — 2026-10-02

Status: `COMPLETED_OFFLINE_COMMAND_FREE`

This is not a robot replay. It consumes actual Phase 11 recorded-input predictions and
checks the Phase 12 observation/action/safety/logging chain without ROS or commands.

Inputs: OpenVLA baseline Episode 1 step 8130 K=1; OFT `oft_run2` baseline Episode 1
vision step 28560 K=5; A0509 URDF; instruction `Pick up the orange cube.`

Verified chain:

```text
recorded joints → canonical order → link_6 FK proxy → canonical model action
→ m-to-mm ×1000 → world rotvec composition → Doosan ZYZ candidate
→ gripper 0=open/1=closed hysteresis → sequential K=5 queue
→ provisional safety inspection → fsync JSONL
```

Results:

- OpenVLA records: 1
- OFT records: 5; indices `[0,1,2,3,4]`
- executed/AI-executed/delivered actions: all null
- `command_issued`: false for all records
- legacy translation gain 2800: unused
- logger status: complete
- Phase 12 tests: 33/33 passed

Limits used here are provisional offline test values, not operator-approved physical
limits. FK is not independently measured TCP, measured gripper feedback remains absent,
and no result authorizes motion or demonstrates rollout success.

Artifacts: `episode1_contract_audit_20261002.jsonl`, its status and summary JSON, and
`03_shadow_mode/offline_contract_integration.py`.

## Operator-selected 20 profile follow-up

Status: `COMPLETED_OFFLINE_COMMAND_FREE_20_PROFILE`

The operator-selected candidate limits were applied to the same actual Phase 11
OpenVLA/OFT predictions without creating or delivering commands:

- translation velocity/acceleration: 20 mm/s and 20 mm/s²;
- rotation velocity/acceleration: 20 deg/s and 20 deg/s²;
- joint velocity/acceleration candidate: 20 deg/s and 20 deg/s²;
- 5 Hz velocity-derived step ceilings: 4 mm and 4 degrees.

Of six candidate actions, four passed and two OFT chunk actions were rejected with
`translation_acceleration_limit`. The rejection is retained; limits were not relaxed
to make the predictions pass. This indicates that a motion implementation would need
an explicitly designed rate/acceleration limiter or smoothing policy before command
delivery. Merely clipping each independent action is not sufficient to prove safe
dynamics.

All six records still contain null executed/delivered actions and
`command_issued=false`. Phase 12 tests pass 34/34.

Artifacts:

- `episode1_contract_audit_vel20_20261002.jsonl`
- `episode1_contract_audit_vel20_20261002.jsonl.status.json`
- `episode1_contract_audit_vel20_20261002_summary.json`

## Stateful limiter follow-up

Status: `PASS_OFFLINE_20_PROFILE_AFTER_LIMITING`

A stateful 5 Hz limiter now converts each raw canonical delta into a velocity target,
clips translation/rotation speed, and limits the change from the previous velocity to
20 mm/s² and 20 deg/s². It starts from zero velocity, handles direction reversal by
ramping, and leaves gripper closedness unchanged. Raw and limited actions are logged
separately.

The first end-to-end run exposed two exact-boundary floating-point rejections. The
supervisor was updated with a numerical-only tolerance (`max(1e-12, limit*1e-9)`),
without changing any physical limit. The repeated audit then produced:

- accepted/rejected: 6/0;
- limiter-modified records: 4/6;
- maximum translation modification: 2.567 mm;
- maximum rotation modification: 0.015819 rad (about 0.906 degrees);
- executed/delivered actions: all null;
- `command_issued`: false for every record;
- Phase 12 tests: 38/38 passed.

This validates the algorithm offline only. Integration into a command-capable runtime,
physical workspace approval and stop/feedback requirements are still blocked.

Final artifacts:

- `episode1_contract_audit_vel20_limited_v2_20261002.jsonl`
- `episode1_contract_audit_vel20_limited_v2_20261002.jsonl.status.json`
- `episode1_contract_audit_vel20_limited_v2_20261002_summary.json`

## Workspace candidate follow-up

The ten Real reference episode files contain 452 planned poses with envelope
X 305.090–523.100 mm, Y -323.005–355.500 mm and Z 297.824–723.678 mm. Every pose is
labelled `planned_actual_duration`, not measured TCP. An outward approximately 30 mm
candidate box was created: X 275–554 mm, Y -355–386 mm, Z 267–754 mm.

All 452 planned poses and all six rate-limited model candidates fall inside the box.
Tests pass 41/41. The box remains `DATA_DERIVED_CANDIDATE_NOT_OPERATOR_APPROVED`;
it does not establish collision clearance or reachability throughout the volume.

Artifact: `episode1_contract_audit_vel20_workspace_v3_20261002_summary.json`.
