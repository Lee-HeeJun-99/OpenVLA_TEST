# Stationary live Shadow smoke test

Status: `NOT_EXECUTED`.

Prerequisites failed: JointState-only bag showed driver/controller feedback stalls; measured TCP and safety-state minimum were unavailable; operator checklist was unconfirmed; model servers/GPU health were unavailable. OpenVLA/OFT were not started. AI executed action, robot-delivered command and command-issued records remain null/null/false by contract.

The statement above is retained as the original Gate 1 result. The post-relaunch
follow-up below supersedes it for the current minimal-observer session.

## OpenVLA post-relaunch smoke — 2026-10-02

Status: `OPENVLA_PREDICTION_ONLY_PASS_OFT_PENDING`

- Server contract: `vanilla_s1_balanced_step8130`, `openvla_token`, K=1,
  action dimension 7, dataset `a0509_sim_cube_pick`, crop-bottom 0.0, RTX 3090.
- Duration/records: 15 seconds / 15 predictions.
- Valid/errors: 15/0.
- Unique input image hashes: 15/15.
- K=1 action-shape validation: PASS.
- Latency mean/median/max: 0.407/0.377/0.824 seconds.
- `executed_action=null`, `robot_delivered_command=null`, and
  `command_issued=false` for every record.
- Static recorder audit found no publisher, service/action client, ActionAdapter,
  DoosanBridge, motion, gripper, Home, Hold/E-stop, or realtime-write capability.

This is prediction-only stationary observation, not a rollout and not evidence of
closed-loop control performance. OFT must be run separately after OpenVLA is stopped
to avoid concurrent model GPU loading.

Artifacts:

- `03_shadow_mode/live_prediction_only_shadow.py`
- `03_shadow_mode/openvla_stationary_shadow_20261002.jsonl`
- `03_shadow_mode/openvla_stationary_shadow_20261002.jsonl.status.json`

## OFT post-relaunch smoke — 2026-10-02

Status: `OFT_PREDICTION_ONLY_PASS_WITH_WARMUP_BUDGET_EXCEEDANCE`

- Server contract: `oft_mixed480_step28560_merged`, `oftplus_h5_vision`, K=5,
  action dimension 7, no proprio, center crop enabled, continuous bounded gripper,
  RTX 3090.
- Duration/records: configured 15-second observation / 15 predictions.
- Valid/errors: 15/0.
- Unique input image hashes: 15/15.
- K=5 action-shape and chunk-index completeness: PASS (five 7-D actions per request).
- Latency mean/median/p95/max: 0.295/0.223/0.231/1.293 seconds.
- Requests exceeding the 1 Hz inference budget: 1/15. This was the maximum/first
  request and is retained as a warm-up/budget-risk observation, not removed.
- `executed_action=null`, `robot_delivered_command=null`, and
  `command_issued=false` for every record.

Both model-specific stationary prediction paths have now passed independently. This
does not approve motion: hardware safety confirmations, operator-approved physical
limits, measured gripper feedback, and runtime command-path integration remain
separate blockers. No model output was delivered to a robot command interface.

Artifacts:

- `03_shadow_mode/oft_stationary_shadow_20261002.jsonl`
- `03_shadow_mode/oft_stationary_shadow_20261002.jsonl.status.json`
