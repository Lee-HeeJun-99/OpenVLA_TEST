# Phase 12 Pre-Rollout Readiness Report

## Final verdict

`BLOCKED_TECHNICAL_VALIDATION_FAILED`

This is not an authorization to move the robot.

## Current status

| Area | Status | Current evidence |
|---|---|---|
| JointState continuity | FAILS AFTER TCP GETTER | Fresh pre-getter 10 s passed at 100 Hz/1,001 samples/12.024 ms max gap; post-timeout 15.054 s received 0 samples |
| ZED | PASS observation-only | live node/topic and health signal present |
| Local operator/physical safety | BLOCKED | checklist remains unconfirmed |
| E-stop/protective-stop/servo/Hold ack | NOT_FOUND / REQUIRES OPERATOR | no verified current-state signal |
| Measured TCP, robot state/mode | NOT_FOUND | current_posx timed out once at 3.000606 s; state/mode were not called by stop rule |
| Measured gripper | NOT_FOUND | command/DO state is not physical feedback; no DI mapping |
| Model/input contract | PASS static | Phase 11 identities, instruction and preprocessing frozen |
| OFT health | PASS live | vision step28560, K=5, center crop, no proprio, RTX 3090 |
| OpenVLA health | NOT_EXECUTED | port 8766 not running while OFT occupies GPU |
| Translation | PASS offline | 1000 mm/m; empirical runtime 2800 rejected |
| Rotation | PASS offline | Doosan ZYZ degree + world/base rotvec matrix composition |
| Gripper polarity | PASS offline / BLOCKED integration | 0=open, 1=closed; existing runtime remains reversed |
| OFT K=5 | PASS offline / BLOCKED integration | sequential index 0→4; existing runtime remains first-only |
| Runtime safety supervisor | PASS offline | watchdog, step/rate/velocity/acceleration, stale/duplicate/NaN/failure paths |
| Logger | PASS offline | disk guard, partial marker, sequence, per-record fsync, atomic completion status |
| Stationary observation | NOT_EXECUTED | TCP/current safety prerequisites missing |
| Prediction-only Shadow | NOT_EXECUTED | observation and OpenVLA health prerequisites missing |
| Offline reference comparison | PASS existing offline only | Phase 10/11 artifacts; no physical replay |

Tests: 31/31 passed (27 unittest plus four JointState readiness-gate checks).

## Remaining mandatory actions

1. Diagnose why `get_current_posx` times out and feedback stops immediately afterwards; do not retry until driver callback/monitor channel behavior is corrected.
2. Local operator confirms explicit protective-stop, servo/mode values and numeric workspace/speed/acceleration limits (general checklist attestation is recorded).
3. Provide stable read-only measured TCP and current robot/safety state, including Hold acknowledgement.
4. Identify measured gripper feedback wiring, or explicitly redesign the protocol around a documented open-loop limitation before any motion approval.
5. Integrate the corrected contracts through a separately reviewed command boundary; do not use current `runtime_oft.yaml`.
6. Stop OFT normally, health-check OpenVLA, then run models sequentially.
7. Pass 10–30 s command-free stationary observation and per-model prediction-only Shadow with executed/delivered command fields always null.
8. Re-audit command capabilities and obtain a separate explicit motion approval.

## Execution counts

- Robot command: 0
- Motion service/action: 0
- Gripper command: 0
- Home: 0
- Trajectory: 0
- Hold/E-stop call: 0
- AI action publish: 0
- Closed-loop: 0
- New model inference requests in this step: 0

## Latest readiness addendum — open-loop gripper policy

The physical checklist and numerical workspace/20-profile/30-second limits were
explicitly operator-confirmed. The no-measured-feedback gripper policy was also
approved and implemented command-disabled, with 62/62 tests passing. Measured
gripper feedback itself remains unavailable and is not fabricated.

The remaining gate is actual motion-capable runtime integration followed by a
separate explicit minimum-motion dry-run approval. No motion approval is implied by
this addendum and all command counts remain zero.
