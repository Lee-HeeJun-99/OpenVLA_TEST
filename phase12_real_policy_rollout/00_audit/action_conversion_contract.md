# Phase 12 input/output contract audit

This is a static/offline audit. No value in this document authorizes a robot command.

## Decision table

| Contract | Evidence-based result | Readiness |
|---|---|---|
| Gripper model semantics | Dataset/training metadata and OFT head use continuous closedness: `0=open`, `1=closed`. Gripper is excluded from OFT affine de-normalization. | `VERIFIED_MODEL_SIDE` |
| Existing runtime gripper | `action_adapter_node._resolve_gripper_state` interprets high as open and low as close. This is the reverse of the model contract. Runtime files were not changed. | `POLARITY_MISMATCH_CONFIRMED` |
| Phase 12 gripper boundary | `ClosednessHysteresis`: `<=0.3=open`, `>=0.7=closed`, intermediate values hold the prior state, duplicate commands are suppressed. | `PASS_OFFLINE_NOT_INTEGRATED` |
| Measured gripper feedback | `/vla/gripper_open` is a command, not feedback. Doosan digital-input getters/state fields exist, but no source/config maps an input bit to gripper-open or gripper-closed sensors. | `NOT_FOUND` |
| Translation model action | Relative translation in metres after checkpoint-specific de-normalization. | `VERIFIED` |
| Physical translation conversion | Exactly `1000 mm/m`. `action_contract.translation_m_to_mm` rejects any extra gain. | `VERIFIED_OFFLINE` |
| Runtime `2800` | Tuning notes describe 1000–2500, then 2800 for speed. It produced target jumps near 8 mm and was followed by one Joint Axis2 collision report. It is not derived from dataset statistics. | `REJECTED_AS_UNIT_CONVERSION` |
| Dataset rotation | Relative rotation vector in radians. Collector computes `q_next * inverse(q_previous)`; replay reconstructs with a world/base-frame left composition. | `VERIFIED_DATASET_SIDE` |
| Doosan pose rotation | Driver messages and `DRFS.h` explicitly document A/B/C as Euler ZYZ in degrees, relative to base coordinates. | `VERIFIED_REPRESENTATION` |
| Rotation conversion candidate | `R_next = Exp(rotvec) @ R_current`, then matrix to Euler ZYZ degrees. Identity and 1°/5° axis tests compare matrices, avoiding Euler non-uniqueness. | `PASS_OFFLINE_NOT_INTEGRATED` |
| Existing runtime rotation | Multiplies rotvec components by rad→deg and adds them directly to A/B/C. This is prohibited. | `FAIL` |
| OFT training horizon | Dataset loader takes actions `t:t+5`; training config records 5 Hz data, K=5, five actions per inference. | `VERIFIED_SEQUENTIAL_TEMPORAL_ORDER` |
| Existing runtime OFT | Receives `(5,7)` then returns only `action_array[0]`. | `FIRST_ONLY_MISMATCH_CONFIRMED` |
| Phase 12 OFT policy | `sequential_k5`: one new chunk per 1 Hz inference, consume indices 0→4 at 5 Hz; duplicate/stale/underrun are rejected. `first_only` remains diagnostic only. | `PASS_OFFLINE_NOT_INTEGRATED` |

## Model/input contract matched to Phase 11

| Field | OpenVLA | OFT |
|---|---|---|
| Checkpoint | `vanilla_s1_balanced_step8130` | `runtime_state/oft_mixed480_step28560_merged` |
| Variant | `openvla_token` | `oftplus_h5_vision` |
| Horizon | K=1 | K=5 |
| Proprio | no | no |
| Crop | bottom fraction 0.0 | center crop enabled |
| Instruction | `Pick up the orange cube.`; server normalization yields `pick up the orange cube` | same |
| Dataset key | `a0509_sim_cube_pick` | `a0509_sim_cube_pick` |

The installed `runtime_oft.yaml` is not Phase-11-equivalent: it points to a proprio step-6000 policy, disables training preprocessing/center crop, uses a different instruction, runs at 0.2 seconds, reverses gripper polarity, applies direct A/B/C addition, and consumes K=5 index 0 only. It must not be used for the proposed Phase 12 comparison.

## Remaining rollout blockers

- No measured gripper feedback signal or validated digital-input wiring map.
- The corrected boundary and sequential K=5 queue are not connected to the preserved Real runtime.
- The ZYZ conversion is verified only offline; live measured TCP has not been obtained and no command test is authorized.
- Physical workspace, velocity limits, hardware E-stop/protective stop, Hold acknowledgement and local operator checklist remain unresolved.

Therefore `motion_readiness=false` remains mandatory.
