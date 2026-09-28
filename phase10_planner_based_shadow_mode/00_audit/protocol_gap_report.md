# Original Protocol vs Current Evidence

Status: `COMPLETED_OFFLINE_ONLY`

| Requirement | Phase 8 | Phase 9 | Phase 10 current state |
|---|---|---|---|
| Planner controls Real robot | No | No Real run | Interface specified; execution blocked pending approval |
| Same recorded input for both models | OFT only | OFT policy rollout | Dual recorded-input adapters implemented |
| OpenVLA + OFT | OFT only | OFT only | Both adapters implemented, not model-executed here |
| Planner-relative model error | No | Failed policy path, not GT | Canonicalizer/metrics implemented; data missing |
| Measured Real state | No, planned pose | No Real | Missing and required |
| Sim Planner reference | Joint replay imagery, not complete Phase 10 pairing | Failed OFT rollout | Existing Sim planner trajectory must be linked/collected separately |
| Online latency | Partial server evidence | Partial | Recorded-input timing supported; online test not executed |
| Real baseline 30 trials | No | No | `NOT_EXECUTED` |
| Environment pilot trials | Offline episodes 4/8/9/10 | No | Protocol/config only |
| Prototype 30 trials | No | No | Protocol only; blocked safety review |
| Final 60 rollouts | No | No | Protocol only; blocked safety review |

## Blocking gaps before Real Shadow Mode

1. Freeze and review the external Real planner/controller commit; it is currently dirty/untracked.
2. Add measured TCP/joint/gripper feedback with source timestamps; do not substitute planned pose.
3. Integrate logger failure into the Real planner hold path and verify it on hardware-in-the-loop or a safe mock.
4. Verify camera/state/planner clock domains and a maximum synchronization tolerance.
5. Confirm actual Home joint vector, gripper Home state, workspace/joint/velocity limits and E-stop topic on the target machine.
6. Confirm both model servers/checkpoints and exact training instruction string. Existing training data uses `Pick up the orange cube.` while bundle validation normalizes red/orange prim naming differently.
7. Run model adapters against the same recorded trial and capture raw, de-normalized and feature outputs.
8. Establish a valid Sim Planner reference; Phase 9 policy failures are excluded.
9. Complete an operator-reviewed planner-only dry run before any Real motion.

Existing Phase 8 Real logs expose `image_timestamp` and `source_timestamp` from
different apparent clock domains (epoch-like versus monotonic-like). They must
not be subtracted directly. Phase 10 alignment computes timestamp skew only
after explicit clock-domain equality; otherwise it reports frame association
and callback ages as lower-confidence evidence.
