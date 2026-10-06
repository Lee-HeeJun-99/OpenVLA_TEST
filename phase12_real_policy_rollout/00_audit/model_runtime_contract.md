# Gate 1 model and action contract

## Phase 11 matched contract update

- OpenVLA: `vanilla_s1_balanced_step8130`, `openvla_token`, K=1, dataset key `a0509_sim_cube_pick`, crop-bottom 0.0.
- OFT: `runtime_state/oft_mixed480_step28560_merged`, `oftplus_h5_vision`, K=5, center crop enabled, proprio disabled.
- Instruction at the experiment boundary: `Pick up the orange cube.`; recorded responses normalize it to `pick up the orange cube`.
- Phase 11 contains actual recorded OpenVLA K=1 and OFT K=5 predictions under these identities.
- `config/runtime_oft.yaml` remains ineligible because it selects proprio step6000 and incompatible instruction, preprocessing, rate, gripper and chunk contracts.

No model server was started and no model was connected to a robot command path in this update.

Live recheck on 2026-10-02: OFT port 8765 reports ready with checkpoint `runtime_state/oft_mixed480_step28560_merged`, variant `oftplus_h5_vision`, K=5, action dimension 7, center crop enabled, no proprio, continuous bounded gripper, RTX 3090 and peak allocation 15.75 GiB. OpenVLA port 8766 is not running. No inference request was sent and no robot runtime node was started.

Neither prediction server was running: GET health probes at `127.0.0.1:8766`, `:8765`, and legacy `:8000` all refused connection. No server was started because live ROS observations needed for this gate were absent.

Required contract retained from Phase 11:

- OpenVLA: `vanilla_s1_balanced_step8130`, `openvla_token`, K=1, dataset `a0509_sim_cube_pick`, crop-bottom 0.0.
- OFT: `oft_mixed480_step28560_merged`, `oftplus_h5_vision`, K=5, no proprio, training center crop.
- Instruction: exactly `Pick up the orange cube.`
- The old real runtime `oftplus_h5_proprio` step 6000 contract is incompatible and is not mixed with these results.

Pre-rollout static recheck confirms existing runtime configs still use `pick up the cube`, disable the required OFT center crop, and point at the incompatible proprio runtime. GPU health was not verifiable (`nvidia-smi` driver communication failure). Therefore live model runtime status remains `NOT_EXECUTED/BLOCKED`; known Phase 11 checkpoint metadata is not equivalent to a live healthcheck.

Current health GETs to ports 8766 and 8765 were connection-refused. Config hashes: runtime `b161b4f8...f5`, runtime OFT `a3669ef8...f5`. Dataset-statistics hashes: OpenVLA `61c9543a...fb8`, OFT `233125ca...fae`. Full checkpoint payload hashes were not computed and model processes were not started.

## Translation scale

Both checkpoint statistics name translation as world-frame metres. OpenVLA q99 is approximately `[0.0151, 0.00947, 0.0209] m`; OFT q99 is `[0.01477, 0.01084, 0.02096] m`. The physical conversion is therefore 1000 mm/m. Runtime `2800` combines that conversion with an undocumented 2.8x empirical gain; tuning notes also record near-8 mm target jumps and a later collision. Phase 12 rejects that gain. Status: `VERIFIED_UNIT_CONVERSION_1000_MM_PER_M`; runtime 2800 is not approved.

## Rotation

Dataset outputs are world/base-frame relative rotation vectors in radians: the collector computes `q_next * inverse(q_previous)` and replay applies the delta on the left. `RobotStateRt.msg` and `DRFS.h` explicitly define Doosan A/B/C as Euler ZYZ degrees in the base frame. `action_contract.py` composes `R_delta @ R_current` and converts the result to an equivalent ZYZ triple; matrix-based identity and axis tests pass. Direct component-wise addition remains prohibited. Status: `VERIFIED_OFFLINE_NOT_RUNTIME_INTEGRATED`.
