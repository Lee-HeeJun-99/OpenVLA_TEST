# Phase 10 Reuse Audit

Status: `COMPLETED_OFFLINE_ONLY`

## Scope and repository state

- Audited local branch: `lhj-research`, commit `deb9e06050b74650feda64cdcb2305a22c861071`.
- Phase 8 and Phase 9 were read only; no existing artifact was deleted or overwritten.
- Real ROS code exists outside this repository under `/home/ubuntu/robot_ws/src`. That workspace contains modified and untracked files, so it is classified `REFERENCE_ONLY` and was not executed.
- No Real robot command, replay, enable topic, gripper command, or closed-loop inference was issued.

## Existing capability inventory

| Capability | Evidence | Classification | Finding |
|---|---|---|---|
| Sim motion planner | `../isaac_sim/a0509_control/lula_planner.py` | `REUSE_AS_IS` | Lula IK/RRT and linear path validation exist. |
| Sim episode recorder | `../isaac_sim/a0509_control/episode_recorder.py` | `REUSE_WITH_EXTENSION` | Records images/state/actions but not dual-model shadow fields. |
| Real motion/collection | external `collect_episode001_from_align_home.py`, `single_robot_simple.py` | `REFERENCE_ONLY` | Has Home, MoveJ/MoveL, image/state recording and explicit `--execute`; outside clean repository. |
| Trajectory replay | external `dataset_episode_replay_node.py` | `REFERENCE_ONLY` | Defaults to dry-run and has start/workspace checks, but can publish when armed. Never imported into Phase 10. |
| Robot state/camera/gripper | external ROS runtime | `REFERENCE_ONLY` | Topics and adapters exist; measured-state provenance is not present in Phase 8 data. |
| OpenVLA inference | external ROS node plus bundle preflight evidence | `REUSE_WITH_EXTENSION` | K=1 server contract verified previously; recorded-input adapter added here. |
| OFT inference | bundle server/client and Phase 8 features | `REUSE_WITH_EXTENSION` | `oftplus_h5_vision`, K=5, 1 Hz/5 Hz contract; adapter added here. |
| De-normalization | model server response | `REUSE_AS_IS` at server boundary | Server returns physical action values. Raw head output and denormalized output still must be logged separately. |
| Gripper post-processing | external action adapter/config | `REFERENCE_ONLY` | Threshold/hysteresis differs by runtime; Phase 10 schema keeps raw/denormalized/postprocessed fields separate. |
| Action chunk execution | Sim validation/external OFT runtime | `REFERENCE_ONLY` | Not needed for recorded-input shadow; no AI execution allowed. |
| Timeout/hold/e-stop | external Doosan bridge/replay | `REFERENCE_ONLY` | Several guards exist, but a unified planner/logger/camera/state safety review is incomplete. |
| Feature hooks | `../sim2real_analysis/common/feature_capture.py` | `REUSE_AS_IS` | Generic module forward hooks and NPZ serialization. |
| Phase 8 metrics/plots | Phase 8 scripts | `REUSE_WITH_EXTENSION` | Observation, representation, component, phase and sensitive projections reusable after semantic changes. |

## Phase 8 interpretation guard

Phase 8 is `Environment-conditioned offline response gap`, not Real Shadow Mode. It has OFT only, fixed-index pairing, planned/commanded Real pose, and no planner-relative error. Its result files remain valid only under that label.

## Phase 9 safety disposition

Five inspected policy rollouts contain zero successes: one `policy_step_timeout` and four `consecutive_ik_failures`. Their action arrays can be diagnostic inputs only. They are not Planner GT, successful trajectories, Real replay commands, or safety evidence. The exporter script is explicitly classified `DO_NOT_USE_FOR_REAL`.

## Timing and action contract confirmed

- Dataset/robot control sample period: nominal 0.2 s (5 Hz).
- Existing episode 4 effective rate: approximately 4.999 Hz; configured and measured rate are distinct fields.
- OFT: inference every 1.0 s, K=5 actions, each action represents 0.2 s.
- OpenVLA: K=1 at 5 Hz.
- Canonical action: base/world-aligned delta translation in m, delta rotation vector in rad, gripper closedness in [0,1].
- Current Phase 8 Real metadata says `pose_source=planned_commanded_pose`, `feedback_pose_samples=0`; it must not be called measured motion.

See `reuse_manifest.json` for machine-readable dispositions.

