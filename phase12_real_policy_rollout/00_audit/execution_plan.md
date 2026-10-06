# Gated execution plan

## Gate 0 — completed offline

Static source/config audit, command-path map, pure safety contracts, mock tests, read-only model/ROS availability attempts. No runtime source modification and no launch.

## Gate 1 — local read-only inspection (requires operator; no motion)

Confirm ROS graph, exact types/rates/QoS, camera resolution/frame, joint/TCP streams, command versus measured gripper, controller revision, hardware E-stop, measured workspace/home/joint limits, orientation convention and disk/clock status. Start prediction-only servers separately and verify exact Phase 11 health metadata. Do not start ActionAdapter, DoosanBridge or EpisodeManager.

## Gate 2 — subscriber-only Shadow readiness

Integrate Phase 12 polarity/queue/logger contracts into a command-free observer. Static inspection must prove it imports/creates no motion publisher/service/action client. Select `first_only` or `sequential_k5` only after reviewing the deployed training/runtime contract. User approval required.

## Gate 3 — one baseline Shadow trial

Only an operator-approved scripted reference controller may move the robot. AI outputs remain prediction-only and `ai_executed_action=null`. Stop after one trial for integrity/safety review.

## Gate 4+ — closed loop

Requires separate approval after Shadow. Run only one trial per model/start position initially; then gated prototype remainder; then condition blocks. Never auto-chain all rollouts.

No command lines that start motion are included at Gate 0.
