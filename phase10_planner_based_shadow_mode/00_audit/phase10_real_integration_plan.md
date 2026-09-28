# Phase 10 Real integration plan

1. Use `real_shadow_config.yaml`; startup fails unless mode is offline/dry-run, shadow mode is true, and every capability flag is false.
2. A future subscriber-only shell may observe configured topics, but must contain no publisher/service/action client. The current deliverable stops before that ROS binding.
3. Convert received data with `RealRuntimeObserverAdapter`; retain native runtime values and clock domains. Do not convert absolute Doosan Euler poses into canonical deltas until convention/frame is verified.
4. Synchronize only samples sharing a verified clock domain. Otherwise use frame/sequence association and mark invalid rather than subtracting timestamps.
5. Persist planner target/raw/safety/executed/measured feedback separately. AI-executed fields remain null.
6. Run OpenVLA K=1 and OFT K=5 only against the immutable recording, then integrity/alignment/action-space validation.
7. Local operator must resolve planner source, measured gripper, EE/command timestamps, rates, controller limits, hold acknowledgement and E-stop state before any Real trial.

Current status: `BLOCKED_REQUIRES_LOCAL_OPERATOR_AND_HARDWARE`. This is not a software failure; it is the required protocol stop point.
