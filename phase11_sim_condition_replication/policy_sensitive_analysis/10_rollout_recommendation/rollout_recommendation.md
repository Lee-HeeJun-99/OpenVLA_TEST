# Phase 11 rollout condition recommendation

- LOW_POLICY_IMPACT: `distractor_swap`.
- HIGH_POLICY_IMPACT: `extra_object`.
- PHASE_SPECIFIC_IMPACT: `lighting_low`, focused on pre-grasp/gripper-close timing.

These are offline hypotheses, not rollout outcomes. Use paired layouts, record reach/alignment/grasp/lift failure, close timing, minimum cube–gripper distance, timeout and hold. A practical exploratory design is at least 10 paired rollouts per selected condition and model; power should be recalculated from pilot outcomes.
