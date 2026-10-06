# Heatmap-grounded rollout hypotheses

## Extra object — approach bias

- Independent variable: additional-object condition.
- Expected change: larger translation/normalized action shift and reduced target-relative occlusion concentration in selected frames.
- Expected failure phase: alignment/approach.
- Measure: EE path direction, minimum target distance, additional-object distance, reach/alignment failure.
- Support: paired path shifts toward the additional-object side together with reduced target ROI sensitivity.
- Reject: trajectory and target-distance distributions remain baseline-like.

## Lighting — OFT premature close

- Independent variable: low-light condition.
- Expected change: early gripper close during alignment/descent; gripper occlusion effect concentrated in robot/background rather than the orange target.
- Expected failure phase: alignment and descent_to_grasp.
- Measure: first/persistent close time, EE–cube distance at close, gripper state, target/background occlusion ratio.
- Support: premature close repeats and target/background ratio stays below baseline/other-condition range.
- Reject: close timing remains baseline-like or target-localized sensitivity returns without early close.

## Distractor swap — low-impact control

- Independent variable: yellow/blue cube swap.
- Expected change: smaller normalized action shift than extra-object and lighting.
- Expected failure phase: no phase-specific increase predicted.
- Measure: paired success, component action shift, target distance, close timing.
- Support: outcomes remain closer to baseline than the other two conditions.
- Reject: paired failures or action deviations match/exceed high-impact conditions.

These are offline hypotheses only. No rollout was executed.
