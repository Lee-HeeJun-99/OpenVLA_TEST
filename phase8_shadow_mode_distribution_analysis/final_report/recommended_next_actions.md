# Recommended Next Actions

## Immediate

1. Use this Phase 8 report as the current offline evidence summary.

2. Add frame-level correlation plots:
   - gripper-sensitive energy vs gripper gap
   - Phase 6 sensitive energy vs action gap
   - hidden cosine vs action gap

3. Inspect high-gap frames from `real4_vs_real8`, especially hold/alignment, to confirm the visual trigger of gripper disagreement.

## Next Data Collection

1. Collect controlled lighting sweep:
   - normal
   - mildly dark
   - strongly dark
   - directional shadow

2. Keep cube layout and trajectory fixed for lighting sweep.

3. For object insertion, collect multiple object types and positions to avoid one-off conclusions.

## Sim Extension

1. Generate matching sim variants for:
   - darker lighting
   - extra object near cube
   - distractor cube position changes

2. Compare fixed-sim reference vs matched-sim condition to separate:
   - real-only sensitivity
   - Real-Sim mismatch reduction

## Deployment Track

1. Do not start closed-loop rollout from this result alone.

2. Next practical validation should be Shadow Mode:
   - real camera input
   - VLA action logging only
   - planner still controls robot
   - compare gripper timing and gripper output confidence across lighting conditions
