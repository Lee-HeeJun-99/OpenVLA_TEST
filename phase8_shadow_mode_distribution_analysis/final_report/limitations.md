# Phase 8 Limitations

1. This is offline policy-output analysis, not closed-loop robot performance.

2. The gripper/action gap is disagreement between two policy outputs under different observations. It is not ground-truth action error.

3. Episode count is small: 45 frames per comparison and only one trajectory family.

4. Phase 8 real-only changes are sequential, not fully factorial:
   - real4→real8 changes lighting.
   - real8→real9 changes blue/yellow cube placement.
   - real9→real10 adds an object on top of the previous condition.

5. Sim images were fixed to episode 4 for reference. Sim was not regenerated to exactly match each new real perturbation.

6. `real9_vs_real10` has a larger planned EEF translation difference than the other real-only comparisons, so small action changes should be interpreted with correspondence caution.

7. The Phase 6 low-rank sensitive basis was learned from the previous 5-episode P0 Real-Sim setting, not from Phase 8 data.

8. The gripper-specific direction uses local first-order gradients and is not a causal intervention by itself.

9. Results apply to `oftplus_h5_vision` checkpoint step 28560 only. They are not ROS proprio-policy results.
