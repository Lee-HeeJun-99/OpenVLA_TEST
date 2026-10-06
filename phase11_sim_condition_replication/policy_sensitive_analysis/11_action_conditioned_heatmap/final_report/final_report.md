# Phase 11 action-conditioned heatmap analysis

## Status

`COMPLETED_WITH_PARTIAL_ATTRIBUTION`

The completed method is actual-output occlusion sensitivity using both local-mean and Gaussian-blur replacement. Stored features did not contain attention matrices or input gradients, and both deployed runtimes use `torch.inference_mode()`. Attention rollout, Gradient×Input and Integrated Gradients were therefore not fabricated. This report provides limited intervention evidence, not a complete three-method causal triangulation.

## Scope and data

- Reused 15/15 valid Phase 11 pairs, 684 aligned samples, 912 OpenVLA predictions, 192 OFT K=5 predictions and the deterministic action metrics.
- Selected 202 representative rows; 18 high-cost rows were occluded in both baseline and condition images.
- Produced 72 NPZ heatmap files: 40 OpenVLA and 32 OFT.
- Component/chunk maps: 800; ROI rows: 4,000; replacement-method agreement rows: 400.
- Patch 160 px, stride 128 px; shared scale per comparison; black occlusion was not used.

## Results

OFT local-mean vs Gaussian-blur agreement was moderate/high: Pearson 0.711 translation, 0.681 rotation, 0.650 gripper and 0.726 normalized-total. OpenVLA agreement was weaker: 0.445, 0.307, 0.548 and 0.461 respectively. OFT conclusions are therefore more stable to the tested occlusion fill choice; OpenVLA spatial interpretation is less stable.

For OFT gripper sensitivity, orange-target/background density ratios were approximately 6.41 for distractor swap, 6.72 for extra object and 2.05 for lighting. In the inspected lighting false-close case, the strongest effect lies around the robot/gripper and lower background region, not the orange cube. Orange-target mass declines modestly from chunk 0 (0.0081) toward chunk 4 (0.0060). This supports a limited hypothesis that low lighting changes the visual regions controlling OFT gripper output and later chunks, but does not prove a causal hardware close.

OpenVLA target ROI estimates are unstable because the conservative orange mask is absent/very small in some selected frames and the two fill methods agree poorly for gripper maps. No strong OpenVLA ROI conclusion is made.

## ROI validation

The first HSV mask falsely labeled reflective robot metal as blue. Direct overlay inspection detected this failure. The analysis was rerun with a conservative orange-target mask. Yellow/blue/red, gripper and extra-object masks are marked unresolved rather than inferred from unreliable color/reflection cues. Consequently, claims about distractor or additional-object ROI movement are not supported by this run.

## Relationship to Phase 11 action results

The occlusion results are consistent with lighting being a gripper-specific risk for OFT, while the earlier total scale-normalized action ranking still places extra-object slightly above lighting. This demonstrates that total action shift and spatial gripper sensitivity answer different questions. Correlations based on only 2–4 selected frames per model/condition are diagnostic and are not treated as population evidence.

## Method interpretation

- Attention: not available; no attention claim.
- Attribution: gradient/IG path blocked; no gradient claim.
- Occlusion effect: measured action change under two image interventions.
- Causal evidence: limited local intervention evidence, susceptible to distribution shift from occlusion.

## Rollout hypotheses

- Extra object: test alignment/approach path bias.
- Lighting: test OFT early close during alignment/descent and record EE–cube distance at close.
- Distractor swap: use as the lower-impact comparison condition.

No actual robot rollout was performed.

## Limitations and blockers

- No attention/gradient triangulation.
- Only 18 high-cost selected rows, not all 912 frames.
- Coarse 160/128 occlusion grid.
- Reliable ROI limited to orange target and its complement.
- OpenVLA action-token gradient path remains unresolved.
- OFT differentiable forward requires removing and validating the inference-only wrapper without changing model semantics.
- Robot/ROS/trajectory/gripper/Home operations: zero.
