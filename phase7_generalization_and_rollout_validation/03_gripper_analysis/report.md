# Phase7 Priority1 Gripper Analysis Report

Status: `PARTIALLY_VERIFIED_REAL_SIM_DISAGREEMENT_ONLY`

The available data supports Real/Sim policy gripper prediction disagreement analysis. It does not support gripper accuracy, false-open, false-close, precision, recall, or F1 because no verified GT/measured gripper state is available.

The binary threshold is `0.5` and is marked `ASSUMED_THRESHOLD`.

Key question answer:

- Whether P4 remains useful without gripper must be evaluated using translation/rotation metrics separately. Phase7 multi-metric outputs include those separate metrics.
- Gripper timing is available only within predicted 5-step action chunks, not as measured physical gripper event timing.
