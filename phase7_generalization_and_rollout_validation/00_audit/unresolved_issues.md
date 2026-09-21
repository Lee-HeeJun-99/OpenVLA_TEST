# Phase7 Priority1 Audit Unresolved Issues

- GT action is not verified; metrics are Real/Sim policy prediction disagreement.
- Gripper binary threshold 0.5 is assumed for disagreement analysis; not GT accuracy.
- Real measured motion and measured gripper feedback are unavailable in the 225-pair offline dataset.
- Direct Real TCP vs Sim EEF physical distance remains invalid without frame convention reconciliation.
- P4+C2 combined shift used 24 px at 224 resolution, not relative equivalent of 24 px at 1280.
- Current ROS proprio deployment is not equivalent to this offline vision-policy analysis.
