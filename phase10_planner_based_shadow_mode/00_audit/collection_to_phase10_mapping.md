# Collection to Phase 10 mapping

| Collection field | Phase 10 field | Status |
|---|---|---|
| `episode_id`, `step_index` | identifiers | direct |
| image path / `image_timestamp` | raw image / camera source timestamp | direct; camera clock |
| `source_timestamp` | receive timestamp | monotonic receipt clock |
| `tcp_pose` | reference commanded pose | planned/commanded, never measured |
| adjacent `tcp_pose` | canonical Reference Command | extractor implementation |
| `planner_phase` | phase | direct; historical field name retained |
| `gripper_closedness` | reference gripper command | commanded state only |
| JointState arrays | measured joint position/velocity | message feedback, header contract still to verify |
| feedback TCP | measured EE pose | absent in episode 4 (`feedback_pose_samples=0`) |
| model action | OpenVLA/OFT prediction | recorded-input replay only |
| executed AI action | null | mandatory Shadow semantics |

Episode 4 recorded data extraction is complete. Actual-model comparison remains separate by checkpoint/variant and is not inferred from Phase 8 fixture/model outputs.
