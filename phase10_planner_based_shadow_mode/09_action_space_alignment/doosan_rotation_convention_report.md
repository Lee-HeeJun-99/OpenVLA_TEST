# Doosan rotation convention report

The collection code sends six-element absolute poses in base reference (`DR_BASE=0`, `DR_MV_MOD_ABS`) and labels the final three controller values as degrees. It stores them by element-wise degree→radian conversion. `single_robot_simple.py` then explicitly interprets the saved triple as roll/pitch/yaw via `rpy_to_quaternion_wxyz`, and computes `q_current * inverse(q_previous)` followed by axis-angle conversion for the dataset action.

The audited Doosan ROS2 service definitions and bridge confirm six Cartesian values, degrees, and base reference, but no inspected local definition establishes that the controller's A/B/C values are intrinsically the same RPY convention assumed by the collector. Therefore:

- Dataset/reference action rotation: valid under the established collector convention and reproducible.
- Native Doosan orientation semantics: `UNRESOLVED_ACTION_CONVENTION` pending vendor/controller confirmation.
- OpenVLA/OFT outputs: trained against the dataset's relative rotation-vector action fields.
- Raw Doosan angle triples must never be copied directly into canonical rotvec fields.

Translation and commanded-gripper comparisons remain valid. Rotation comparisons must carry `source_interpretation=collector_rpy_assumption` and cannot be presented as independently verified physical orientation error.
