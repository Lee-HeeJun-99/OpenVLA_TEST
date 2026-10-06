# Phase 12 Cartesian workspace candidate audit

Status: `DATA_DERIVED_CANDIDATE_NOT_OPERATOR_APPROVED`

Source: Real episode 1–10 `steps_with_actions.jsonl`, 452 rows. Every source pose is
labelled `pose_source=planned_actual_duration`; it is not measured TCP feedback.

Observed planned-pose envelope:

| Axis | Minimum | Maximum |
|---|---:|---:|
| X | 305.090 mm | 523.100 mm |
| Y | -323.005 mm | 355.500 mm |
| Z | 297.824 mm | 723.678 mm |

Candidate envelope after an outward approximately 30 mm audit margin:

| Axis | Minimum | Maximum |
|---|---:|---:|
| X | 275 mm | 554 mm |
| Y | -355 mm | 386 mm |
| Z | 267 mm | 754 mm |

This box is intended only for fail-closed offline candidate validation. It does not
prove collision clearance, robot reachability, singularity safety, table clearance or
the absence of obstacles throughout the enclosed volume. It must be reviewed at the
site before it can become an approved motion workspace.

The workspace checker has no ROS or command capability and returns
`command_issued=false`. Closed-loop remains blocked while protective-stop state, Hold
acknowledgement and measured gripper feedback/no-feedback policy remain unresolved.
