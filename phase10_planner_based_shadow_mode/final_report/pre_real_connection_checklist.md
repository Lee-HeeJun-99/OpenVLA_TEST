# Pre-Real connection checklist

| Item | Status | Required evidence/action |
|---|---|---|
| Robot/network connection | REQUIRES_HARDWARE | Local operator verifies; not attempted remotely |
| Controller revision | REQUIRES_LOCAL_OPERATOR | Record robot/controller/driver revisions |
| ROS graph | REQUIRES_ROS_GRAPH | Read-only local inspection after approval |
| Camera topic | REQUIRES_ROS_GRAPH | Confirm `/vla/image_rgb`, encoding, header clock and measured rate |
| Measured joint state | REQUIRES_ROS_GRAPH | Confirm `/dsr01/joint_states`, names/units/rate/stamps |
| Measured EE state | UNRESOLVED | `/doosan/current_pose` is unstamped `[mm,deg]`; establish timestamp/frame |
| Measured gripper state | BLOCKED | `/vla/gripper_open` is a command proxy, not confirmed feedback |
| Planner action | BLOCKED | Dedicated planner and raw-action interface not found |
| Executed command | REQUIRES_ROS_GRAPH | Confirm selected servol/speedl/service mode and observable command semantics |
| Home pose | REQUIRES_LOCAL_OPERATOR | Verify definition and measured tolerance; do not move remotely |
| Gripper Home | REQUIRES_LOCAL_OPERATOR | Verify physical state and feedback; do not actuate remotely |
| Workspace limits | REQUIRES_LOCAL_OPERATOR | Review configured `[250,-400,20]..[750,400,700]` mm against cell |
| Joint limits | UNRESOLVED | No package-side joint-limit source found |
| Velocity limits | UNRESOLVED | Review bridge parameters and driver/controller enforcement |
| Hold acknowledgement | BLOCKED | Stop service exists; explicit acknowledgement/state not found |
| Hardware E-stop | REQUIRES_HARDWARE | Local physical test/procedure, outside this stage |
| Logger disk path/capacity | READY_OFFLINE | JSONL fsync implemented; local capacity still check before collection |
| Clock synchronization | BLOCKED | Camera/joint headers versus unstamped EE/commands need contract |
| OpenVLA server | READY_OFFLINE | Recorded-input K=1 fixture path tested; live server not exercised |
| OFT server/checkpoint | READY_OFFLINE | Recorded-input K=5 fixture path tested; live server not exercised |
| Instruction normalization | READY_OFFLINE | Exact baseline `Pick up the orange cube.` configured |
| Planner-only dry run | REQUIRES_LOCAL_OPERATOR | Requires resolved planner and explicit approval |
| Real Shadow Mode | BLOCKED | Depends on all blockers and operator approval |

Overall: `BLOCKED_REQUIRES_LOCAL_OPERATOR_AND_HARDWARE`.
