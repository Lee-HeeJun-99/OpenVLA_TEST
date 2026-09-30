# Pre-Real connection checklist

| Item | Status | Evidence / next action |
|---|---|---|
| Real collection code audit | READY_OFFLINE | Both collection files and direct control/logging paths audited |
| Reference trajectory classification | READY_OFFLINE | `SCRIPTED_REFERENCE_TRAJECTORY` |
| Reference action extraction | READY_OFFLINE | Episode 4: 45/45, exact match to source actions |
| Doosan rotation convention | UNRESOLVED | Dataset RPY assumption reproducible; native controller convention needs confirmation |
| OpenVLA checkpoint | READY_OFFLINE | `vanilla_s1_balanced_step8130`, `openvla_token`, H=1 |
| OFT checkpoint/variant | READY_OFFLINE | Phase 8 vision: step 28560; Real runtime proprio: step 6000; never merged |
| Preprocessing alignment | UNRESOLVED | Vanilla crop-bottom=0; OFT vision server defaults center crop; Real runtime crop disabled |
| Instruction normalization | READY_OFFLINE | Source is `Pick up the orange cube.`; servers normalize differently and log both |
| Recorded-input actual-model test | READY_OFFLINE | OpenVLA 45 H=1 and OFT 9 K=5 chunks; zero errors/commands |
| Timestamp policy | READY_OFFLINE | Explicit source/receive clocks and no cross-domain subtraction |
| Measured EE pose | UNRESOLVED | Runtime pose is unstamped `Float64MultiArray` |
| Measured gripper state | BLOCKED | Only commanded state found |
| Reference command logging | READY_OFFLINE | Schema/extractor complete; live observation not run |
| AI command disconnection | READY_OFFLINE | Empty launch draft; no publisher/service/action client |
| Shadow launch safety | READY_OFFLINE | Unsafe switches rejected; launch contains zero nodes |
| Home pose | REQUIRES_LOCAL_OPERATOR | Verify joints/tolerance without remote movement |
| Gripper Home | REQUIRES_LOCAL_OPERATOR | Verify physical state/feedback locally |
| Workspace/joint/velocity limits | UNRESOLVED | Workspace configured; controller joint/velocity enforcement needs review |
| Hold acknowledgement | BLOCKED | Explicit acknowledgement not found |
| Hardware E-stop | REQUIRES_HARDWARE | Local physical procedure |
| Local operator | REQUIRES_LOCAL_OPERATOR | Required for every next Real step |
| Robot/network | REQUIRES_HARDWARE | Not attempted |
| ROS graph | REQUIRES_ROS_GRAPH | Read-only local verification next |
| Subscriber-only recorder | REQUIRES_ROS_GRAPH | Do not start until clock/topic review |
| Reference-only dry run | REQUIRES_LOCAL_OPERATOR | Requires explicit approval |
| Real Shadow Mode | BLOCKED | Depends on unresolved items and approval |

Overall: `BLOCKED_REQUIRES_LOCAL_OPERATOR_AND_HARDWARE`.
