# Actual arm-only one-step system integration test

Physical arm commands **1**; gripper **0**; physical Hold/Stop **0**. Real inference requests **1**. No second step/retry/batch/home/recovery. Scope SYSTEM_INTEGRATION_TEST, task_success=null; never counted as policy task SUCCESS/FAILURE.

Fresh operator four-condition confirmation received and preserved with timestamp. REAL/AUTO/STANDBY current precheck PASS. Existing persistent clean10s readiness obtained, start pose within0.5 degrees, zero JointState runtime faults. Existing CurrentPosX(ref0) getter called once, valid response; no setter/tool change. TCP formal classification remains TCP_ZERO_OFFSET_SUPPORTED_NOT_VERIFIED; execution acceptance is explicitly scoped consistency-based, active TCP name not directly observed. Post-motion state query returned STANDBY.

Fresh image 1280x720 bgra8 -> RGB -> JPEG -> strict OpenVLA step8130 K1 dim7 proprio=false. Camera request-start age 29.665ms, end age 471.544ms, inference 441.879ms. Raw action [0.0014104356263888885, -0.00017932358931696795, -0.00019476661160999081, -0.004078358995333046, 0.002364704483548279, 0.010181925561301089, 0.0019607843137254832]. Translation norm 1.435068mm <13mm; rotation 0.642879 degrees <4deg. Gripper raw 0.001961 stored only; no hardware IO/pulse/abort gripper output.

Existing velocity stage preserved the 1.435068mm translation; existing first-step acceleration limiter reduced it to 0.800000mm. No new clipping/scaling. Final candidate and exact current/target pose, BASE absolute ref0/mode0, vel/acc20 are retained. Euler target values can look different by equivalent representation; rotation validation uses matrices/geodesics, not component subtraction.

MoveLine sent once, ACK success=true; synchronous service receipt completed after 555.992ms. Service ACK alone is not used as proof of physical PASS.

| Metric | Recorded result |
|---|---|
| Requested limited xyz mm | [0.7862684173434938, -0.09996661465904517, -0.10857555821531598] |
| Observed FK flange xyz mm | [0.792227537606216, -0.10499691473057036, -0.10998288281593993] |
| Requested / observed norm mm | 0.800000 / 0.806688 |
| Delta error mm | 0.007924 |
| Direction cosine | 0.999986000 |
| Magnitude ratio | 1.008360 |
| Requested / observed rotation deg | 0.642879 / 0.641841 |
| Relative rotation mismatch deg | 0.001875 |
| Max source / receive gap ms | 12.043 / 15.126 |
| Runtime >=100ms events | 0 / 0 |
| Final joint velocities | [0.0, 0.0, 0.0, -0.0, -0.0, 0.0] |

Feedback is measured JointState plus computed flange FK, not independently measured TCP. Small requested/observed motion agreement is strong command-path evidence, but no new physical acceptance tolerance was invented and formal TCP verification is not promoted. E-stop used/unexpected motion remain null until operator supplies actual observation.

Current result **PHYSICAL_CONFIRMATION_PENDING**. Final ARM_ONLY_SHORT_HORIZON_PASS/FAIL requires operator confirmation of expected motion, no gripper movement, no unexpected motion and E-stop usage. No further physical command is permitted in this trial. Next ANALYZE_ARM_TEST.
