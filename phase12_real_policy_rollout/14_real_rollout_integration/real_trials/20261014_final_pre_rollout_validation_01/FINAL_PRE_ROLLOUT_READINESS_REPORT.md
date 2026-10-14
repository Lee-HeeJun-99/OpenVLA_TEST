# Final pre-rollout validation — no physical commands

Production translation gate changed by explicit user request from 10 to 13 mm, grounded in 50/50 valid real samples with p99/max 12.875 mm. Configuration snapshot hashes and git baseline preserved; no other safety thresholds changed. Exact 13 mm accepted, 13.1 mm rejected; NaN/Inf rejected. Shared source is safety_limits.yaml -> action_step_limits -> production SafetyPipeline and batch. Historical >4mm statistics and 4mm velocity-derived ceilings are diagnostics/dynamic consequences, not residual raw rejection gates.

| Item | Verdict |
|---|---|
| Hardware state | PASS (REAL/AUTO/STANDBY scalar only) |
| JointState | PASS |
| Start pose | PASS |
| Camera | PASS |
| Model identity | PASS |
| Translation 13mm gate | PASS |
| Rotation gate | PASS |
| Dynamic limiter | PASS (raw action modified) |
| Workspace | PASS (50/50 hypothetical targets; earlier controller pose) |
| Action convention | PASS (software); physical axis proof pending |
| TCP evidence | CONDITIONAL / NOT VERIFIED |
| Gripper semantics | FAIL (hardware mapping unverified) |
| Initial gripper state | FAIL (UNKNOWN) |
| Logger | PASS |
| Command inhibition | PASS |
| Operator/E-stop | FAIL (fresh manual confirmation required) |

Current JointState monitoring requested 30s and observed 41.057s including post-getter/prediction phase: 4107 samples, max source gap 12.769ms, max receive gap 33.417ms; >=100ms events 0, invalid/duplicate/regression 0. Hardware current getter responses REAL/AUTO/STANDBY. Latest JointState velocities all zero is software stationary support only.

Camera prediction: start age 57.742ms; end age 492.097ms; inference 434.355ms; actual bgra8 -> RGB -> JPEG input and hashes saved. Strict current model health validated. Raw translation 12.875387mm passes 13mm; gripper 0.998039 close.

Dynamic limits remain 20mm/s translation velocity and 20mm/s² acceleration, rotation20deg/s and20deg/s² at5Hz. Raw13mm implies65mm/s. Speed-limited step max4mm; first acceleration-limited step0.8mm. Current raw action first candidate translation norm 0.800000mm. Raw action is NOT delivered unchanged. Exact per-stage vectors/target are in dynamic_limiter_validation.json; these are unauthorized hypothetical calculations, actual delivered action null.

Coordinate source confirms BASE translation metres -> mm, relative base rotvec radians composed R_delta @ R_current -> ZYZ degrees. Sink is absolute BASE active-TCP MoveLine ref0/mode0, not flange-space. Fifty raw hypothetical targets calculated using earlier controlled current PosX snapshot are inside bounds; orientation relative-matrix checks finite. Earlier pose is not promoted to fresh verified TCP. TCP zero-offset remains supported not verified; no auto engineering approval or tolerance creation.

Model gripper0=open/1=closed differs from legacy digital commanded-open bit. Phase12 physical sink uses separate output channels and checks polarity/abort confirmation; config has polarity_confirmed=false and abort_value=null. Initial physical gripper state UNKNOWN. Source shows configured open_pulse_count2 is not consumed by sink's one on/off pulse; mismatch is reported, not silently fixed. Legacy ActionAdapter inverted mapping must not be used as verified evidence. Short-horizon currently gripper-disabled and rejects close>=0.7; full-task is the only Controller.step path that dispatches gripper. ARM_ONLY can only be a system integration test, never full-policy task success evaluation.

All unit suites run with fake/no physical transport; detailed returns captured. Logger uses actual FsyncJsonlLogger append/flush/fsync and100MiB free-space threshold; probe PASS. Fault injection includes sensor/model/translation/rotation/logger/NaN terminal INVALID and no future command dispatch. Single trial runner has no batch/retry/home/recovery; its2s max10actions is not an exactly-one-pose-command protocol. Minimum-motion is separate max1 and must precede it with physical PASS; no such PASS/physical stop evidence found.

Fresh operator/workspace/E-stop/stationary confirmations REQUIRED; prior confirmations not reused. Authority/servo/protective/emergency remain unknown independently of STANDBY. These are not satisfied by software event silence or successful scalar getters.

Final: **PHYSICAL_SHORT_HORIZON_BLOCKED**. Reason: unverified TCP, physical gripper mapping/abort/initial state, fresh manual/hardware safety evidence, required minimum-motion/stop proof, and close-prediction rejection in current short-horizon protocol. Physical commands **0**. No physical Hold/Stop/gripper/motion client used for live checks; fake test transport actions are not robot commands.
