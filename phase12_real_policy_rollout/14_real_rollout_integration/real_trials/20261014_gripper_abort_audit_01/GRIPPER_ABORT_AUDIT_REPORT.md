# Gripper abort hardware audit

Branch `lhj-research`, HEAD `98c4879ff456370698329c3009d3b88b214c4ba6`.
Existing dirty worktree preserved. No running process restarted, no physical
command sent, no network, TCP, mode or power configuration changed.

## Normal command versus hardware safety

Current observed calibration: Tool DO1 ON=1 for 1s then OFF=0 is logical OPEN;
Tool DO2 ON=1 for 4s then OFF=0 is logical CLOSE. OPEN initialization followed
by CLOSE worked according to the operator; initial CLOSE alone did not.
These are normal command observations, not a cancellation/failsafe test.

Historical collector DigitalGripper uses OPEN two 1s pulses and CLOSE one 1s
pulse. Current production yaml uses OPEN two 1s pulses and CLOSE one 4s pulse.
The new full-policy lifecycle requires one initialization pulse. This additional
configuration mismatch is reported, not silently changed or treated as PASS.

Code expresses command knowledge, not jaw feedback. Two command channels and
sequence dependence support a stateful interface, but do not identify the actual
gripper as a latching relay, dual-solenoid valve, or momentary actuator. No
current hardware-model manual, wiring binding or jaw sensor mapping was found.

## Abort evidence

The legacy bridge clears pending gripper commands on emergency-stop. Its pulse
path has interruptible sleep and cancellation checks before the inactive write.
Cancellation can therefore leave a previously asserted output ON. No pending
command is not equivalent to electrical OFF or physical HOLD_LAST.

Phase12 Controller inhibits arm/gripper dispatch and requests existing arm HOLD
(Stop fallback). During a pulse the sink cancels the wait and writes a configured
selected-channel abort_value; during idle there is no independent gripper write.
The config still has abort_value null. A direct sink.stop is arm-only and is not
an independently verified gripper failsafe.

No existing real design specifies jaw OPEN/CLOSED/HOLD_LAST for model, sensor,
workspace, ACK, logger or operator faults. Normal pulse OFF cannot establish
that specification. OPEN-on-fault could drop an object; HOLD_LAST under an
interrupted pulse, supply failure or partial actuation is also unproven.

## Live read-only evidence

Tool and control-box DI/DO channels1/2 each queried once, all success=true and
raw0. These are electrical bits, not mapped jaw-limit sensors. Hardware current
REAL/AUTO/STANDBY. JointState RUNTIME_READY, no fault; post-query runtime
source/receive >=100ms events0, max12.803/36.956ms. Camera stream and model
identity current PASS. These facts do not authorize an unverified gripper abort.

## Tests and outcome

243 tests PASS: integration110, safety103, INVALID policy30. Added fake-transport
coverage for faults while commanded OPEN/CLOSED, repeated abort, future dispatch
inhibit and broken logger. These prove software control behavior only; they are
not physical jaw failsafe evidence.

**GRIPPER_ABORT_BEHAVIOR_UNVERIFIED.** Abort behavior/value not selected.
**Physical test NOT EXECUTED**, physical commands0. Full-policy launcher and
full episode remain BLOCKED as requested when verified=false. No claim that a
complete live full-policy launcher has been delivered.

Next evidence: actual gripper model and wiring/pneumatic diagram or manufacturer
state-retention/fault specification. Then a narrowly approved gripper-only test
may verify OPEN initialization, CLOSED observation, one defined abort and final
jaw state, including interruption during a pulse where relevant. Arm remains
stationary, no retry/channel search. A completed-pulse idle observation alone
must not be generalized to every fault timing or power/air failure.
