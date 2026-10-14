# Full-policy integration and preflight result

Controller now offers the explicit `execution_mode='FULL_POLICY_ROLLOUT'`
for OpenVLA/full_task. Legacy and arm-only modes remain separate. The new
gripper lifecycle requires OPEN initialization ACK and independent actual OPEN
confirmation, uses strict <0.3/>0.7 hysteresis, suppresses duplicate states,
and commits command knowledge only after an acknowledged transition. It does
not claim measured jaw feedback. Arm hard gates, dynamic limiter and workspace
remain unchanged. Concurrent full-policy steps are refused.

240 tests passed: integration 107, safety 103, INVALID policy 30. Synthetic
test abort evidence never authorizes real hardware.

The current hardware config still has `abort_value: null` and
`abort_value_confirmed: false`. Existing audit explicitly classifies gripper
abort behavior as unverified. A normal pulse's OFF edge proves neither safe
release nor safe retention during a fault. No OUTPUT_OFF/HOLD_LAST/OPEN
behavior was invented, and no hardware config or safety threshold was changed.

Consequently the requested full episode was **NOT STARTED**. There were zero
physical commands in this request, including OPEN initialization. No task
SUCCESS/FAILURE/INVALID was fabricated for an episode that never began.

TCP remains formally supported-not-verified; operator engineering acceptance
is recorded separately, not promoted to formal verification.

## Remaining implementation boundary

The new lifecycle is connected at the production Controller API and tested
with fake transport. The existing command-line launcher has not been enabled
for this mode: a live initialization-confirmation workflow and episode outcome
handling must be connected after a grounded abort contract is supplied. This
is not a completed or live-validated full-episode runner.

Next required evidence is the real gripper's fault/abort behavior and the
corresponding supported IO command, from hardware documentation, a previously
validated implementation, or a specifically approved physical verification.
Do not choose this behavior by guessing the desired jaw state.
