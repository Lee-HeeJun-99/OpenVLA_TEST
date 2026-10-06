# Local operator checklist

No item was automatically approved. Statuses are operator attestations, not controller telemetry.

2026-10-02: the user/local operator reported that the requested physical checklist was checked. This is recorded as an operator attestation, not as independently measured controller signals. Items such as servo, protective stop and E-stop still require a corresponding signal or explicit value before motion readiness can pass.

| Item | Status |
|---|---|
| Robot area clear | operator_confirmed |
| Collision objects absent | operator_confirmed |
| Physical E-stop location known | operator_confirmed |
| Immediate E-stop access | operator_confirmed |
| E-stop blocks controller motion | operator_confirmed |
| Protective-stop state observable/currently clear | operator_confirmed |
| Controller alarm observable/currently clear | operator_confirmed |
| Servo state observable/current state accepted | operator_confirmed |
| Robot mode observed/current mode accepted | operator_confirmed |
| Robot stationary | operator_confirmed |
| Current joints safe | operator_confirmed |
| Current TCP safe | operator_confirmed |
| Gripper state safe | operator_confirmed |
| ZED mechanically fixed | operator_confirmed |
| Cube/obstacle layout approved | operator_confirmed |
| Workspace X 275–554, Y -355–386, Z 267–754 mm approved | operator_confirmed |
| Joint velocity/acceleration 20 deg/s, 20 deg/s² approved | operator_confirmed |
| TCP translation velocity/acceleration 20 mm/s, 20 mm/s² approved | operator_confirmed |
| TCP rotation velocity/acceleration 20 deg/s, 20 deg/s² approved | operator_confirmed |
| Maximum rollout duration 30 s approved | operator_confirmed |
| Authorized motion approver identified | operator_confirmed |

## 2026-10-02 latest field attestation

The local operator reported: `현장 특이사항 없음` (no unusual condition at the
site). This is recorded as a current general field attestation. It confirms that no
new visible site anomaly was reported; it does **not** supply missing numerical limits
or independently verify controller safety signals.

The following existing confirmations remain current: robot area clear, collision
objects absent, physical E-stop location known and immediately accessible, operator
attestation that E-stop blocks motion, robot stationary, ZED fixed, layout approved,
and authorized motion approver present.

## 2026-10-02 explicit checklist confirmation

The user/local operator subsequently confirmed that every value in the requested
physical-safety form and its stated numeric candidates is correct. These are recorded
as `operator_confirmed`; this is not independent controller telemetry and is not an
explicit motion-command approval. Measured gripper feedback remains unavailable.

### Operator-requested speed profile

The operator initially requested a speed comparable to `vel=50, acc=50`, then selected
`20` for every velocity and acceleration limit. The audited Doosan
interfaces define task-space `vel` as `[mm/s, deg/s]` and `acc` as
`[mm/s², deg/s²]`; joint values use `deg/s` and `deg/s²`. Phase 12 therefore records
the following operator-confirmed ceilings:

- TCP translation: 20 mm/s; acceleration: 20 mm/s².
- TCP rotation: 20 deg/s; acceleration: 20 deg/s².
- Joint: 20 deg/s; acceleration: 20 deg/s².

At the 5 Hz policy rate, velocity alone corresponds to ceilings of 4 mm translation
and 4 degrees rotation per control interval. Acceleration and per-action clipping must
still be enforced independently. Status: `CANDIDATE_OFFLINE_ONLY`; no robot runtime
was changed and this is not motion approval.
