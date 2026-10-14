# Zero-offset candidate TCP cross-validation

## Evidence and scope

Operator supplied pendant settings transcription [0,0,0,0,0,0] mm/deg. Active TCP name and current pendant Cartesian pose remain UNKNOWN. No screenshot was actually attached to this request, so the artifact records text provenance only; no image or active binding was invented. No setters, motion, gripper, Hold/Stop or model rollout were called.

Fresh JointState was ordered by joint name joint_1 through joint_6, not message array position. Existing A0509 URDF FK base_link -> link_6 was reused; the output remains computed flange, not measured TCP. Same subscriber and existing FIRST_FRESH_SAMPLE -> consecutive 10-second clean policy remained active throughout the controlled calls.

## Source audit and controlled calls

- GetCurrentToolFlangePosx callback reads g_stDrState.fCurrentToolPosx with no vendor pointer or state-changing call. OnMonitoringDataExCB updates this from pData->_tCtrl._tTask._fActualPos[1]. No cache timestamp/freshness indicator is exposed. The callback ignores request ref; BASE was requested, but frame must not be claimed independently proven solely from this request. Response treated as controller flange cache only.
- GetCurrentPosx calls Drfl->get_current_posx(req->ref), with a nullptr check and no visible setter/motion. Prior live stability risk remains; classified READ_ONLY_BUT_STABILITY_UNVERIFIED, exercised only once under this explicit controlled-test request. Ref 0 is DR_BASE in the service schema. Project Cartesian convention is translation mm, angles degrees, intrinsic Euler ZYZ (02_safety/action_contract.py). Motion approval is not inferred from this one successful getter.
- Existing safe scalar mode/state/system queries each called once before pose getters: AUTO / STANDBY / REAL. Latest reported joint velocities were zero, supporting stationary state in software, not substituting for operator confirmation.
- Flange getter latency 7.206 ms. PosX getter latency 3.816 ms. Both success=true, valid finite 6D poses. Each call followed by 10 seconds of continuous JointState monitoring. No polling/retry of these getters.

## Results

FK flange position [304.607750, -11.262126, 724.503762] mm. Rotation matrix, quaternion, Euler ZYZ and source/receive timestamps retained in fk_flange_pose.json.

Controller flange and controller current PosX both returned:

[305.090179, -11.828292, 723.677795, 179.910461, -155.561234, 3.350623] mm/deg.

| Comparison | Translation error mm | Geodesic rotation error deg |
|---|---:|---:|
| FK vs controller flange | 1.111531 | 0.178201 |
| FK flange vs current PosX | 1.111531 | 0.178201 |
| Controller flange vs current PosX | 0.000000 | 0.000000 |

Rotation comparison used rotation composition and geodesic magnitude, not Euler subtraction. Controller minus FK xyz differences were [0.482429, -0.566166, -0.825967] mm.

JointState before/after: RUNTIME_READY, fault null. 3,003 samples overall including startup/warmup; runtime >=100 ms events 0. No timeout, null/invalid response or observed JointState feedback loss during this test.

## Interpretation and gate

This is strong numerical evidence supporting TCP=flange for the observed controller Cartesian response. It also supports consistency of the FK chain, while the nonzero FK residual is preserved rather than hidden. It does not prove the active configured TCP binding, cache freshness/frame identity, or establish a project-approved pose acceptance tolerance. No applicable crosscheck tolerance was found in the existing safety config; one was not invented.

Classification: TCP_ZERO_OFFSET_SUPPORTED_NOT_VERIFIED.

tcp_contract_verified: false. Physical commands: 0. Next: BLOCKED, not SHORT_HORIZON_READY.

Remaining TCP evidence: current active binding/controller configuration or pendant evidence identifying the active TCP, plus an approved criterion for accepting the observed FK/controller residual. Short-horizon additionally still requires fresh camera/model/hardware/operator checks and prior physical minimum-motion/stop-path validation. The last action over 10 mm and close prediction are not overridden by this read-only TCP test.
