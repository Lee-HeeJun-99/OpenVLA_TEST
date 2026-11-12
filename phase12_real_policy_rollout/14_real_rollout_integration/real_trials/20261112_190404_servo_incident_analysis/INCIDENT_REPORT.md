# 20261112_190404 ServoL incident — recorded evidence only

Physical robot calls: **0**. Original rollout artifacts unchanged. No runtime code or limits changed.

## Findings

The launcher observed robot_state=10 (local Doosan DRFC mapping: SAFE_OFF2), then attempted HOLD(3). Stop response success=false; this is not a verified software stop. JointState reports zero final velocities, but does not turn failed Stop ACK into PASS.

Joints were reordered by name (original array order: J1,J2,J4,J5,J3,J6). In the last roughly 3 seconds J5 moved from approximately -108 to -137.91 degrees, peak reported speed 21.290 deg/s. J6 reached approximately -127.86deg, peak absolute reported speed 16.066deg/s. J5 first crossed historical -135deg reference at monotonic 78802.104071199; minimum -137.916874deg. Actual configured controller limit and detailed controller alarm are not present in this folder, so a specific joint-limit alarm cannot be proven from state=10 alone.

## Rotation/command evidence

- Maximum raw model rotation over all 224 predictions: **3.985417deg**, step 80. Thus the new 8deg allowance was not exercised in this run; reverting to 4deg would not have rejected any of these raw rotation actions.
- Maximum consecutive published orientation geodesic step: **0.916081deg**. Maximum raw Euler component-difference L2 between consecutive commands: **2.642273deg**. No sudden 180/360deg jump in emitted messages.
- Maximum internally commanded angular speed: **17.168495deg/s**, below configured 20deg/s. Cartesian angular limits do not bound each joint to that same numerical speed.
- Published C first moved below -180deg at monotonic **78799.595141412**, about 2.835sec before software Stop request. At that point ABC=[111.424848,-160.609622,-180.059768]. Final published C=-225.366790deg. Controller CurrentPosX wraps its reported C into another equivalent range.
- The closest stream B distance to 0 or +/-180deg over the run is **13.767280deg**. This is not an exact Euler singularity crossing.
- Current controller orientation and most recent logged requested orientation diverge from about 0.46deg (step217) to **11.62deg** (step223). This comparison is orientation-invariant; it is not merely a 360deg subtraction artifact. Request/receive times are monotonic, but vendor command consumption times and actual acquisition times are not known.

| Step | Seconds before Stop request | Raw rotation deg | Current/requested orientation difference deg | J5 deg |
|---:|---:|---:|---:|---:|
|217|3.007|0.850|0.462|-107.898|
|218|2.529|0.850|0.902|-108.383|
|219|1.978|0.773|1.695|-110.298|
|220|1.529|1.068|3.621|-114.081|
|221|1.026|0.805|7.304|-121.607|
|222|0.514|0.111|11.568|-131.064|
|223|0.026|2.711 (filtered 0.804)|11.618|-137.909|

In particular, raw rotation at step222 was only 0.111deg while the physical trajectory was already rotating rapidly. This is not evidence of a single huge policy rotation commanding the entire motion.

## Causal assessment

Observed chronological sequence (not proof of the specific controller alarm): expanding real/requested orientation discrepancy and accelerating joint movement -> J5 beyond the historical -135deg reference -> controller protective state SAFE_OFF2 -> software inhibits future dispatch -> HOLD response false.

Most important unresolved control-side hypotheses: unwrapped Euler delivery semantics through the vendor ServoL interface (the onset is near C=-180), the independent virtual-state tracker vs real feedback, vendor retargeting under a 0.7sec target_time with 20Hz updates, and IK/joint feasibility. These are hypotheses, not identified vendor bugs. The driver source simply copies pos/vel/acc/time into Drfl->servol without a quaternion conversion or per-command execution acknowledgement. A matrix-equivalent ABC choice is not proof of vendor joint-path equivalence.

The tracker uses persistent virtual pose and velocities, while each new policy target is derived from real CurrentPosX. It has no current ServoL following-error inhibit or joint-range forecast at the publish boundary. That allows the discrepancy to grow despite a valid raw-action gate. SpeedL forecast code elsewhere is not attached to this ServoL path.

Camera/JointState loss did not trigger this incident: JointState max source gap=12.397ms, receive gap=36.462ms. All 224 predictions were accepted. No raw rotation/workspace rejection ended the trial. Maximum stream tick gap=65.455ms. Detailed controller alarm, per-command vendor receipt and IK branch are not logged, limiting root-cause certainty.

## Required disposition

Do not treat this run as a safe-motion PASS or fix it by expanding limits further. Preserve the controller alarm from pendant/driver logs before recovery. Investigate vendor Euler range/retarget semantics and reproduce the saved stream offline first; implement validated following-error and joint-feasibility inhibition before further physical continuation. No automatic recovery was performed by this audit.

## Artifacts

- `incident_summary.json`: computed extrema/times/caveats.
- `feedback_command_timeline.csv`: all aligned prediction/pose/requested-command comparisons.
- `jointstate_last_5_seconds.csv`: name-reordered joint position/velocity samples.
- `source_sha256.json`: original evidence hashes.
- Reproduce with `/usr/bin/python3 audit_servo_190404_incident.py`; output folder must not already exist.
