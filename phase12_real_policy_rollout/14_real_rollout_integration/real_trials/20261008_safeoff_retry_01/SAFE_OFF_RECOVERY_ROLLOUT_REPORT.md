# SAFE_OFF recovery retry

SAFE_OFF recovered: YES, read-only driver responses REAL/AUTO/STANDBY, success=true each. No setter or motion was used for recovery.

Start position match: PASS. Maximum error 0.140187 degrees <= existing 0.5-degree tolerance, reordered by joint names. Reset intentionally skipped, physical commands=0.

JointState readiness: PASS, consecutive 10 clean seconds, no runtime fault. Runtime including the post-getter watch and one prediction: 11.348 seconds, 1,136 samples; maximum source/receive gap 12.487/17.978 ms, >=100 ms events=0. DDS root-cause analysis was not repeated.

OpenVLA health/processor identity: PASS. Exactly one prediction on real ZED 1280x720 BGRA8 -> RGB -> JPEG input completed successfully with finite seven-dimensional output. No physical model command was requested or issued; delivered/executed action null. Source image and exact JPEG hashes, RGB PNG and model-input JPEG are preserved.

Prediction preflight: FAIL. Inference latency 0.769390 seconds, selected-frame receive-to-response age 0.802849 seconds >= unchanged 0.5-second threshold. This was the first sample after server restoration; whether warm calls meet the threshold is not established by this one call. No automatic second prediction was made.

Raw translation XYZ in metres: [0.0014968155858540912, 0.0026335443427412267, -0.007474322441489991], norm 8.064832 mm. Raw rotation [ -0.008621855310862958, 0.002364704483548279, 0.014905046792334375 ], norm 0.017380697 rad. Gripper closedness 0.0019607843. Translation exceeds existing 4mm gate; this is recorded as a diagnostic, not a task FAILURE or proof of a current physical safety evaluation with unknown TCP. No clipping/scaling/limit change was applied.

TCP/action contract is still unverified, TCP_FLANGE_ONLY. No zero offset/flange=TCP assumption is made. Short-horizon NOT_STARTED; command count 0; task_success/runtime_valid null because no physical task trial began. E-stop/unexpected-motion results N/A. Next: FIX_TCP_CONTRACT and resolve current camera/action preflight blockers before physical rollout.
