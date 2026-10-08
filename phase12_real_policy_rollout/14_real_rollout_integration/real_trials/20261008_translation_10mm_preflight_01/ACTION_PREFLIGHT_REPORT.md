# 10 mm translation hard rejection gate

Branch lhj-research; baseline HEAD ba94b54cdc38de34f493df903e63c8a1a85a9466. Existing uncommitted camera timing changes were preserved. Historical logs and offline analyses were not rewritten.

## Production change

01_configs/safety_limits.yaml translation_step_m changed from 0.004 to 0.010 m. The shared action_step_limits module reads this config; the actual Controller -> SafetyPipeline -> CommandDisabledPolicyRuntime path and prediction-only batch use it. RuntimeSafetySupervisor receives the same translation step value. command_disabled_runtime.yaml records the same raw threshold. Flange Shadow anomaly classification follows the current gate. Legacy >4 mm batch distribution count remains a historical diagnostic, alongside a new current-limit count.

Exact 10 mm is accepted, with the pre-existing 1e-12 m numerical tolerance; 10.1 mm is rejected and classified INVALID_SAFETY_REJECTION, then commands remain inhibited. No new clipping/scaling or normalization changes were added. Rotation 4 degrees, runtime timing/watchdog, camera start-age 0.5 seconds, workspace, velocity/acceleration and gripper policies are unchanged. The velocity-derived 0.004 m ceiling in the offline candidate config remains mathematically correct for its unchanged 20 mm/s at 5 Hz; it is not the raw rejection threshold.

Important: an existing downstream velocity/acceleration limiter already modifies candidate actions. It is NOT removed by this threshold-only change. Passing the raw gate must not be claimed as proof that a future delivered action equals the original action. This requires a separately authorized policy decision before any such physical evaluation.

## Current live sample: exactly one prediction

- Hardware REAL / AUTO / STANDBY, current safe read-only getters once each.
- Start pose within existing 0.5 degree tolerance; no reset motion.
- JointState RUNTIME_READY, no fault; 1,103 post-ready samples / 11.023 s; max source gap 13.223 ms, max receive gap 34.272 ms; >=100 ms events 0.
- OpenVLA current strict model identity PASS, finite 7D real response.
- Camera 1280x720 bgra8 -> RGB -> JPEG. Start receive age 22.707 ms; end age 425.963 ms; inference latency 403.256 ms. ROS source ages 308.975 / 712.273 ms are retained as separate diagnostics.
- Raw translation [0.114736, 4.007271, -4.330878] mm; norm 5.901515 mm <10 mm: ACTION_TRANSLATION_GATE_PASS.
- Raw rotation [0.000578725, 0.004899442, 0.011015418] rad; norm 0.012069750 rad (~0.6915 degrees): raw rotation gate PASS, existing 4 degree limit untouched.
- Gripper closedness 0.001960784. No gripper output.
- Previous 7.880887 mm prediction also passes the new raw translation criterion; its historical artifact is unchanged.

Prediction image, exact JPEG, source/model hashes and timestamp details are retained in sample_prediction.json. No model output was clipped/scaled in this sample; no physical candidate was delivered.

## Final

CAMERA_PREFLIGHT_PASS. MODEL_ACTION_PREFLIGHT_PASS (raw translation/rotation, valid model response, clean concurrent inputs).

TCP_CONTRACT_FAIL: active TCP/name/offset/current pose remain unverified. Context is FK_ESTIMATED_FLANGE only. No zero-offset or flange=TCP assumptions. PHYSICAL_ROLLOUT_BLOCKED; short horizon NOT_STARTED. Fresh TCP/hardware/manual/stop-path/stage preflight is still required before physical dispatch; passing this limited preflight is not motion authorization.

Physical commands 0. Prediction requests 1. Driver/controller/camera/model server not restarted. No synthetic task success/failure assigned.
