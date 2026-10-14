# Real OpenVLA action distribution — prediction only

Attempts 50; inference requests 50; valid 50; invalid 0; preflight rejected 0. Physical commands **0**. Samples are real ZED 1280x720 bgra8 -> RGB -> JPEG, OpenVLA step8130 K1 dim7 proprio=false. Strict identity verified at startup; unchanged identity checked before each request. REAL/AUTO/STANDBY scalar read-only responses refreshed before every request. Start-pose error <=0.5 degrees checked each time. JointState and camera use the same persistent subscribers throughout. JointState events 0, runtime fault null. Source/model hashes and camera stamps are all independently audited in sample_integrity_audit.json.

## Distribution

Translation mm: min 11.220949; median 12.875387; mean 12.412145; std 0.742841; p75 12.875387; p90 12.875387; p95 12.875387; p99 12.875387; max 12.875387.

Over 10 mm 50/50 (100.0%); over 12 mm 36/50 (72.0%); over 15 mm 0/50 (0.0%). These remain SAMPLE_VALID model observations; safety acceptance is recorded separately, and none were removed because of magnitude.

Raw rotation norm degrees: median 0.056348; p95 1.827170; max 1.827170. Rotation hard gate remains 4 degrees; rejects 0. This is the norm of the raw action rotation vector, not a measured robot orientation delta.

Gripper closedness: min/median/mean/p90/max 0.998039/0.998039/0.998039/0.998039/0.998039. Open-like (<0.3) 0; middle [0.3,0.7] 0; close-like (>0.7) 50. All-close prediction is a separate behavior concern, not resolved by raising translation gate. No physical gripper output.

Inference seconds: median 0.412447; p90 0.434169; p95 0.444245; p99 0.466069; max 0.471677. Camera receive age at request start seconds: median 0.024419, p95 0.034093, max 0.057420. End ages and ROS source ages retained separately, never substituted into the start-age gate.

## Threshold candidates — NOT applied

Current 10 mm rejects 100% of this sampled scene. Empirical p95, p99 and max are all 12.875387 mm and all give zero observed rejection under the existing inclusive boundary semantics. Repeated token/action levels cause tied quantiles; they must not be interpreted as a precise population upper bound. The observed maximum occurred 36 times, rather than being a lone spike.

Recommended **candidate**: 13 mm, empirical p99 rounded upward to whole-mm reporting precision (headroom 0.124613 mm). This is a distribution-coverage proposal, not a physical safety-approved limit. No additional max-based margin is invented. Applied production threshold remains **10 mm**. The new candidate requires explicit review/approval and fresh validation before changing production configuration.

Downstream limits remain translation velocity 20 mm/s and acceleration 20 mm/s² at 5 Hz; implied speed-only step ceiling 4 mm and first-step acceleration-limited translation 0.8 mm. Existing limiter can modify raw output. Raising raw rejection threshold does NOT remove these limits or make delivered action equal raw action. No clipping/scaling occurred in this prediction-only collection.

## Independence, top actions and caveats

50 unique source hashes, 50 unique model input hashes, 50 unique source timestamps. Fresh-frame uniqueness is verified, but the same stationary scene is temporally correlated: these are NOT IID independent robot tasks or multi-scene action coverage. Distinct raw action vectors 9. All samples and top five JPEG-decoded RGB PNGs retained; the top PNGs derive from actual model JPEG inputs, not original uncompressed RGB pixels. No outliers removed.

The representative top image shows the tabletop/block scene and gripper. This static sample does not establish why the model requests the observed motion/closure, whether behavior is task-appropriate, or how the policy changes after motion. No claim of safe trajectory/task success can be made from this distribution.

TCP remains TCP_ZERO_OFFSET_SUPPORTED_NOT_VERIFIED; physical rollout readiness **BLOCKED_TCP** plus unresolved physical stage/hardware/operator and gripper behavior requirements. Readiness is not promoted to READY_AFTER_TCP_APPROVAL solely from these model statistics. Latest real hardware state/sensor checks here are collection preflight evidence, not motion authorization.

New distribution tests: 4 PASS. No motion/action/gripper/Stop/Hold client exists in the collector. No driver/controller/server restart. Observation logs always command_requested=false, command_issued=false, delivered_action=null, executed_action=null.
