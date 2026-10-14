# Historical TCP tolerance validation

Independent valid historical FK/controller pose pairs: **0**.

Repository JSON/JSONL keyword scan found 6 candidate files. Entries lacking paired measured controller poses cannot become samples. Historical FK validation contains computed flange only. Actual real episode 1-10 metadata/steps were checked: 452 steps total; these use planned/commanded/interpolated poses and expose no measured EE pose feedback. They are not calibration pairs. Simulation, fixture and unavailable/null feedback are excluded. File-by-file reasons and provenance are in historical_pose_samples.json.

Current target is deliberately held out. Its FK-vs-current-PosX translation error is 1.111531477 mm and geodesic rotation error 0.178201019 degrees, recomputed from matrices using intrinsic ZYZ. Controller flange vs PosX is 0.000000000 mm / 0.000000000 degrees. These are one stationary observation, not two independent calibration samples. The cached flange getter has no timestamp and ignores ref, so requested BASE alone is not affirmative cache frame proof.

No existing official FK/controller crosscheck tolerance was found in safety config/source audit. Existing joint-angle tolerance, workspace bounds, action limits, numerical floating-point tolerance and simulator FK tests are not TCP pose error tolerances. No threshold was invented. Translation/rotation distribution min/median/mean/p90/p95/p99/max are null because sample_count=0. Recommended p99/max-based derivation cannot be evaluated without a historical population. Current target cannot calibrate a threshold that then validates itself.

Tolerance: **NOT_DEFINED**. Current translation/rotation acceptance: **INCONCLUSIVE**, not numerical FAIL and not PASS. Classification remains TCP_ZERO_OFFSET_SUPPORTED_NOT_VERIFIED; tcp_contract_verified=false. Active TCP name was not directly observed. Absence of a nonzero transform is not positive binding evidence.

Physical short-horizon: BLOCKED. Independently, the latest prediction-only sample (20261014_after_home_check_01) is 12.214 mm > existing 10 mm and predicts gripper close; this analysis does not override those gates or claim old camera/model/hardware checks are current. No ROS process/getter/request/physical command was executed in this analysis.

Next evidence needed: synchronized, independently measured, valid real FK/controller pose pairs with BASE/unit conventions, freshness and clean-runtime provenance, ideally at multiple stationary configurations, and explicit current TCP binding. A candidate tolerance derived from such evidence must be reviewed before being used as a physical dispatch gate.
