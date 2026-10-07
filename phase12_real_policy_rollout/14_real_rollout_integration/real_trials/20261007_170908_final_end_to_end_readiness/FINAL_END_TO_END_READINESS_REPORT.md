# Final end-to-end readiness — 20261007_170908_final_end_to_end_readiness

Latest verdict: `BLOCKED_MULTIPLE`. Physical robot commands **0**. Driver/controller not restarted; thresholds unchanged. Minimum-motion, model rollout, gripper, Home, trajectory and physical Hold/Stop not executed.

## JointState

Fixed baseline: PASS; 180.0 s. Messages 18000; rate 100.00003088348159 Hz. Max source/receive gaps 0.012284994125366211 / 0.06550357304513454 s. ≥100 ms source/receive events 0 / 0. Invalid 0, missing 0, duplicate 0, regression 0.

FIRST_FRESH→10 s warm-up is retained; startup/warm-up events are preserved separately in all samples. Runtime faults latch; no failed window reset. Same subscriber continues into Shadow. Shadow JointState: FAIL. This is current evidence, not a reused PASS artifact.

## Camera / OpenVLA

Current strict GPU health: MODEL_HEALTH_PASS. OpenVLA step8130, K=1, dim=7, proprio=false. Actual server is RTX3090; no fixture prediction. One first-trial prediction is saved separately (server health initially request_count=0), then 10 s model warm-up, then fixed 120 s Shadow.

Shadow attempts 292, successful predictions 291; model failures 0. Selected-camera maximum age 0.494729561265558 s; age-threshold failures 0, input failures 1, total camera failures 1; camera FAIL. A missing ≤10 ms fresh snapshot is a camera input failure, not a GPU inference failure; initial automatic aggregate is preserved separately. BGRA8→RGB uses actual encoding/row stride then JPEG95→processor. No heavy health, graph/CLI, SHA or image disk writes before inference; background logger saves image/hash after response. Selected-age includes inference latency, not just snapshot age.

Translation median/p95/max: 0.605877713859466 / 3.451404485655279 / 4.613487840673505 mm. Rotation max 3.743540020841421 deg. ≥0.7 close candidates 0. >4 mm 15; >4 deg 0. Outlier records retain raw vector, frame image, JointState, timestamps and latency. Model failures include any nonfinite/schema failure and are not hidden.

## Action distribution interpretation

Training: 11 unique episodes / 539 steps, 10 Hz, 10 corrective +1 nominal. Translation median/p95/p99/max: 7.095518739607351 / 29.14799397261564 / 37.64361539449053 / 48.31506790879339 mm. Live max inside observed training range: True.

4 mm is the configured fail-closed **raw action step acceptance limit**, not a manufacturer physical limit and not merely a diagnostic plot threshold. An action around 4.6 mm is not outside the observed training range, but remains rejected by current command safety. No threshold tuning performed. Training sample checkpoint membership is unverified; differing frequency, unmatched views and phases forbid causal/distribution-equivalence claims or direct implied-velocity comparison. Minimum-motion uses a separate deterministic 0.5 mm action, never model output.

Temporal adjacent translation Δaction statistics: {"count": 290, "mean": 0.596247412116568, "std": 1.2908389546164085, "median": 0.0, "p90": 2.46750359506155, "p95": 4.049785815623917, "p99": 4.049785815623917, "max": 4.049785815623917}. >4 mm runs: [[8], [10], [92], [102], [125], [153], [178], [184], [198], [205], [228], [243], [284], [288], [292]]; isolated runs: 15. Images are retained; no visual causal claim made from action timing alone. No physical gripper output; training close timings are descriptive, not a current physical phase.

## TCP / hardware

TCP classification: TCP_FLANGE_ONLY. Active TCP/tool UNKNOWN; offset/current TCP unverified. Existing empty-name getters not repeated. FK is base_link→link_6 only, never measured TCP. Tool_v1 zero offset is historical, not current binding.

RobotState/RobotStateRt schemas contain relevant fields but current graph produced no verified live samples. Optional RT publisher configuration is disabled; no parameter mutation/RT start was performed. Flange cache getter lacks public timestamp/sequence: no freshness proof, not called. Pointer-risk current Cartesian and LastAlarm getters not called. No getter refresh without stable current JointState evidence.

Mode/state/system prior AUTO/STANDBY/REAL remain timestamped previous evidence only. Current connection/authority/servo/protective stop/E-stop UNKNOWN. STANDBY and event silence do not establish those states. Gripper physical state UNKNOWN; output and pulse capability absent in this audit.

## Runtime / research logs / manual gates

Independent JointState watchdog: FAIL. Camera failure checked at each response; this measurement does not validate a physical abort/stop response. Logger: PASS; session COMPLETE; records 293. Same sensors persist; failures do not authorize motion. Host sampled at 1 Hz with CPU/network/process counters; gap correlation is descriptive, no unsupported root-cause attribution. No runtime ROS graph/CLI polling.

Image IDs, source/model JPEG hashes, timestamps, instruction, action, JointState and latency retained. Observation Gap join fields null where unavailable; no invented matched pairs or phase labels.

Operator presence, workspace physically clear and physical E-stop reachable: MANUAL_CONFIRMATION_REQUIRED. They are not the only blockers.

## Final gate

Remaining blockers: TCP_CONTRACT_UNVERIFIED, HARDWARE_STATE_UNKNOWN, JOINTSTATE_RUNTIME_FAIL, CAMERA_RUNTIME_FAIL, RAW_ACTION_STEP_LIMIT_EXCEEDED, MANUAL_SAFETY_CONFIRMATIONS_REQUIRED.

Next allowed stage: BLOCKED. No minimum-motion execution; no short-horizon/full-task execution. Resolve current stream/latency and verified TCP/hardware evidence, then obtain manual confirmations and re-evaluate fresh gates. Do not bypass 100 ms JointState /0.5 s camera thresholds.
