# Live prediction-only batch

`run_live_prediction_batch.py` uses real JointState, ZED images and OpenVLA HTTP
prediction. It creates one ROS node and two persistent subscriptions. It does not
import or instantiate a robot command sink or create ROS service/action clients.
The existing driver, camera and model server are not restarted or modified.

Each attempt creates new readiness state on the same subscriptions:
FIRST_FRESH_SAMPLE (valid six joints, finite values, ROS header age <100 ms),
10-second warm-up, then runtime. Startup/warm-up samples are retained separately.
The trial window is not restarted on failure. An independent monitor runs during
HTTP inference; invalidation prevents further inference reservation. Responses to
already in-flight requests are discarded from performance and retained separately.

## Runtime completion is not a task outcome

Normal prediction-only completion has `runtime_status=VALID`, `runtime_valid=true`,
`status=null`, `task_success=null`. INVALID has `status=INVALID`,
`runtime_status=INVALID`, `runtime_valid=false`, `task_success=null`.
This is a separate runtime aggregation schema: it never assigns task SUCCESS or
FAILURE and cannot be mixed into the physical-task success-rate denominator.

The raw translation 4 mm and rotation 4 degree hard limits retain existing values
and terminate the prediction trial on exceedance. Sensor/runtime thresholds are
unchanged: JointState gaps <100 ms, latest age <500 ms, selected camera age <0.5 s.
Camera encoding/1280x720 resolution are checked; BGRA8 is decoded to RGB before
JPEG encoding and OpenVLA preprocessing. Runtime normality alone does not validate
TCP-dependent workspace, velocity/acceleration under physical dispatch, gripper
phase/state or hardware authorization. These remain physical rollout gate checks.
No unknown tool offset is replaced with zero, and no TCP pose is fabricated.

## Execute

From this directory, with the existing live driver/camera/model server running:

```bash
source /opt/ros/humble/setup.bash
source /home/ubuntu/robot_ws/install/setup.bash
/usr/bin/python3 -u run_live_prediction_batch.py \
  --output live_batch_NEW_TIMESTAMP \
  --target-valid-trials 10 --max-attempts 20 --duration 20
```

Existing output paths are refused. Every INVALID attempt is preserved. Images
are the exact JPEG bytes submitted to the model; no post-hoc re-capture is used.
Per-trial startup/warm-up and runtime JointState sample streams are saved.
Action distribution uses ONLY runtime-valid trials. Invalid action diagnostics
must be reported separately, never substituted for an empty valid distribution.

Batch PASS requires ten runtime-valid trials, no model failures and no fatal
runtime error. PASS does not authorize motion: current TCP/tool, hardware-state
and operator/workspace/E-stop confirmations remain independent gates.
