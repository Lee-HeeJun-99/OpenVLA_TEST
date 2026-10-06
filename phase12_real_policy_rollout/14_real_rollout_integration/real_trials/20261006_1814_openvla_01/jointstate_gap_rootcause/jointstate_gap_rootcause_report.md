# JointState gap root-cause comparison

Verdict: STILL_INCONCLUSIVE. Feedback continuity FAIL; no motion authorized.

Two 120-second acquisitions were actually performed. Driver PID3017586 was not restarted. Camera/model/rollout processes were not running. Command-capable candidates were inspected; no ActionAdapter/DoosanBridge/teleop/model runner process was found. /so101_usd_dashboard existed in discovery without an identifiable matching host process; remote client absence cannot be proved. Operator no-command confirmation was requested but not automatically asserted. Runs with new movej/movel/mode/tool-output callback logs are invalid; see run metadata. No command-capable process was automatically killed.

| Run | Stream | Messages | First receive delay s | Source Hz after first sample | Max source gap s | Source gaps >=1s |
|---|---|---:|---:|---:|---:|---:|
| run1 | best_effort | 5588 | 12.192050457000732 | 50.4059784042207 | 3.070021152496338 | 18 |
| run1 | reliable | 5587 | 12.192384958267212 | 50.4015153276373 | 3.070021152496338 | 18 |
| run1 | bag | 5558 | 61.38033986091614 | 90.07938441180687 | 3.0700113773345947 | 2 |
| run2 | best_effort | 6053 | 10.623080492019653 | 53.824257464224814 | 3.0700135231018066 | 17 |
| run2 | reliable | 6052 | 10.62326955795288 | 53.8201503940373 | 3.0700135231018066 | 17 |
| run2 | bag | 6030 | 50.56553816795349 | 86.82316048581866 | 3.060014009475708 | 3 |

Matching source-before/source-after intervals across all three streams: 5. All gap intervals (>=100ms) are in gap_correlation.csv; bag entries outside actual bag receive coverage are not evidence of bag-normal delivery. Source and receive distributions, >=100/500ms/1/2s counts and duplicate/regression counts are in summary.json. Subscriber sequence is local receive order, not publisher sequence.

BEST_EFFORT and RELIABLE share a lightweight Python executor; rosbag is a separate process. Common gaps across both reliable receivers and BEST_EFFORT support a common upstream or shared middleware path, not exclusively BEST_EFFORT loss or a single Python observer. All still share host/RMW. Without publisher/update/RT instrumentation, direct driver/controller stall versus shared DDS loss is not proven.

Header timestamps come from broadcaster update time (see publisher_source_audit.md), not controller sampling time. The distinction is essential: source jumps represent missing/delayed generated update stamps in the delivered stream, not proof of controller hardware packet interruption.

## Host/network

200ms snapshots contain CPU, memory/load, network RX/TX/error/drop counters, driver CPU ticks and main-thread scheduling counters. Per-run counter changes/maxima:

run1: {"network_counter_delta": {"lo": {"rx_bytes": 10185297, "rx_errors": 0, "rx_drops": 0, "tx_bytes": 10185297, "tx_errors": 0, "tx_drops": 0}, "enp3s0": {"rx_bytes": 163030948, "rx_errors": 0, "rx_drops": 0, "tx_bytes": 813948, "tx_errors": 0, "tx_drops": 0}, "enp6s0": {"rx_bytes": 5652, "rx_errors": 0, "rx_drops": 0, "tx_bytes": 0, "tx_errors": 0, "tx_drops": 0}, "wlp4s0": {"rx_bytes": 5512965, "rx_errors": 0, "rx_drops": 0, "tx_bytes": 8221461, "tx_errors": 0, "tx_drops": 0}}, "driver_cpu_percent_mean": 203.7728774987608, "driver_cpu_percent_max": 209.40026605075766, "host_cpu_busy_percent_max": 34.335839598997495, "driver_main_thread_sched_wait_delta_max_s": 0.000624456, "host_sample_max_gap_s": 0.20214214315637946}
run2: {"network_counter_delta": {"lo": {"rx_bytes": 7397177, "rx_errors": 0, "rx_drops": 0, "tx_bytes": 7397177, "tx_errors": 0, "tx_drops": 0}, "enp3s0": {"rx_bytes": 163118224, "rx_errors": 0, "rx_drops": 0, "tx_bytes": 829908, "tx_errors": 0, "tx_drops": 0}, "enp6s0": {"rx_bytes": 744, "rx_errors": 0, "rx_drops": 0, "tx_bytes": 0, "tx_errors": 0, "tx_drops": 0}, "wlp4s0": {"rx_bytes": 5742911, "rx_errors": 0, "rx_drops": 0, "tx_bytes": 6186475, "tx_errors": 0, "tx_drops": 0}}, "driver_cpu_percent_mean": 203.74329740947786, "driver_cpu_percent_max": 209.39348245513023, "host_cpu_busy_percent_max": 60.69651741293532, "driver_main_thread_sched_wait_delta_max_s": 5.624e-06, "host_sample_max_gap_s": 0.20222872402518988}

Counters are host-interface counters, not controller-link packet capture or switch/controller counters. Main-thread schedstat does not measure every driver worker thread. No causal network/CPU claim follows solely from a matching peak. Gap events and raw host timestamps allow further comparison. Driver-event correlation includes the gap duration plus +/-1s using receive-wall times; ROS log clock equality was not independently validated. NO_NEW_DRIVER_LOG does not mean no internal stall, especially when tracing is disabled. Historical startup Skip-dt logs were excluded using file offsets.

## Answers and next gate

1. Source-header discontinuities and reliable/bag correspondence: numerical results above; ordinary 10ms samples coexist with multi-second missing intervals.
2. Driver-log synchronization: inspect driver_event_correlation.csv; absence of event logs cannot localize vendor internals.
3. Host/network relation: recorded but no proven causal association. QoS publisher configuration was not changed.
4. JointState is not currently accepted as a rollout feedback source: repeated >=1s gaps fail continuity.
5. Next read-only experiment should observe publisher pre-publish/update and hardware read/RT packet counters on the same monotonic timeline alongside reliable bag, under operator-confirmed no external callers. This distinguishes publish omission/trylock/update stall from downstream middleware starvation. No getter or driver recovery is authorized by this report.

Actual robot/motion/gripper/Home/trajectory/Hold/E-stop/real rollout calls by this audit: 0. External activity is reported separately and cannot be certified globally. Production thresholds and driver sources unchanged.
