# JointState revalidation

Classification: JOINTSTATE_INCONCLUSIVE. Motion remains prohibited.

| Run (60 s each) | Messages | First receive delay s | Steady receive Hz | Max source gap s | Source gaps >=1 s |
|---|---:|---:|---:|---:|---:|
| run1 | 4349 | 10.425179711077362 | 87.69805864991083 | 3.060014247894287 | 2 |
| run2 | 4809 | 2.7605595770291984 | 83.99725248207164 | 3.070002555847168 | 3 |
| run3 | 4473 | 9.18693088926375 | 87.9973153905839 | 3.060013771057129 | 2 |

Full-window and steady-state statistics, all receive/source timestamps and joint vectors are retained in CSV/JSON. No camera subscription or ROS graph polling ran during these measurements. ZED launch PID 3017687 was stopped with SIGINT; Doosan PID 3017586 was not restarted. No model/rollout process was discovered. Other ROS infrastructure (RViz, robot_state_publisher) was not stopped.

Camera-on comparison was not executed because the required three stable JointState-only runs did not pass. Camera remains stopped pending a separate restart; no automatic driver recovery was performed.

External mode/movej driver callback logs in measurement interval: True. These are not calls made by this measurement. Physical result and initiating client are unknown. This violates a strictly uncontrolled stationary-baseline assumption and prevents unqualified causal classification. Source discontinuities establish a failure of the delivered timestamp sequence, but do not alone distinguish controller feedback stall, driver update stall, DDS/sample loss or observer scheduling. BEST_EFFORT is compatible with advertised RELIABLE/TRANSIENT_LOCAL; loss remains possible. No QoS tuning was performed.

Driver CPU snapshot: 202% process lifetime average, RSS 118500 KiB. Host load/memory/network counters are in host_stats.csv; per-second driver process counters were additionally recorded in Run 3. Old startup Skip-dt warnings are not classified as current errors. Current-interval lines are preserved separately. DDS controller node names were UNKNOWN/missing in discovery despite service endpoints; this does not prove process exit.

Timing clarification: the mode/movej callbacks (1791278478.174 / 1791278479.700) occurred BETWEEN Run 1 (1791278376.315–1791278436.320) and Run 2 (starting 1791278517.809), not inside an individual 60-second acquisition. They still alter the same-condition/stationary-baseline assumption across runs. No causal link to any particular gap is established. End-of-test driver PID was unchanged, CPU lifetime average 203%, RSS 118608 KiB. Post-test publisher/QoS remained identical to the initial snapshot.

Next: stop external command sources with operator coordination, preserve driver logs, correlate gap_events.csv with hardware read/update/monitor traces and network counters, and compare an independent reliable subscriber/bag under an explicitly documented QoS. Do not call unstable TCP getter, model rollout, or motion stages on this evidence.

All robot/motion/gripper/Home/trajectory/Hold/E-stop/real rollout calls made by this audit: 0. This cannot certify zero external command activity.
