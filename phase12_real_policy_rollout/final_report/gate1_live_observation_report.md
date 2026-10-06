# Gate 1 live observation recovery report

Final status: `BLOCKED_REQUIRES_LOCAL_OPERATOR_AND_HARDWARE`; motion readiness false.

Service definitions were recovered through source `.srv` files and generated Python introspection. `ros2 interface show` fails on a generated `//` comment parser defect. Driver callbacks prove the six allowlisted interfaces are getters only.

Six allowlisted read-only requests were sent once each. Control mode=3, control space=1 and last-alarm response succeeded. TCP, robot state and robot mode timed out. Before the isolated TCP retry, all Doosan getter services disappeared; the retry stopped at service discovery and sent no request. JointState also lost its publisher. Consequently measured TCP and a stationary synchronized record were not obtained.

TCP expected contract is raw x/y/z mm, raw A/B/C degrees, DR_BASE, solution-space integer, controller feedback source, no source timestamp, separate ROS/monotonic receive timestamps. No value was fabricated and orientation remains unresolved.

Added Phase 12-only read-only service adapter, name-based JointState ordering, effort-NaN handling, passive stability monitor and allowlist tests. Offline suite: 19 passed. Stationary samples: 0. Live model inference: 0.

Safety counts: Robot command 0; motion service/action 0; gripper/Home/trajectory 0; AI action publish 0; Hold/E-stop call 0; closed-loop 0. Read-only getter requests: 6 total, one per allowlisted service. Executed action and robot-delivered command remain null by contract.

Next Gate is not permitted. Restore the controller/state broadcaster under local operator supervision, verify robot state/mode/TCP and hardware safety signals, then rerun stationary observation only.

## 30-second stability follow-up

The later passive stability gate also failed. It received 1,196 finite JointState samples at 99.9997 Hz during an 11.9500-second source-timestamp span, only 39.82% of the 30.0107-second wall interval. Timestamps were monotonic with zero duplicates; wire joint order was `1,2,4,5,3,6` and all 1,196 effort arrays were treated as unsupported. Three getter services and a JointState publisher endpoint appeared in all 31 polls, but both required controller/broadcaster nodes were absent in 7 polls. Initial controller listing was delayed and hardware-component listing emitted a 10-second timeout warning.

Because the stability gate failed, current_posx, robot-state and robot-mode requests in this follow-up were all **0**. Measured TCP/state/mode remain unavailable, and the stationary observation test remains not executed. Status: `DOOSAN_STATE_STACK_UNSTABLE`.
