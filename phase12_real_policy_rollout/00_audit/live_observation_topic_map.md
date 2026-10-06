# Live observation topic map

| Source | Status | Semantics |
|---|---|---|
| ZED rectified RGB | available, 1280x720, source header | measured camera observation |
| `/dsr01/joint_states` | previously reported ~100 Hz; unavailable during this recovery run (publisher 0) | measured position/velocity when broadcaster is live |
| Joint ordering | `1,2,4,5,3,6` on wire | must reorder by name to canonical `1..6` |
| Joint effort | NaN | unsupported/not available; does not invalidate finite position/velocity |
| `/doosan/current_pose` | absent | no measured TCP topic |
| GetCurrentPosx | verified getter but timed out, then disappeared | intended measured controller feedback in DR_BASE |
| Model servers | not running | prediction not attempted |

The driver/controller services and JointState publisher disappeared during the audit. Final graph contained only robot_state_publisher/RViz/launch and ZED nodes. This triggers the communication-loss stop condition for observation collection.

On the later stability run, publisher/service endpoints stayed discoverable but JointState messages covered only 11.95 seconds of a 30.01-second observation. Required node discovery failed in 7/31 polls. Endpoint presence is therefore not accepted as continuous data-path health.
