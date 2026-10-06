# Integrated real readiness — latest verdict: BLOCKED

## ROS / DDS

RMW: rmw_fastrtps_cpp; ROS_DOMAIN_ID unset/default 0; ROS_LOCALHOST_ONLY=0. Driver/daemon environment reads showed no explicit FastDDS/Cyclone config or domain mismatch. NIC enp3s0=192.168.0.100/24, Wi-Fi=192.168.1.98/24; both UP/MULTICAST. This flag is not a multicast traffic test. Driver network 192.168.0.0/24 has a connected route. No daemon or driver restart was performed.

Direct polling: 359 samples at target200ms; missing publisher polls 0. Node metadata may be UNKNOWN even with an endpoint. CLI currently identifies joint_state_broadcaster,/dsr01 and agrees publisher count1; earlier historical disagreement remains preserved. No current CLI_DAEMON_DISCOVERY_INCONSISTENCY proved in this snapshot. Service endpoints list_controllers/list_hardware_interfaces appeared in direct graph, types are recorded in timeline; GID unavailable in installed rclpy graph API. Endpoint presence is not a successful service reply or controller connection confirmation. No new service invocation needed/attempted.

## JointState

Clean gate: JOINTSTATE_CLEAN_GATE_FAIL. Required first-normal-sample +30s / max discovery wait60s; the preserved records include initial/cache samples rather than silently deleting them. A cached first sample followed by source/receive gap fails this gate. Invalid finite/name counts are 0. Both QoS compatible with offered RELIABLE/TRANSIENT_LOCAL.

```json
{
  "best_effort": {
    "count": 866,
    "coverage": 26.955627616029233,
    "rate": 32.08977406579194,
    "max_source_gap": 3.070023775100708,
    "max_receive_gap": 3.0722382497042418,
    "latest_age": 0.009550637099891901,
    "invalid": 0,
    "duplicate": 0,
    "regression": 0,
    "first_receive_delay": 15.320367093198001,
    "pass": false
  },
  "reliable": {
    "count": 867,
    "coverage": 30.018788233865052,
    "rate": 28.848599525514512,
    "max_source_gap": 3.070023775100708,
    "max_receive_gap": 3.0723345652222633,
    "latest_age": 0.009435095358639956,
    "invalid": 0,
    "duplicate": 0,
    "regression": 0,
    "first_receive_delay": 12.257322017103434,
    "pass": false
  }
}
```

Simultaneous rosbag samples 3916. Exact shared >=100ms source-gap pairs across BE/reliable/bag: 2. Details in jointstate_gap_comparison.json. This is not proof of controller generation failure, because all recipients share the same host/RMW. No new driver log bytes during integrated window: no new Skip-dt can be causally correlated. Host load/memory/cpu/network/driver scheduling raw data stored at target200ms in host_stats.csv. No isolated controlled host-load intervention was performed; correlation is not causality.

The subsequent passive follow-up received 1135 samples, but coverage/first discovery did not establish a complete20s gate. This newer observation does not replace the failed clean trial.

## Camera / OpenVLA

Camera30s: FAIL, 66 predictions / 30.034s, selected-frame max age 0.526089s; camera_failure 2. Threshold0.5s unchanged. BGRA1280x720→RGB fast decoder→JPEG quality95→verified processor; source/RGB/JPEG hashes stored.

GPU/model health: MODEL_HEALTH_PASS; checkpoint8130, K1, dim7, no proprio; strict processor metadata checked before fresh frame, lightweight unchanged-health check before subsequent snapshots. Sample real predictions66, fixture forbidden. Translation max 2.369mm, rotation max 0.889deg, close candidates 0, critical magnitude anomalies 0. Compared with earlier~2.5mm/~0.6deg, rotation max rose but stays below4deg limit; this is not task-success evidence.

Since JointState failed, continuing camera/model diagnostics was vision-only, not integrated motion-ready Shadow. SafetyPipeline received joint_state_ok=false when unavailable and tcp_ok=false always; invalid placeholder context is explicitly NOT current TCP. All predictions stayed command-disabled. Full hardware watchdog/operator state was NOT verified; no PASS is claimed for integrated Shadow.

## TCP / robot hardware

Active TCP/tool UNKNOWN, offsetNOT_FOUND/current controller Cartesian poseNOT_FOUND. TCP_FLANGE_ONLY is available FK capability, not fresh measured TCP. Current passive TF/robot_description/state-topic evidence in tcp_source_followup.json; unrelated SO101 TCP frames do not establish active Doosan TCP. No historical Tool_v1 or zero offset applied.

Name getters are source getter-only but vendor internal stability unknown. Joint clean gate FAIL and controller connection UNKNOWN violate invocation prerequisites: getter requests0, no unstable posx call. No setter was used. Current config search found no proven active-name/6D-offset binding; no active name was obtained. Controller/mode/authority/servo/protective/emergency state remains UNKNOWN. Event silence is not affirmative connection/safety.

## Manual-only / minimum-motion

Operator present, E-stop reachable, workspace physically clear: MANUAL_CONFIRMATION_REQUIRED; not requested mid-task, not auto-approved. Watchdog and logger conditions do not grant motion authority in absence of all gates.

Minimum-motion authorizedNO, executedNO, physical pose commands0. Planned protocol baseX+0.5mm/no rotation/gripper disabled, requested/observed delta null (never sent). ResultBLOCKED. Short/full/OFT not executed.

## Final

Next allowed stage: BLOCKED. Remaining blockers: JointState clean continuity/discovery, camera selected-frame0.5s violations, current TCP/tool transform, affirmative hardware states, physical operator/workspace/E-stop confirmations. Software diagnostics completed without weakening gates. Robot/motion/gripper/Home/trajectory/Hold/Stop/rollout physical calls all0.

Artifacts are new, historical logs retained. Runtime source/safety configs unchanged. Script compile checks PASS; no full regression-suite claim. Monitoring/CLI/rosbag themselves add host load, so root cause remains unresolved rather than inferred from these traces.
