import csv,json,os,re
from pathlib import Path
root=Path(__file__).resolve().parent
runs=[json.loads((root/f'run{i}_summary.json').read_text()) for i in (1,2,3)]
combined=[]; gaps=[]
for i in (1,2,3):
    with (root/f'run{i}_host_stats.csv').open() as f:combined.extend(csv.DictReader(f))
    with (root/f'run{i}_jointstate.csv').open() as f:rows=list(csv.DictReader(f))
    for a,b in zip(rows,rows[1:]):
        sg=float(b['source_ros'])-float(a['source_ros']); rg=float(b['receive_monotonic'])-float(a['receive_monotonic'])
        if sg>=.1 or rg>=.1:gaps.append(dict(run=i,source_before=a['source_ros'],source_after=b['source_ros'],source_gap_s=sg,receive_before=a['receive_monotonic'],receive_after=b['receive_monotonic'],receive_gap_s=rg))
with (root/'host_stats.csv').open('x',newline='') as f:
    keys=sorted(set().union(*(r.keys() for r in combined)));w=csv.DictWriter(f,fieldnames=keys);w.writeheader();w.writerows(combined);f.flush();os.fsync(f.fileno())
with (root/'gap_events.csv').open('x',newline='') as f:
    keys=list(gaps[0]) if gaps else ['run','source_gap_s'];w=csv.DictWriter(f,fieldnames=keys);w.writeheader();w.writerows(gaps)
log=Path('/home/ubuntu/.ros/log/ros2_control_node_3017586_1791277811547.log').read_text()
begin=min(r['start_wall'] for r in runs);end=max(r['start_wall']+r['duration_s'] for r in runs)
lines=[]
for line in log.splitlines():
    match=re.search(r'\[(\d{10}\.\d+)\]',line)
    if match and begin<=float(match.group(1))<=end:lines.append(line)
(root/'driver_log_test_interval.txt').write_text('\n'.join(lines)+'\n')
external=any('movej_cb' in s or 'set_robot_mode_cb' in s for s in lines)
repeated=sum(r['steady_source']['gaps_ge_1s'] for r in runs)>1
classification='JOINTSTATE_INCONCLUSIVE' if external else ('JOINTSTATE_DRIVER_OR_TRANSPORT_UNSTABLE' if repeated else 'JOINTSTATE_STABLE')
summary=dict(classification=classification,runs=runs,external_command_log_detected=external,
    observed_source_discontinuity=repeated,camera_comparison='NOT_EXECUTED_JOINTSTATE_ONLY_FAILED',
    commands_by_this_agent=dict(robot=0,motion_service_action=0,gripper=0,home=0,trajectory=0,hold_estop=0,real_rollout=0),
    external_physical_command_count='UNKNOWN; driver callback logs observed',motion_authorized=False)
(root/'summary.json').write_text(json.dumps(summary,indent=2))
table='\n'.join(f"| {r['run']} | {r['message_count']} | {r['first_receive_delay_s']} | {r['steady_receive']['rate_hz']} | {r['steady_source']['max_gap_s']} | {r['steady_source']['gaps_ge_1s']} |" for r in runs)
(root/'jointstate_revalidation_report.md').write_text(f'''# JointState revalidation

Classification: {classification}. Motion remains prohibited.

| Run (60 s each) | Messages | First receive delay s | Steady receive Hz | Max source gap s | Source gaps >=1 s |
|---|---:|---:|---:|---:|---:|
{table}

Full-window and steady-state statistics, all receive/source timestamps and joint vectors are retained in CSV/JSON. No camera subscription or ROS graph polling ran during these measurements. ZED launch PID 3017687 was stopped with SIGINT; Doosan PID 3017586 was not restarted. No model/rollout process was discovered. Other ROS infrastructure (RViz, robot_state_publisher) was not stopped.

Camera-on comparison was not executed because the required three stable JointState-only runs did not pass. Camera remains stopped pending a separate restart; no automatic driver recovery was performed.

External mode/movej driver callback logs in measurement interval: {external}. These are not calls made by this measurement. Physical result and initiating client are unknown. This violates a strictly uncontrolled stationary-baseline assumption and prevents unqualified causal classification. Source discontinuities establish a failure of the delivered timestamp sequence, but do not alone distinguish controller feedback stall, driver update stall, DDS/sample loss or observer scheduling. BEST_EFFORT is compatible with advertised RELIABLE/TRANSIENT_LOCAL; loss remains possible. No QoS tuning was performed.

Driver CPU snapshot: 202% process lifetime average, RSS 118500 KiB. Host load/memory/network counters are in host_stats.csv; per-second driver process counters were additionally recorded in Run 3. Old startup Skip-dt warnings are not classified as current errors. Current-interval lines are preserved separately. DDS controller node names were UNKNOWN/missing in discovery despite service endpoints; this does not prove process exit.

Next: stop external command sources with operator coordination, preserve driver logs, correlate gap_events.csv with hardware read/update/monitor traces and network counters, and compare an independent reliable subscriber/bag under an explicitly documented QoS. Do not call unstable TCP getter, model rollout, or motion stages on this evidence.

All robot/motion/gripper/Home/trajectory/Hold/E-stop/real rollout calls made by this audit: 0. This cannot certify zero external command activity.
''')
print(json.dumps(summary,indent=2))
