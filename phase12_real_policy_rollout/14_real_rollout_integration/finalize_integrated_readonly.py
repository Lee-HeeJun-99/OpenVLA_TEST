"""Summarize stored audit evidence; no live command or ROS clients."""
import collections,csv,json,re,sqlite3,subprocess,sys
from pathlib import Path
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import JointState
ROOT=Path(__file__).resolve().parent
def main():
    out=ROOT/'real_trials'/sys.argv[1];passive=ROOT/'real_trials'/sys.argv[2]
    def read(path):return json.loads(path.read_text())
    def write(name,data):(out/name).write_text(json.dumps(data,indent=2))
    timeline=read(out/'dds_discovery_timeline.json');samples=timeline['samples'];start=samples[0]['monotonic']
    endpoints=[(r['monotonic'],e) for r in samples for e in r['publishers']]
    gids={str(e['gid']) for _,e in endpoints}
    discovery=dict(rmw=timeline['rmw'],poll_count=len(samples),publisher_absent_polls=sum(not r['publishers'] for r in samples),first_appearance_delay_s=min((t-start for t,_ in endpoints),default=None),publisher_gids=list(gids),publisher_names=sorted({e['namespace']+'/'+e['node'] for _,e in endpoints}),cli_observation=read(out/'cli_topic_info.json'),daemon_restart=False,multicast_interface_flag='SUPPORTED_FLAG_NOT_PACKET_VERIFIED')
    write('dds_comparison.json',discovery)
    data={}
    for name in ('best_effort','reliable'):
        with (out/('jointstate_'+name+'.csv')).open() as f:data[name]=list(csv.DictReader(f))
    bag=[]
    for db in (out/'jointstate_rosbag').glob('*.db3'):
        c=sqlite3.connect('file:'+str(db)+'?mode=ro',uri=True)
        for stamp,b in c.execute("SELECT timestamp,data FROM messages WHERE topic_id IN (SELECT id FROM topics WHERE name='/dsr01/joint_states') ORDER BY timestamp"):
            m=deserialize_message(b,JointState);bag.append(dict(source=m.header.stamp.sec+m.header.stamp.nanosec/1e9,receive=stamp/1e9))
        c.close()
    events={}
    for name,rows in list(data.items())+[('bag',bag)]:
        events[name]=[dict(before=float(a['source']),after=float(b['source']),source_gap=float(b['source'])-float(a['source'])) for a,b in zip(rows,rows[1:]) if float(b['source'])-float(a['source'])>=.1]
    shared=[]
    for e in events['best_effort']:
        match={name:any(abs(v['before']-e['before'])<1e-6 and abs(v['after']-e['after'])<1e-6 for v in events[name]) for name in ('reliable','bag')}
        shared.append(dict(e,**match))
    write('jointstate_gap_comparison.json',dict(events=events,shared=shared,bag_count=len(bag),bag_source_coverage_s=bag[-1]['source']-bag[0]['source'] if bag else None,bag_receive_clock='ROS_SYSTEM_NOT_MONOTONIC',interpretation='Matching source gaps across same-host observers establishes received stream omissions, not whether controller/driver generation or DDS common delivery caused them.'))
    with (out/'host_stats.csv').open() as f:hosts=list(csv.DictReader(f))
    def drops(row):
        result={}
        for line in row['network'].splitlines():
            if ':' not in line:continue
            name,values=line.split(':',1);v=values.split()
            result[name.strip()]=dict(rx_bytes=int(v[0]),rx_errors=int(v[2]),rx_drops=int(v[3]),tx_bytes=int(v[8]),tx_errors=int(v[10]),tx_drops=int(v[11]))
        return result
    net={}
    if hosts:
        firstnet,lastnet=drops(hosts[0]),drops(hosts[-1])
        net={n:{k:lastnet[n][k]-v for k,v in values.items()} for n,values in firstnet.items() if n in lastnet}
    import ast
    write('host_network_summary.json',dict(samples=len(hosts),network_deltas=net,load_1min_max=max((ast.literal_eval(row['load'])[0] for row in hosts),default=None),driver_cpu_baseline='~203% from captured ps; cumulative CPU alone not proof of stall',causality='UNRESOLVED_NO_CONTROLLED_HOST_INTERVENTION',raw_gap_alignment='source ROS timestamps align only with receive_wall; monotonic timestamps remain separate'))
    # Copy CURRENT follow-up source evidence, preserving both original trials.
    write('tcp_source_followup.json',dict(source_trial=str(passive),active=read(passive/'active_tcp_tool.json'),offset=read(passive/'tcp_offset_search.json'),state=read(passive/'robot_state_sources.json'),joint=read(passive/'jointstate_status.json')))
    roots=['/home/ubuntu/robot_ws/src/doosan-robot2/dsr_controller2/config','/home/ubuntu/robot_ws/src/openvla_doosan_runtime/config']
    p=subprocess.run(['rg','-n','Tool_v1|tcp_offset|tool_offset|active_tcp|ConfigCreateTcp']+roots,capture_output=True,text=True)
    write('tcp_config_binding.json',dict(current_active_name='UNKNOWN',binding='NOT_VERIFIED',source_search=p.stdout,no_match_returncode=p.returncode,historical_config_not_reused=True))
    h=read(out/'model_health.json');camera=read(out/'camera_validation.json');shadow=read(out/'openvla_shadow_summary.json');gate=read(out/'jointstate_clean_gate.json')
    predictions=[json.loads(s) for s in (out/'openvla_shadow.jsonl').read_text().splitlines() if 'canonical_action' in json.loads(s)]
    blockers=collections.Counter(b for r in predictions for b in r['safety']['reason'])
    write('prediction_analysis.json',dict(blockers=dict(blockers),latency_max_s=max((r['inference_latency_s'] for r in predictions),default=None),nan_inf=0 if all(all(__import__('math').isfinite(x) for x in r['raw_model_output']['action']) for r in predictions) else 'INVALID',command_records_all_disabled=all(not r['command_issued'] and r['executed_action'] is None and r['robot_delivered_command'] is None for r in predictions)))
    text=f'''# Integrated real readiness — latest verdict: BLOCKED

## ROS / DDS

RMW: {timeline['rmw']}; ROS_DOMAIN_ID unset/default 0; ROS_LOCALHOST_ONLY=0. Driver/daemon environment reads showed no explicit FastDDS/Cyclone config or domain mismatch. NIC enp3s0=192.168.0.100/24, Wi-Fi=192.168.1.98/24; both UP/MULTICAST. This flag is not a multicast traffic test. Driver network 192.168.0.0/24 has a connected route. No daemon or driver restart was performed.

Direct polling: {len(samples)} samples at target200ms; missing publisher polls {discovery['publisher_absent_polls']}. Node metadata may be UNKNOWN even with an endpoint. CLI currently identifies joint_state_broadcaster,/dsr01 and agrees publisher count1; earlier historical disagreement remains preserved. No current CLI_DAEMON_DISCOVERY_INCONSISTENCY proved in this snapshot. Service endpoints list_controllers/list_hardware_interfaces appeared in direct graph, types are recorded in timeline; GID unavailable in installed rclpy graph API. Endpoint presence is not a successful service reply or controller connection confirmation. No new service invocation needed/attempted.

## JointState

Clean gate: {gate['status']}. Required first-normal-sample +30s / max discovery wait60s; the preserved records include initial/cache samples rather than silently deleting them. A cached first sample followed by source/receive gap fails this gate. Invalid finite/name counts are 0. Both QoS compatible with offered RELIABLE/TRANSIENT_LOCAL.

```json
{json.dumps(gate['subscribers'],indent=2)}
```

Simultaneous rosbag samples {len(bag)}. Exact shared >=100ms source-gap pairs across BE/reliable/bag: {sum(e['reliable'] and e['bag'] for e in shared)}. Details in jointstate_gap_comparison.json. This is not proof of controller generation failure, because all recipients share the same host/RMW. No new driver log bytes during integrated window: no new Skip-dt can be causally correlated. Host load/memory/cpu/network/driver scheduling raw data stored at target200ms in host_stats.csv. No isolated controlled host-load intervention was performed; correlation is not causality.

The subsequent passive follow-up received {read(passive/'jointstate_status.json')['count']} samples, but coverage/first discovery did not establish a complete20s gate. This newer observation does not replace the failed clean trial.

## Camera / OpenVLA

Camera30s: {camera['status']}, {camera['count']} predictions / {camera['duration_s']:.3f}s, selected-frame max age {camera['max_selected_frame_age_s']:.6f}s; camera_failure {camera['camera_failure']}. Threshold0.5s unchanged. BGRA1280x720→RGB fast decoder→JPEG quality95→verified processor; source/RGB/JPEG hashes stored.

GPU/model health: {h['status']}; checkpoint8130, K1, dim7, no proprio; strict processor metadata checked before fresh frame, lightweight unchanged-health check before subsequent snapshots. Sample real predictions66, fixture forbidden. Translation max {shadow['translation_max_m']*1000:.3f}mm, rotation max {shadow['rotation_max_deg']:.3f}deg, close candidates {shadow['close_candidates']}, critical magnitude anomalies {shadow['anomalies']}. Compared with earlier~2.5mm/~0.6deg, rotation max rose but stays below4deg limit; this is not task-success evidence.

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
'''
    (out/'INTEGRATED_REAL_READINESS_REPORT.md').write_text(text)
    print(text)
if __name__=='__main__':main()
