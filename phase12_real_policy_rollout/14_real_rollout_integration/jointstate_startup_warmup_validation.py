"""Passive fixed-phase JointState validation. No command/service capability."""
import csv,datetime,json,math,os,time
from pathlib import Path
import rclpy
from rclpy.qos import QoSProfile,ReliabilityPolicy,DurabilityPolicy
from sensor_msgs.msg import JointState
ROOT=Path(__file__).resolve().parent

def phase_at(t,first,t0):
    if first is None or t<first:return 'STARTUP_DISCOVERY_PHASE'
    if t0 is None or t<t0:return 'POST_DISCOVERY_WARMUP'
    return 'RUNTIME_CLEAN_WINDOW'

def main():
    out=ROOT/'real_trials'/datetime.datetime.now().strftime('%Y%m%d_%H%M%S_jointstate_startup_warmup_validation');out.mkdir(exist_ok=False,parents=True)
    rclpy.init();node=rclpy.create_node('phase12_jointstate_fixed_phase_validation')
    rows={'best_effort':[],'reliable':[]};firsts={};discovery=[];allrows=[]
    started=time.monotonic();deadline=started+60;first=None;t0=None;end=None
    def receive(kind):
        def cb(msg):
            mono=time.monotonic();ros=node.get_clock().now().nanoseconds;stamp=msg.header.stamp.sec*10**9+msg.header.stamp.nanosec
            names=list(msg.name);index={n:i for i,n in enumerate(names)};canonical=[f'joint_{i}' for i in range(1,7)]
            missing=len(names)!=6 or len(index)!=6 or any(n not in index for n in canonical)
            invalid_position=missing or len(msg.position)!=len(names) or not all(math.isfinite(msg.position[index[n]]) for n in canonical)
            invalid_velocity=missing or len(msg.velocity)!=len(names) or not all(math.isfinite(msg.velocity[index[n]]) for n in canonical)
            prior=rows[kind][-1] if rows[kind] else None
            regression=prior is not None and stamp<prior['source_ns']
            age=(ros-stamp)/1e9
            valid=not(missing or invalid_position or invalid_velocity or regression)
            fresh=valid and 0<=age<.1
            row=dict(subscriber=kind,index=len(rows[kind]),receive_monotonic=mono,receive_wall=time.time(),receive_ros_ns=ros,source_ns=stamp,header_age_s=age,joint_names=json.dumps(names),position=json.dumps(list(msg.position)),velocity=json.dumps(list(msg.velocity)),canonical_order=json.dumps(canonical),missing_joints=missing,invalid_position=invalid_position,invalid_velocity=invalid_velocity,regression=regression,fresh=fresh,effort='UNSUPPORTED')
            rows[kind].append(row);allrows.append(row)
            if fresh and kind not in firsts:firsts[kind]=mono
        return cb
    for kind,rel,dur in [('best_effort',ReliabilityPolicy.BEST_EFFORT,DurabilityPolicy.VOLATILE),('reliable',ReliabilityPolicy.RELIABLE,DurabilityPolicy.TRANSIENT_LOCAL)]:
        node.create_subscription(JointState,'/dsr01/joint_states',receive(kind),QoSProfile(depth=1000,reliability=rel,durability=dur))
    nextpoll=started
    while True:
        now=time.monotonic()
        if first is None and len(firsts)==2:
            # Shared window waits for both FIRST_FRESH_SAMPLE events; individual
            # first-fresh timestamps retained. Boundary never resets on failure.
            first=max(firsts.values());t0=first+10;end=t0+30
        if first is None and now>=deadline:break
        if end is not None and now>=end:break
        if now>=nextpoll:
            discovery.append(dict(receive_monotonic=now,publishers=[dict(name=e.node_name,namespace=e.node_namespace,gid=list(e.endpoint_gid),qos=str(e.qos_profile)) for e in node.get_publishers_info_by_topic('/dsr01/joint_states')]))
            nextpoll=now+1 # Minimal graph polling, no external CLI processes.
        rclpy.spin_once(node,timeout_sec=.01)
    finished=time.monotonic()
    for row in allrows:row['phase']=phase_at(row['receive_monotonic'],first,t0)
    events=[]
    for kind,data in rows.items():
        for a,b in zip(data,data[1:]):
            sg=(b['source_ns']-a['source_ns'])/1e9;rg=b['receive_monotonic']-a['receive_monotonic']
            if sg>=.1 or rg>=.1:
                events.append(dict(subscriber=kind,event_index=len(events),index_before=a['index'],index_after=b['index'],source_before_ns=a['source_ns'],source_after_ns=b['source_ns'],receive_before=a['receive_monotonic'],receive_after=b['receive_monotonic'],source_gap_s=sg,receive_gap_s=rg,header_age_before_s=a['header_age_s'],header_age_after_s=b['header_age_s'],phase=b['phase'],phase_before=a['phase'],cross_boundary=a['phase']!=b['phase']))
    def stats(data,phase):
        selected=[r for r in data if r['phase']==phase]
        sg=[(b['source_ns']-a['source_ns'])/1e9 for a,b in zip(selected,selected[1:])];rg=[b['receive_monotonic']-a['receive_monotonic'] for a,b in zip(selected,selected[1:])]
        # Conservatively include intervals crossing into runtime from warmup.
        runtime_events=[e for e in events if e['subscriber']==(data[0]['subscriber'] if data else None) and e['phase']==phase]
        coverage=selected[-1]['receive_monotonic']-selected[0]['receive_monotonic'] if len(selected)>1 else 0
        lastage=finished-selected[-1]['receive_monotonic'] if selected else None
        valid=bool(selected) and not any(r['missing_joints'] or r['invalid_position'] or r['invalid_velocity'] or r['regression'] for r in selected)
        return dict(count=len(selected),coverage_s=coverage,rate_hz=(len(selected)-1)/coverage if coverage else 0,latest_receive_age_s=lastage,max_source_gap_s=max(sg+[e['source_gap_s'] for e in runtime_events],default=None),max_receive_gap_s=max(rg+[e['receive_gap_s'] for e in runtime_events],default=None),duplicate=sum(g==0 for g in sg),regression=sum(g<0 for g in sg),invalid_position=sum(r['invalid_position'] for r in selected),invalid_velocity=sum(r['invalid_velocity'] for r in selected),missing_joints=sum(r['missing_joints'] for r in selected),event_ge_100ms=len(runtime_events),pass_runtime=coverage>=29.9 and lastage is not None and lastage<.5 and valid and bool(sg) and all(0<g<.1 for g in sg) and all(0<g<.1 for g in rg) and not runtime_events)
    phases={phase:{kind:stats(data,phase) for kind,data in rows.items()} for phase in ['STARTUP_DISCOVERY_PHASE','POST_DISCOVERY_WARMUP','RUNTIME_CLEAN_WINDOW']}
    runtime=phases['RUNTIME_CLEAN_WINDOW'];passed=first is not None and all(r['pass_runtime'] for r in runtime.values())
    verdict='DISCOVERY_NOT_STABILIZED' if first is None else 'STARTUP_ARTIFACT_ONLY_RUNTIME_PASS' if passed else 'RUNTIME_GAP_CONFIRMED' if any(e['phase']=='RUNTIME_CLEAN_WINDOW' for e in events) else 'INCONCLUSIVE'
    def save(name,value):
        with (out/name).open('x') as f:json.dump(value,f,indent=2)
    for name,phase in [('startup_phase.json','STARTUP_DISCOVERY_PHASE'),('warmup_phase.json','POST_DISCOVERY_WARMUP'),('runtime_clean_window.json','RUNTIME_CLEAN_WINDOW')]:save(name,dict(phase=phase,subscribers=phases[phase]))
    save('gap_phase_classification.json',events)
    for name,data in [('jointstate_all_samples.csv',allrows)]+[('jointstate_'+k+'.csv',v) for k,v in rows.items()]:
        with (out/name).open('x',newline='') as f:
            w=csv.DictWriter(f,fieldnames=list(allrows[0]) if allrows else ['subscriber','index','phase']);w.writeheader();w.writerows(data);f.flush();os.fsync(f.fileno())
    result=dict(verdict=verdict,started_monotonic=started,first_fresh_sample_by_subscriber=firsts,shared_first_fresh_sample=first,startup_duration_s=first-started if first else None,warmup_duration_s=10,runtime_t0=t0,runtime_t1=end,runtime_duration_s=30,runtime_gate='JOINTSTATE_RUNTIME_CLEAN_PASS' if passed else 'JOINTSTATE_RUNTIME_CLEAN_FAIL',phases=phases,publisher_discovery=discovery,physical_commands=0,next_allowed_stage='TCP_PRECHECK' if passed else 'BLOCKED',clock_domains=dict(source='ROS_HEADER',receive_ros='LOCAL_NODE_ROS_CLOCK',receive_monotonic='HOST_MONOTONIC',receive_wall='SYSTEM_WALL'),clock_caveat='Header-age compared only in ROS clock domain; no source-minus-monotonic arithmetic. Negative age samples never FIRST_FRESH_SAMPLE.')
    save('summary.json',result)
    (out/'JOINTSTATE_STARTUP_WARMUP_REPORT.md').write_text('# JointState startup/warmup validation\n\n```json\n'+json.dumps(result,indent=2)+'\n```\n\nStartup/warmup raw samples and all gap events preserved. Fixed runtime T0+30s never restarts after gaps. Boundary-straddling >=100ms intervals arriving in runtime are conservatively counted as runtime failures. Runtime thresholds unchanged (100ms gap/500ms age). Both subscriber first-fresh clocks preserved; shared gate starts after the later first-fresh +10s. No getters/services/publishers/actions, driver restart, robot command or mode/tool/servo change. Rosbag omitted to minimize additional host load.\n')
    node.destroy_node();rclpy.shutdown();print(str(out),flush=True);print(json.dumps(dict(verdict=verdict,startup_duration=result['startup_duration_s'],runtime=runtime,physical_commands=0),indent=2),flush=True)
if __name__=='__main__':main()
