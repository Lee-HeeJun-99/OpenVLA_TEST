import csv,json,math,re,sqlite3,statistics
from pathlib import Path
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import JointState
ROOT=Path(__file__).resolve().parent
def save(name,rows,keys):
    with (ROOT/name).open('x',newline='') as f:
        w=csv.DictWriter(f,fieldnames=list(rows[0]) if rows else keys);w.writeheader();w.writerows(rows)
def stats(rows):
    out={'count':len(rows)}
    for column in ('source_ros','receive_wall'):
        ts=[float(r[column]) for r in rows];gs=[b-a for a,b in zip(ts,ts[1:])];ss=sorted(gs)
        out[column]={'rate_hz':(len(ts)-1)/(ts[-1]-ts[0]) if len(ts)>1 and ts[-1]>ts[0] else None,
            'mean_gap_s':statistics.mean(gs) if gs else None,'max_gap_s':max(gs) if gs else None,
            'p95_gap_s':ss[min(len(ss)-1,math.ceil(len(ss)*.95)-1)] if ss else None,
            'p99_gap_s':ss[min(len(ss)-1,math.ceil(len(ss)*.99)-1)] if ss else None,
            'duplicates':sum(x==0 for x in gs),'regressions':sum(x<0 for x in gs),
            **{'gaps_ge_'+str(t)+'s':sum(x>=t for x in gs) for t in (.1,.5,1,2)}}
    return out
results=[];events=[];driver_events=[];hosts=[]
for run in ('run1','run2'):
    meta=json.loads((ROOT/(run+'_metadata.json')).read_text());streams={}
    for kind in ('best_effort','reliable'):
        with (ROOT/(run+'_'+kind+'.csv')).open() as f:streams[kind]=list(csv.DictReader(f))
    bag=[]
    for db in sorted((ROOT/(run+'_rosbag')).glob('*.db3')):
        con=sqlite3.connect(str(db))
        for seq,(timestamp,data) in enumerate(con.execute('SELECT timestamp,data FROM messages ORDER BY timestamp,id')):
            m=deserialize_message(data,JointState)
            bag.append(dict(sequence=seq,receive_wall=timestamp/1e9,source_ros=m.header.stamp.sec+m.header.stamp.nanosec/1e9,
                joint_names=json.dumps(list(m.name)),position=json.dumps(list(m.position)),velocity=json.dumps(list(m.velocity))))
        con.close()
    streams['bag']=bag;save(run+'_bag_messages.csv',bag,['sequence','receive_wall','source_ros'])
    gaps={k:[(a,b) for a,b in zip(rows,rows[1:]) if float(b['source_ros'])-float(a['source_ros'])>=.1] for k,rows in streams.items()}
    unique={ (round(float(a['source_ros']),6),round(float(b['source_ros']),6)) for gs in gaps.values() for a,b in gs }
    log=(ROOT/(run+'_driver_new.log')).read_text().splitlines()
    with (ROOT/(run+'_host.csv')).open() as f:hr=list(csv.DictReader(f));hosts.extend(hr)
    for before,after in sorted(unique):
        event=dict(event_id=run+'_'+str(len(events)),run=run,source_before=before,source_after=after,source_gap_s=after-before)
        wall=None
        for k,gs in gaps.items():
            match=next(((a,b) for a,b in gs if abs(float(a['source_ros'])-before)<.00001 and abs(float(b['source_ros'])-after)<.00001),None)
            event[k+'_matched']=match is not None
            event[k+'_receive_gap_s']=float(match[1]['receive_wall'])-float(match[0]['receive_wall']) if match else None
            if match:wall=float(match[1]['receive_wall'])
        event['wall_after']=wall;events.append(event)
        matched=0
        for line in log:
            stamp=re.search(r'\[(\d{10}\.\d+)\]',line)
            if stamp and wall and wall-(after-before)-1<=float(stamp.group(1))<=wall+1:
                driver_events.append(dict(event_id=event['event_id'],driver_log=line));matched+=1
        if not matched:driver_events.append(dict(event_id=event['event_id'],driver_log='NO_NEW_DRIVER_LOG_IN_EVENT_INTERVAL'))
    def network(row):
        out={}
        for line in row['network'].split(';'):
            if ':' not in line:continue
            name,values=line.split(':',1);tokens=values.split()
            if len(tokens)==16:out[name.strip()]=list(map(int,tokens))
        return out
    firstnet=network(hr[0]);lastnet=network(hr[-1]);network_delta={}
    for name,v in lastnet.items():
        if name in firstnet:
            d=[b-a for a,b in zip(firstnet[name],v)]
            network_delta[name]=dict(rx_bytes=d[0],rx_errors=d[2],rx_drops=d[3],tx_bytes=d[8],tx_errors=d[10],tx_drops=d[11])
    drivercpu=[];sched_wait=[];cpubusy=[]
    for a,b in zip(hr,hr[1:]):
        dt=float(b['receive_monotonic'])-float(a['receive_monotonic'])
        if a['driver_stat']!='MISSING' and b['driver_stat']!='MISSING':
            aa=a['driver_stat'].split(') ',1)[1].split();bb=b['driver_stat'].split(') ',1)[1].split()
            import os
            drivercpu.append(100*(int(bb[11])+int(bb[12])-int(aa[11])-int(aa[12]))/os.sysconf('SC_CLK_TCK')/dt)
            sched_wait.append((int(b['driver_schedstat'].split()[1])-int(a['driver_schedstat'].split()[1]))/1e9)
        aa=list(map(int,a['cpu_stat'].split()[1:]));bb=list(map(int,b['cpu_stat'].split()[1:]));d=[y-x for x,y in zip(aa,bb)];total=sum(d)
        if total:cpubusy.append(100*(total-d[3]-(d[4] if len(d)>4 else 0))/total)
    results.append(dict(metadata=meta,streams={k:stats(v) for k,v in streams.items()},
        host=dict(network_counter_delta=network_delta,driver_cpu_percent_mean=statistics.mean(drivercpu) if drivercpu else None,
            driver_cpu_percent_max=max(drivercpu) if drivercpu else None,host_cpu_busy_percent_max=max(cpubusy) if cpubusy else None,
            driver_main_thread_sched_wait_delta_max_s=max(sched_wait) if sched_wait else None,
            host_sample_max_gap_s=max(float(b['receive_monotonic'])-float(a['receive_monotonic']) for a,b in zip(hr,hr[1:]))),
        first_receive_delay={k:float(v[0]['receive_wall'])-meta['start_wall'] if v else None for k,v in streams.items()}))
save('gap_correlation.csv',events,['event_id']);save('driver_event_correlation.csv',driver_events,['event_id','driver_log']);save('host_network_stats.csv',hosts,['run','wall'])
allmatched=[e for e in events if all(e[k+'_matched'] for k in ('best_effort','reliable','bag'))]
invalid=any(r['metadata']['external_command_detected'] for r in results)
verdict='STILL_INCONCLUSIVE'
summary=dict(classification=verdict,runs=results,gap_count=len(events),three_stream_matching_gaps=len(allmatched),
    invalid_external_command=invalid,robot_commands_by_audit=0,motion_authorized=False,
    caveat='All receivers share host/RMW; matching gaps localize upstream of observer, not uniquely controller hardware. Header is ROS update clock, not controller sampling clock.',
    evidence_interpretation='Shared stream discontinuity if three-stream matches exist; direct publisher/update/RT trace not captured, therefore no upstream hardware CONFIRMED verdict.',
    observer_independence='BEST_EFFORT and RELIABLE share one Python executor; bag uses separate process. Bag gaps outside its actual receive interval cannot be compared.')
(ROOT/'summary.json').write_text(json.dumps(summary,indent=2));print(json.dumps(summary,indent=2))
table=[]
for r in results:
    for kind,s in r['streams'].items():
        t=s['source_ros'];table.append(f"| {r['metadata']['run']} | {kind} | {s['count']} | {r['first_receive_delay'][kind]} | {t['rate_hz']} | {t['max_gap_s']} | {t['gaps_ge_1s']} |")
hosttext='\n'.join(r['metadata']['run']+': '+json.dumps(r['host']) for r in results)
(ROOT/'jointstate_gap_rootcause_report.md').write_text('''# JointState gap root-cause comparison

Verdict: STILL_INCONCLUSIVE. Feedback continuity FAIL; no motion authorized.

Two 120-second acquisitions were actually performed. Driver PID3017586 was not restarted. Camera/model/rollout processes were not running. Command-capable candidates were inspected; no ActionAdapter/DoosanBridge/teleop/model runner process was found. /so101_usd_dashboard existed in discovery without an identifiable matching host process; remote client absence cannot be proved. Operator no-command confirmation was requested but not automatically asserted. Runs with new movej/movel/mode/tool-output callback logs are invalid; see run metadata. No command-capable process was automatically killed.

| Run | Stream | Messages | First receive delay s | Source Hz after first sample | Max source gap s | Source gaps >=1s |
|---|---|---:|---:|---:|---:|---:|
'''+ '\n'.join(table)+f'''

Matching source-before/source-after intervals across all three streams: {len(allmatched)}. All gap intervals (>=100ms) are in gap_correlation.csv; bag entries outside actual bag receive coverage are not evidence of bag-normal delivery. Source and receive distributions, >=100/500ms/1/2s counts and duplicate/regression counts are in summary.json. Subscriber sequence is local receive order, not publisher sequence.

BEST_EFFORT and RELIABLE share a lightweight Python executor; rosbag is a separate process. Common gaps across both reliable receivers and BEST_EFFORT support a common upstream or shared middleware path, not exclusively BEST_EFFORT loss or a single Python observer. All still share host/RMW. Without publisher/update/RT instrumentation, direct driver/controller stall versus shared DDS loss is not proven.

Header timestamps come from broadcaster update time (see publisher_source_audit.md), not controller sampling time. The distinction is essential: source jumps represent missing/delayed generated update stamps in the delivered stream, not proof of controller hardware packet interruption.

## Host/network

200ms snapshots contain CPU, memory/load, network RX/TX/error/drop counters, driver CPU ticks and main-thread scheduling counters. Per-run counter changes/maxima:

{hosttext}

Counters are host-interface counters, not controller-link packet capture or switch/controller counters. Main-thread schedstat does not measure every driver worker thread. No causal network/CPU claim follows solely from a matching peak. Gap events and raw host timestamps allow further comparison. Driver-event correlation includes the gap duration plus +/-1s using receive-wall times; ROS log clock equality was not independently validated. NO_NEW_DRIVER_LOG does not mean no internal stall, especially when tracing is disabled. Historical startup Skip-dt logs were excluded using file offsets.

## Answers and next gate

1. Source-header discontinuities and reliable/bag correspondence: numerical results above; ordinary 10ms samples coexist with multi-second missing intervals.
2. Driver-log synchronization: inspect driver_event_correlation.csv; absence of event logs cannot localize vendor internals.
3. Host/network relation: recorded but no proven causal association. QoS publisher configuration was not changed.
4. JointState is not currently accepted as a rollout feedback source: repeated >=1s gaps fail continuity.
5. Next read-only experiment should observe publisher pre-publish/update and hardware read/RT packet counters on the same monotonic timeline alongside reliable bag, under operator-confirmed no external callers. This distinguishes publish omission/trylock/update stall from downstream middleware starvation. No getter or driver recovery is authorized by this report.

Actual robot/motion/gripper/Home/trajectory/Hold/E-stop/real rollout calls by this audit: 0. External activity is reported separately and cannot be certified globally. Production thresholds and driver sources unchanged.
''')
