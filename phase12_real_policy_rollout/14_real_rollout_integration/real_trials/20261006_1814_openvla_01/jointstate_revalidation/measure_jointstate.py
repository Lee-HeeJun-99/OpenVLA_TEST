"""Read-only subscriber measurement; no robot clients or publishers."""
import csv, json, math, os, statistics, time
from pathlib import Path
import argparse
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState

def distribution(gaps):
    ordered = sorted(gaps)
    def percentile(p):
        return ordered[min(len(ordered)-1, math.ceil(p*len(ordered))-1)] if ordered else None
    return dict(mean_gap_s=statistics.mean(gaps) if gaps else None,
                p95_gap_s=percentile(.95), p99_gap_s=percentile(.99),
                max_gap_s=max(gaps) if gaps else None,
                gaps_ge_100ms=sum(x >= .1 for x in gaps),
                gaps_ge_500ms=sum(x >= .5 for x in gaps),
                gaps_ge_1s=sum(x >= 1 for x in gaps),
                duplicates=sum(x == 0 for x in gaps), regressions=sum(x < 0 for x in gaps))

def main():
    p=argparse.ArgumentParser(); p.add_argument('--run', required=True); a=p.parse_args()
    root=Path(__file__).resolve().parent
    samples=[]; hosts=[]; rclpy.init(); node=Node('phase12_jointstate_only_'+a.run)
    start=time.monotonic(); wall=time.time(); last_host=0
    canonical=['joint_'+str(i) for i in range(1,7)]
    def callback(msg):
        now=time.monotonic(); source=msg.header.stamp.sec+msg.header.stamp.nanosec/1e9
        missing=any(name not in msg.name for name in canonical)
        badpos=len(msg.position)!=len(msg.name) or not all(math.isfinite(x) for x in msg.position)
        badvel=len(msg.velocity)!=len(msg.name) or not all(math.isfinite(x) for x in msg.velocity)
        samples.append(dict(receive_monotonic=now,receive_wall=time.time(),source_ros=source,
            joint_names=json.dumps(list(msg.name)), position=json.dumps(list(msg.position)),
            velocity=json.dumps(list(msg.velocity)),missing_joint_name=missing,
            invalid_position=badpos,invalid_velocity=badvel,effort_status='unsupported_not_used'))
    node.create_subscription(JointState,'/dsr01/joint_states',callback,qos_profile_sensor_data)
    while time.monotonic()-start<60:
        rclpy.spin_once(node,timeout_sec=.02)
        now=time.monotonic()
        if now-last_host>=1:
            last_host=now
            hosts.append(dict(run=a.run,monotonic=now,wall=time.time(),load=json.dumps(os.getloadavg()),
                cpu_stat=Path('/proc/stat').read_text().splitlines()[0],
                memory=Path('/proc/meminfo').read_text().replace('\n',';'),
                network=Path('/proc/net/dev').read_text().replace('\n',';'),
                driver_proc_stat=Path('/proc/3017586/stat').read_text() if Path('/proc/3017586/stat').exists() else 'PROCESS_NOT_FOUND',
                driver_proc_status=Path('/proc/3017586/status').read_text().replace('\n',';') if Path('/proc/3017586/status').exists() else 'PROCESS_NOT_FOUND'))
    duration=time.monotonic()-start
    for filename, rows in [(a.run+'_jointstate.csv',samples), (a.run+'_host_stats.csv',hosts)]:
        with (root/filename).open('x',newline='') as f:
            keys=list(rows[0]) if rows else ['receive_monotonic','source_ros']
            w=csv.DictWriter(f,fieldnames=keys);w.writeheader();w.writerows(rows);f.flush();os.fsync(f.fileno())
    receive=[s['receive_monotonic'] for s in samples]; source=[s['source_ros'] for s in samples]
    def stats(ts):
        gaps=[b-a for a,b in zip(ts,ts[1:])]
        return dict(distribution(gaps),rate_hz=(len(ts)-1)/(ts[-1]-ts[0]) if len(ts)>1 and ts[-1]>ts[0] else None)
    result=dict(run=a.run,duration_s=duration,start_wall=wall,message_count=len(samples),
        full_window_rate_hz=len(samples)/duration,first_receive_delay_s=receive[0]-start if receive else None,
        steady_receive=stats(receive),steady_source=stats(source),
        invalid_position=sum(s['invalid_position'] for s in samples),
        invalid_velocity=sum(s['invalid_velocity'] for s in samples),
        missing_joint_name=sum(s['missing_joint_name'] for s in samples),
        observed_orders=list(set(s['joint_names'] for s in samples)),
        subscriber_qos=dict(reliability='BEST_EFFORT',durability='VOLATILE',history='KEEP_LAST',depth=5),
        robot_command_count=0)
    with (root/(a.run+'_summary.json')).open('x') as f:json.dump(result,f,indent=2);f.flush();os.fsync(f.fileno())
    print(json.dumps(result,indent=2),flush=True)
    node.destroy_node();rclpy.shutdown()
if __name__=='__main__':main()
