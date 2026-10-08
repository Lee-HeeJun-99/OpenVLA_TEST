"""Independent passive gap collector; no runtime changes or robot capability."""
import argparse
import csv
import datetime
import json
import math
import os
import time
from pathlib import Path

TOPIC='/dsr01/joint_states'


def summarize(rows):
    def gaps(kind):return [row[kind] for row in rows if row.get(kind) is not None]
    source=gaps('source_gap_s');receive=gaps('receive_gap_s')
    duration=rows[-1]['receive_monotonic']-rows[0]['receive_monotonic'] if len(rows)>1 else 0
    return dict(sample_count=len(rows),receive_coverage_s=duration,receive_rate_hz=(len(rows)-1)/duration if duration else None,
                max_source_gap_s=max(source,default=None),max_receive_gap_s=max(receive,default=None),
                source_gap_ge_100ms=sum(value>=.1 for value in source),receive_gap_ge_100ms=sum(value>=.1 for value in receive),
                duplicate_source_timestamps=sum(value==0 for value in source),regressions=sum(value<0 for value in source),
                invalid_samples=sum(not row['valid'] for row in rows),
                interpretation='NO_SAMPLES_NOT_STABILITY_PASS' if not rows else 'DIAGNOSTIC_ONLY_NOT_MOTION_READINESS')


def run(args):
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile,ReliabilityPolicy,DurabilityPolicy,HistoryPolicy
    from sensor_msgs.msg import JointState
    from rclpy.utilities import get_rmw_implementation_identifier
    args.output.mkdir(parents=True,exist_ok=False)
    __import__("os").environ["ROS_LOCALHOST_ONLY"] = "1"; rclpy.init();node=Node('gap_monitor');rows=[];events=[];endpoints=[]
    started=time.monotonic();first=None;previous=None
    def callback(message):
        nonlocal first,previous
        tick=time.monotonic();wall=time.time();source=message.header.stamp.sec*10**9+message.header.stamp.nanosec
        if first is None:first=tick
        valid=(len(message.name)==6 and set(message.name)=={f'joint_{i}' for i in range(1,7)} and
               len(message.position)==len(message.velocity)==6 and all(math.isfinite(v) for v in list(message.position)+list(message.velocity)))
        row=dict(index=len(rows),receive_monotonic=tick,receive_wall=wall,source_ns=source,
                 header_age_s=(node.get_clock().now().nanoseconds-source)/1e9,
                 source_gap_s=(source-previous['source_ns'])/1e9 if previous else None,
                 receive_gap_s=tick-previous['receive_monotonic'] if previous else None,
                 names=list(message.name),position=list(message.position),velocity=list(message.velocity),valid=valid)
        if previous and (row['receive_gap_s']>=.1 or row['source_gap_s']>=.1):
            events.append(dict(event_id=len(events),**row,source_before_ns=previous['source_ns'],
                               receive_before=previous['receive_monotonic'],wall_before=previous['receive_wall']))
            print(f"receive gap {row['receive_gap_s']:.6f}s | source gap {row['source_gap_s']:.6f}s | wall {wall:.6f}",flush=True)
        row['callback_work_s']=time.monotonic()-tick
        rows.append(row);previous=row
    qos=QoSProfile(depth=1,reliability=ReliabilityPolicy.BEST_EFFORT,
                   durability=DurabilityPolicy.VOLATILE,history=HistoryPolicy.KEEP_LAST)
    node.create_subscription(JointState,TOPIC,callback,qos)
    try:
        # Discovery only until first sample; no graph/CLI polling during measurement.
        next_discovery=started
        while first is None and time.monotonic()-started<args.discovery_timeout:
            rclpy.spin_once(node,timeout_sec=.01)
            if time.monotonic()>=next_discovery:
                endpoints=[dict(node_name=e.node_name,node_namespace=e.node_namespace,topic_type=e.topic_type,
                    endpoint_gid=list(e.endpoint_gid),reliability=str(e.qos_profile.reliability),
                    durability=str(e.qos_profile.durability),history=str(e.qos_profile.history),depth=e.qos_profile.depth)
                    for e in node.get_publishers_info_by_topic(TOPIC)]
                next_discovery=time.monotonic()+1
        while first is not None and time.monotonic()-first<args.duration:
            rclpy.spin_once(node,timeout_sec=.01)
    except KeyboardInterrupt:pass
    finally:
        result=dict(**summarize(rows),case=args.case,topic=TOPIC,duration_requested_s=args.duration,
            first_receive_delay_s=first-started if first is not None else None,
            latest_receive_age_s=time.monotonic()-rows[-1]['receive_monotonic'] if rows else None,
            rmw=get_rmw_implementation_identifier(),environment={key:os.environ.get(key) for key in ('ROS_DOMAIN_ID','ROS_LOCALHOST_ONLY','RMW_IMPLEMENTATION')},
            qos=dict(reliability='BEST_EFFORT',durability='VOLATILE',history='KEEP_LAST',depth=1),
            publisher_endpoints=endpoints,physical_commands=0,settings_changed=False)
        with (args.output/'samples.csv').open('x',newline='') as file:
            names=list(rows[0]) if rows else ['index','source_ns','receive_monotonic']
            writer=csv.DictWriter(file,fieldnames=names);writer.writeheader()
            for row in rows:
                writer.writerow({key:json.dumps(value) if isinstance(value,list) else value for key,value in row.items()})
        (args.output/'gap_events.json').write_text(json.dumps(events,indent=2))
        (args.output/'summary.json').write_text(json.dumps(result,indent=2))
        node.destroy_node();rclpy.shutdown();print(json.dumps(result,indent=2),flush=True)


if __name__=='__main__':
    parser=argparse.ArgumentParser()
    parser.add_argument('--case',required=True,help='User-selected case label; does not change settings')
    parser.add_argument('--duration',type=float,default=60)
    parser.add_argument('--discovery-timeout',type=float,default=60)
    parser.add_argument('--output',type=Path,default=Path(__file__).resolve().parent/'results'/datetime.datetime.now().strftime('%Y%m%d_%H%M%S'))
    args=parser.parse_args()
    if args.duration<=0 or args.discovery_timeout<=0:parser.error('durations must be positive')
    run(args)
