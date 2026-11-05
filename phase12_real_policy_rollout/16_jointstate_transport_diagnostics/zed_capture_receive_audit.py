"""Passive ZED diagnostics + image/camera-info receive audit. No camera SDK open.

SDK diagnostic rates are windowed aggregates, not individual grab timestamps.
Image header stamps describe published source images; do not identify grab vs
publish stalls by themselves. All raw evidence is retained for this distinction.
"""
import argparse,json,os,time,math
from pathlib import Path
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image,CameraInfo
from diagnostic_msgs.msg import DiagnosticArray

def run(out,duration):
    out.mkdir(parents=True,exist_ok=False)
    os.environ['ROS_LOCALHOST_ONLY']='1';rclpy.init()
    node=rclpy.create_node('zed_passive_capture_receive_audit')
    executor=SingleThreadedExecutor();executor.add_node(node)
    streams={'image':[],'camera_info':[]};diagnostics=[];host=[]
    first=[None];events=[]
    def sample(kind,msg):
        t=time.monotonic();wall=time.time();stamp=msg.header.stamp.sec+msg.header.stamp.nanosec/1e9
        row=dict(receive_monotonic=t,receive_wall=wall,source_timestamp=stamp,
            source_age_s=node.get_clock().now().nanoseconds/1e9-stamp)
        previous=streams[kind][-1] if streams[kind] else None
        if previous:
            row.update(receive_gap_s=t-previous['receive_monotonic'],source_gap_s=stamp-previous['source_timestamp'])
            if row['receive_gap_s']>=.1 or row['source_gap_s']>=.1:
                events.append(dict(stream=kind,**row,previous_source=previous['source_timestamp'],previous_receive=previous['receive_monotonic']))
        if kind=='image':
            row.update(width=msg.width,height=msg.height,encoding=msg.encoding)
            if first[0] is None:first[0]=t
        streams[kind].append(row)
    def diagnostic(msg):
        diagnostics.append(dict(receive_monotonic=time.monotonic(),receive_wall=time.time(),
            source_stamp=msg.header.stamp.sec+msg.header.stamp.nanosec/1e9,
            statuses=[dict(level=s.level[0] if isinstance(s.level,bytes) else int(s.level),name=s.name,message=s.message,hardware_id=s.hardware_id,
                values=[dict(key=v.key,value=v.value) for v in s.values]) for s in msg.status if 'zed' in s.name.lower()]))
    node.create_subscription(Image,'/zed/zed_node/rgb/color/rect/image',lambda m:sample('image',m),qos_profile_sensor_data)
    node.create_subscription(CameraInfo,'/zed/zed_node/rgb/color/rect/camera_info',lambda m:sample('camera_info',m),qos_profile_sensor_data)
    node.create_subscription(DiagnosticArray,'/diagnostics',diagnostic,10)
    start=time.monotonic();next_host=start;next_update=start+30;error=None
    try:
        while True:
            now=time.monotonic()
            if first[0] is None and now-start>15:raise TimeoutError('no_image_within_15s')
            if first[0] is not None and now-first[0]>=duration:break
            executor.spin_once(timeout_sec=.005)
            if now>=next_host:
                host.append(dict(receive_monotonic=now,wall=time.time(),loadavg=os.getloadavg(),proc_stat=Path('/proc/stat').read_text().splitlines()[0]))
                next_host=now+1
            if now>=next_update:
                print(json.dumps(dict(elapsed_s=round(now-start,1),image_count=len(streams['image']),diagnostic_count=len(diagnostics),
                    receive_gaps_ge_500ms=sum(e.get('receive_gap_s',0)>=.5 for e in events if e['stream']=='image'))),flush=True)
                next_update=now+30
    except Exception as exc:error=str(exc)
    finally:
        end=time.monotonic()
        summary=dict(duration_requested_s=duration,observation_duration_s=None if first[0] is None else end-first[0],error=error,
            physical_commands=0,settings_changed=False,diagnostic_messages=len(diagnostics),streams={},
            limitation='SDK Data Capture frequency/processing time are aggregate diagnostics, not per-grab timestamps. Image header gaps alone do not separate grabbing from wrapper publishing.')
        for kind,rs in streams.items():
            gaps=[r for r in rs if 'receive_gap_s' in r]
            summary['streams'][kind]=dict(count=len(rs),receive_hz=(len(rs)-1)/(rs[-1]['receive_monotonic']-rs[0]['receive_monotonic']) if len(rs)>1 else None,
                max_receive_gap_s=max((r['receive_gap_s'] for r in gaps),default=None),max_source_gap_s=max((r['source_gap_s'] for r in gaps),default=None),
                max_source_age_s=max((r['source_age_s'] for r in rs),default=None),receive_gap_ge_100ms=sum(r['receive_gap_s']>=.1 for r in gaps),
                receive_gap_ge_500ms=sum(r['receive_gap_s']>=.5 for r in gaps),source_gap_ge_500ms=sum(r['source_gap_s']>=.5 for r in gaps),
                nonincreasing_source_stamps=sum(r['source_gap_s']<=0 for r in gaps),latest=rs[-1] if rs else None)
        for name,rs in [('images.jsonl',streams['image']),('camera_info.jsonl',streams['camera_info']),('diagnostics.jsonl',diagnostics),('host.jsonl',host),('gap_events.jsonl',events)]:
            with (out/name).open('x') as f:
                for row in rs:f.write(json.dumps(row)+'\n')
        (out/'summary.json').write_text(json.dumps(summary,indent=2))
        executor.remove_node(node);executor.shutdown();node.destroy_node();rclpy.shutdown()
        print(json.dumps(summary,indent=2),flush=True)

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--output',required=True,type=Path);p.add_argument('--duration',type=float,default=120)
    a=p.parse_args()
    if not 1<=a.duration<=600:p.error('duration must be between 1 and 600 seconds')
    run(a.output,a.duration)
