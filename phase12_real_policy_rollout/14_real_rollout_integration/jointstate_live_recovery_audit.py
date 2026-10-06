"""Read-only controller queries and passive stream comparison; no robot commands."""
import csv, datetime, glob, json, math, os, signal, subprocess, time
from pathlib import Path
import rclpy
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import JointState
ROOT=Path(__file__).resolve().parent

def main():
    out=ROOT/'real_trials'/datetime.datetime.now().strftime('%Y%m%d_%H%M%S_jointstate_live_recovery_audit')
    out.mkdir(parents=True,exist_ok=False)
    def run(args,timeout):
        start=time.monotonic()
        p=subprocess.Popen(args,stdout=subprocess.PIPE,stderr=subprocess.STDOUT,text=True,start_new_session=True)
        expired=False
        try: text=p.communicate(timeout=timeout)[0]
        except subprocess.TimeoutExpired:
            expired=True;os.killpg(p.pid,signal.SIGINT)
            try:text=p.communicate(timeout=3)[0]
            except subprocess.TimeoutExpired:
                os.killpg(p.pid,signal.SIGTERM);text=p.communicate(timeout=3)[0]
        return dict(command=args,duration_s=time.monotonic()-start,timeout=expired,exit_code=p.returncode,output=text)
    def save(name,value):
        (out/name).write_text(json.dumps(value,indent=2))
    logs={Path(f):Path(f).stat().st_size for f in glob.glob('/home/ubuntu/.ros/log/ros2_control_node_*.log')}
    save('ros_graph.txt',run(['ros2','node','list'],10))
    save('jointstate_topic_info.txt',run(['ros2','topic','info','-v','/dsr01/joint_states'],10))
    # Installed ros2controlcli verbs instantiate only ListControllers / ListHardwareInterfaces requests.
    save('controller_status.txt',run(['ros2','control','list_controllers','-c','/dsr01/controller_manager'],12))
    save('hardware_interface_report.txt',run(['ros2','control','list_hardware_interfaces','-c','/dsr01/controller_manager'],12))
    save('driver_process.txt',run(['ps','-eo','pid,ppid,stat,pcpu,pmem,nlwp,rss,args'],5))
    rclpy.init();node=rclpy.create_node('phase12_jointstate_live_recovery_audit');rows={'best_effort':[],'reliable':[]}
    start=time.monotonic()
    def cb(kind):
        def receive(m):
            idx={name:i for i,name in enumerate(m.name)};canonical=[f'joint_{i}' for i in range(1,7)]
            missing=any(n not in idx for n in canonical)
            invalid=missing or any(len(v)!=len(m.name) or not all(math.isfinite(v[idx[n]]) for n in canonical) for v in (m.position,m.velocity))
            rows[kind].append(dict(sequence=len(rows[kind]),receive_monotonic=time.monotonic(),receive_wall=time.time(),source_ns=m.header.stamp.sec*10**9+m.header.stamp.nanosec,joint_names=json.dumps(list(m.name)),position=json.dumps(list(m.position)),velocity=json.dumps(list(m.velocity)),invalid=invalid,missing_joint=missing))
        return receive
    for kind,rel,dur in [('best_effort',ReliabilityPolicy.BEST_EFFORT,DurabilityPolicy.VOLATILE),('reliable',ReliabilityPolicy.RELIABLE,DurabilityPolicy.TRANSIENT_LOCAL)]:
        node.create_subscription(JointState,'/dsr01/joint_states',cb(kind),QoSProfile(depth=1000,reliability=rel,durability=dur))
    hzfile=(out/'topic_hz.txt').open('x');echofile=(out/'topic_echo_once.txt').open('x')
    hz=subprocess.Popen(['ros2','topic','hz','/dsr01/joint_states'],stdout=hzfile,stderr=subprocess.STDOUT,start_new_session=True)
    echo=subprocess.Popen(['ros2','topic','echo','/dsr01/joint_states','--once'],stdout=echofile,stderr=subprocess.STDOUT,start_new_session=True)
    while time.monotonic()-start<30:rclpy.spin_once(node,timeout_sec=.02)
    now=time.monotonic()
    endpoints=[dict(name=e.node_name,namespace=e.node_namespace,gid=list(e.endpoint_gid),type=e.topic_type,qos=str(e.qos_profile)) for e in node.get_publishers_info_by_topic('/dsr01/joint_states')]
    for p in (hz,echo):
        if p.poll() is None:
            os.killpg(p.pid,signal.SIGINT)
            try:p.wait(timeout=3)
            except subprocess.TimeoutExpired:os.killpg(p.pid,signal.SIGTERM);p.wait(timeout=3)
    hzfile.close();echofile.close();node.destroy_node();rclpy.shutdown()
    def stats(data):
        sg=[(b['source_ns']-a['source_ns'])/1e9 for a,b in zip(data,data[1:])];rg=[b['receive_monotonic']-a['receive_monotonic'] for a,b in zip(data,data[1:])]
        result=dict(count=len(data),first_receive_delay=data[0]['receive_monotonic']-start if data else None,receive_rate=(len(data)-1)/(data[-1]['receive_monotonic']-data[0]['receive_monotonic']) if len(data)>1 else 0,max_source_gap=max(sg,default=None),max_receive_gap=max(rg,default=None),latest_age=now-data[-1]['receive_monotonic'] if data else None,invalid=sum(d['invalid'] for d in data),duplicate=sum(g==0 for g in sg),regression=sum(g<0 for g in sg))
        recent=[d for d in data if d['receive_monotonic']>=now-20]
        a=[(b['source_ns']-x['source_ns'])/1e9 for x,b in zip(recent,recent[1:])];b=[y['receive_monotonic']-x['receive_monotonic'] for x,y in zip(recent,recent[1:])]
        result['recent_20s_pass']=len(recent)>1 and now-recent[0]['receive_monotonic']>=19.9 and all(0<g<.1 for g in a) and all(0<g<.1 for g in b) and not any(d['invalid'] for d in recent) and result['latest_age']<.5
        return result
    summary={k:stats(v) for k,v in rows.items()}
    for kind,data in rows.items():
        with (out/(kind+'.csv')).open('x',newline='') as f:
            w=csv.DictWriter(f,fieldnames=list(data[0]) if data else ['sequence','receive_monotonic','receive_wall','source_ns','joint_names','position','velocity','invalid','missing_joint']);w.writeheader();w.writerows(data)
    # Bag-only phase: subscribers above destroyed, no simultaneous CLI graph polling.
    bagfile=(out/'rosbag_stdout.txt').open('x');bagstart=time.monotonic()
    bag=subprocess.Popen(['ros2','bag','record','/dsr01/joint_states','-o',str(out/'jointstate_bag')],stdout=bagfile,stderr=subprocess.STDOUT,start_new_session=True)
    try:bag.wait(timeout=25)
    except subprocess.TimeoutExpired:
        os.killpg(bag.pid,signal.SIGINT)
        try:bag.wait(timeout=8)
        except subprocess.TimeoutExpired:os.killpg(bag.pid,signal.SIGTERM);bag.wait(timeout=3)
    bagfile.close();save('rosbag_metadata.txt',run(['ros2','bag','info',str(out/'jointstate_bag')],8))
    import sqlite3
    bag_times=[]
    for db in (out/'jointstate_bag').glob('*.db3'):
        conn=sqlite3.connect('file:'+str(db)+'?mode=ro',uri=True)
        bag_times.extend(x[0] for x in conn.execute("SELECT timestamp FROM messages WHERE topic_id IN (SELECT id FROM topics WHERE name='/dsr01/joint_states') ORDER BY timestamp"));conn.close()
    newlogs=[]
    for p,offset in logs.items():
        with p.open(errors='replace') as f:f.seek(offset);s=f.read()
        if s:newlogs.append(str(p)+'\n'+s)
    (out/'driver_recent.log').write_text('\n'.join(newlogs) or 'No newly appended ros2_control_node log bytes during this audit.\n')
    verdict='JOINTSTATE_RECOVERED_SPONTANEOUSLY' if all(s['recent_20s_pass'] for s in summary.values()) and bag_times else 'STILL_INCONCLUSIVE'
    if endpoints and all(s['count']==0 for s in summary.values()) and not bag_times:verdict='JOINTSTATE_PUBLISHER_PRESENT_BUT_NOT_PUBLISHING'
    result=dict(subscribers=summary,publishers=endpoints,rosbag_count=len(bag_times),rosbag_phase_elapsed=time.monotonic()-bagstart,rosbag_receive_rate=(len(bag_times)-1)/((bag_times[-1]-bag_times[0])/1e9) if len(bag_times)>1 else 0,rosbag_first_receive_delay='UNVERIFIED_CLOCK_DOMAIN_ALIGNMENT',classification=verdict,hardware_feedback='UNKNOWN',controller_connection='UNKNOWN',physical_commands=0,next_allowed_stage='TCP_PRECHECK' if verdict=='JOINTSTATE_RECOVERED_SPONTANEOUSLY' else 'BLOCKED')
    save('jointstate_recovery_summary.json',result)
    (out/'JOINTSTATE_RECOVERY_REPORT.md').write_text('# JointState live recovery audit\n\n```json\n'+json.dumps(result,indent=2)+'\n```\n\nController queries are read-only; timeout is not proof of inactive controllers. Interface lists enumerate names/claims, not numeric values or freshness. No read-only numeric hardware interface value source was established. Publisher-present/not-publishing classification means no live samples observed by both QoS, CLI echo/hz and separate rosbag; it does not prove publish() was never called. Event silence cannot prove controller connected. No robot getter, motion, setter, restart or activation was performed. Bag timestamps are ROS/system receive time, not local monotonic.\n')
    print(str(out),flush=True);print(json.dumps(result,indent=2),flush=True)
if __name__=='__main__':main()
