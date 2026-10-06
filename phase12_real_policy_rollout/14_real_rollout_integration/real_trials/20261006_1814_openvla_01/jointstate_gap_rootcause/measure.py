"""Simultaneous passive subscribers and rosbag; no robot service clients."""
import argparse,csv,json,math,os,signal,subprocess,threading,time
from pathlib import Path
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile,ReliabilityPolicy,DurabilityPolicy,HistoryPolicy
from sensor_msgs.msg import JointState
ROOT=Path(__file__).resolve().parent
def writecsv(path,rows,emptykeys):
    with path.open('x',newline='') as f:
        w=csv.DictWriter(f,fieldnames=list(rows[0]) if rows else emptykeys);w.writeheader();w.writerows(rows);f.flush();os.fsync(f.fileno())
def main():
    p=argparse.ArgumentParser();p.add_argument('--run',required=True);a=p.parse_args()
    rows={'best_effort':[],'reliable':[]};hosts=[];stop=threading.Event()
    logpath=Path('/home/ubuntu/.ros/log/ros2_control_node_3017586_1791277811547.log')
    initial=logpath.stat().st_size
    baglog=(ROOT/(a.run+'_bag_stdout.txt')).open('x')
    bag=subprocess.Popen(['ros2','bag','record','/dsr01/joint_states','-o',str(ROOT/(a.run+'_rosbag')),
        '--qos-profile-overrides-path',str(ROOT/'bag_qos.yaml')],stdout=baglog,stderr=subprocess.STDOUT,start_new_session=True)
    rclpy.init();node=Node('phase12_gap_'+a.run)
    def cb(kind):
        def receive(m):
            rows[kind].append(dict(sequence=len(rows[kind]),receive_monotonic=time.monotonic(),receive_wall=time.time(),
                source_ros=m.header.stamp.sec+m.header.stamp.nanosec/1e9,joint_names=json.dumps(list(m.name)),
                position=json.dumps(list(m.position)),velocity=json.dumps(list(m.velocity)),
                invalid_values=not(len(m.position)==6 and len(m.velocity)==6 and all(math.isfinite(v) for v in list(m.position)+list(m.velocity))),
                missing_names=set(m.name)!=set('joint_'+str(i) for i in range(1,7))))
        return receive
    for kind,reliability,durability,depth in [('best_effort',ReliabilityPolicy.BEST_EFFORT,DurabilityPolicy.VOLATILE,5),('reliable',ReliabilityPolicy.RELIABLE,DurabilityPolicy.TRANSIENT_LOCAL,1000)]:
        node.create_subscription(JointState,'/dsr01/joint_states',cb(kind),QoSProfile(reliability=reliability,durability=durability,history=HistoryPolicy.KEEP_LAST,depth=depth))
    def monitor():
        while not stop.is_set():
            pid=Path('/proc/3017586');now=time.monotonic()
            hosts.append(dict(run=a.run,receive_monotonic=now,wall=time.time(),load=json.dumps(os.getloadavg()),
                cpu_stat=Path('/proc/stat').read_text().splitlines()[0],memory=Path('/proc/meminfo').read_text().replace('\n',';'),
                network=Path('/proc/net/dev').read_text().replace('\n',';'),
                driver_stat=(pid/'stat').read_text() if pid.exists() else 'MISSING',
                driver_schedstat=(pid/'schedstat').read_text() if pid.exists() else 'MISSING'))
            stop.wait(.2)
    worker=threading.Thread(target=monitor);worker.start();start=time.monotonic();wall=time.time()
    try:
        while time.monotonic()-start<120:rclpy.spin_once(node,timeout_sec=.02)
    finally:
        endwall=time.time();stop.set();worker.join();node.destroy_node();rclpy.shutdown()
        if bag.poll() is None:
            os.killpg(bag.pid,signal.SIGINT)
            try:bag.wait(timeout=10)
            except subprocess.TimeoutExpired:bag.terminate();bag.wait(timeout=5)
        baglog.close()
    for kind,data in rows.items():writecsv(ROOT/(a.run+'_'+kind+'.csv'),data,['sequence','receive_monotonic','receive_wall','source_ros'])
    writecsv(ROOT/(a.run+'_host.csv'),hosts,['run','wall'])
    with logpath.open() as f:f.seek(initial);newlogs=f.read()
    (ROOT/(a.run+'_driver_new.log')).write_text(newlogs)
    result=dict(run=a.run,start_wall=wall,end_wall=endwall,duration_s=time.monotonic()-start,
        counts={k:len(v) for k,v in rows.items()},bag_returncode=bag.returncode,
        external_command_detected=any(x in newlogs for x in ('movej_cb','movel_cb','set_robot_mode_cb','set_tool_digital_output_cb')),
        robot_command_calls_by_measurement=0,clock_domains='source ROS; subscriber receive monotonic/wall; bag receive ROS/system (NOT monotonic)')
    (ROOT/(a.run+'_metadata.json')).write_text(json.dumps(result,indent=2));print(json.dumps(result),flush=True)
if __name__=='__main__':main()
