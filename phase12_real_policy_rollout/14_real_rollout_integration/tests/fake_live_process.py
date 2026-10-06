"""Separate runner processes consuming actual local DDS observations and fake HTTP.
Not model research results. Fake data never authorizes physical motion.
"""
import os,sys,json,time,tempfile,threading,subprocess
from pathlib import Path
from http.server import BaseHTTPRequestHandler,ThreadingHTTPServer
if os.environ.get('ROS_DOMAIN_ID')!='231' or os.environ.get('ROS_LOCALHOST_ONLY')!='1':raise RuntimeError('isolated_domain_required')
import rclpy,yaml
from sensor_msgs.msg import Image,JointState
from std_msgs.msg import Float64MultiArray
from dsr_msgs2.msg import RobotState
from rclpy.executors import MultiThreadedExecutor
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT));from pre_real_rollout_check import REQUIRED

def run(model,fault=None):
    cfg=yaml.safe_load((ROOT/'configs'/f'real_{model}.yaml').read_text())
    health=dict(server_version='FAKE_TEST_ONLY',model=model,checkpoint=cfg['checkpoint'],variant=cfg['variant'],
        action_dim=7,chunk_size=5 if model=='oft' else 1,requires_proprio=False,center_crop=model=='oft',
        preprocessing=dict(color_order='RGB',source_dtype='uint8',tensor_dtype='bfloat16',processor={'FAKE':True},instruction_handling='FAKE_TEST_ONLY'))
    class Handler(BaseHTTPRequestHandler):
        def log_message(self,*a):pass
        def do_GET(self):self.reply(health)
        def do_POST(self):
            self.rfile.read(int(self.headers.get('Content-Length',0)))
            self.reply({'actions':[[0]*7]*5,'action':[0]*7,'fixture':True})
        def reply(self,value):
            self.send_response(200);self.send_header('Content-Type','application/json');self.end_headers();self.wfile.write(json.dumps(value).encode())
    port=8765 if model=='oft' else 8766
    first_prediction=[None]
    old_post=Handler.do_POST
    def tracked_post(self):
        if first_prediction[0] is None:first_prediction[0]=time.monotonic()
        old_post(self)
    Handler.do_POST=tracked_post
    server=ThreadingHTTPServer(('127.0.0.1',port),Handler)
    http=threading.Thread(target=server.serve_forever);http.start()
    node=rclpy.create_node('fake_observations_'+model)
    camera=node.create_publisher(Image,'/zed/zed_node/rgb/color/rect/image',10)
    joints=node.create_publisher(JointState,'/dsr01/joint_states',10)
    tcp=node.create_publisher(Float64MultiArray,'/doosan/current_pose',10)
    status=node.create_publisher(RobotState,'/phase12_test/robot_state',10)
    def publish():
        inject=first_prediction[0] is not None and time.monotonic()-first_prediction[0]>.3
        stamp=node.get_clock().now().to_msg();i=Image();i.header.stamp=stamp;i.width=i.height=32;i.step=96;i.encoding='rgb8';i.data=bytes([128])*3072
        if not (inject and fault=='camera'):camera.publish(i)
        j=JointState();j.header.stamp=stamp;j.name=['joint_1','joint_2','joint_4','joint_5','joint_3','joint_6'];j.position=[0.]*6;j.velocity=[0.]*6;j.effort=[float('nan')]*6
        if not (inject and fault=='joints'):joints.publish(j)
        t=Float64MultiArray();t.data=[400.,0.,500.,0.,0.,0.]
        if not (inject and fault=='tcp'):tcp.publish(t)
        s=RobotState();s.robot_state=5 if inject and fault=='protective' else 1;s.disconnected=False
        if not (inject and fault=='state'):status.publish(s)
    node.create_timer(.01,publish);executor=MultiThreadedExecutor();executor.add_node(node)
    spin=threading.Thread(target=executor.spin);spin.start()
    try:
        with tempfile.TemporaryDirectory() as d:
            path=Path(d);evidence={k:True for k in REQUIRED};evidence.update(source='FAKE_TEST_GRAPH',verified_wall_time=time.time(),phase='alignment',verified_model_health_contract=health,verified_robot_state_topic='/phase12_test/robot_state')
            (path/'evidence.json').write_text(json.dumps(evidence))
            result=subprocess.run([sys.executable,str(ROOT/'run_real_rollout.py'),'--dry-run','--live','--model',model,
                '--protocol','short_horizon','--output',str(path/'log.jsonl'),'--preflight-evidence',str(path/'evidence.json'),
                '--session-results',str(path/'stages')],capture_output=True,text=True,timeout=20)
            if result.returncode and not fault:raise RuntimeError(result.stderr)
            rows=[json.loads(x) for x in (path/'log.jsonl').read_text().splitlines()]
            actions=[r for r in rows if r.get('event')=='ACTION']
            if any(r['command_issued'] for r in actions):raise AssertionError('command_issued')
            if fault:
                if not any(r.get('event')=='ABORT' for r in rows):raise AssertionError('no_abort:'+str(fault))
                print(model+': '+fault+' LOSS_FAIL_CLOSED_PASS')
            else:
                if len(actions)!=10:raise AssertionError({'count':len(actions),'blockers':[r['safety_blockers'] for r in actions],'aborts':[r for r in rows if r.get('event')=='ABORT']})
                print(model+': PROCESS_DRY_RUN_PASS 10 actions / 0 physical commands')
    finally:
        executor.shutdown();spin.join();node.destroy_node();server.shutdown();http.join();server.server_close()

if __name__=='__main__':
    rclpy.init()
    try:
        run('openvla');run('oft')
        for fault in ('camera','joints','tcp','state','protective'):run('openvla',fault)
    finally:rclpy.shutdown()
