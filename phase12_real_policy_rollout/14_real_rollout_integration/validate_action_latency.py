"""Bounded prediction-only dispatch-latency probe. No physical sink or IO client.

Exercises production frame selection, warm-up, required JPEG/JSONL persistence,
fresh robot-state read, consecutive settle, then measures age (never dispatches).
"""
import argparse,json,math,os,threading,time
from pathlib import Path
from urllib.request import urlopen
from run_arm_only_once import FsyncJsonlLogger,JointStateStartup,validate_identity
from rollout_latency import select_fresh_frame,predict_frame,warm_up,archive_request_jpeg,deferred_artifacts
from settle_window import wait_for_settle,SETTLE_CLEAN_SECONDS
from camera_inference_timing import CAMERA_STREAM_TIMEOUT_S, camera_stream_lost
import yaml

def run(output,count=5):
    if not 1<=count<=20:raise ValueError('bounded_sample_count_required')
    output.mkdir(parents=True,exist_ok=False)
    def save(name,obj):(output/name).write_text(json.dumps(obj,indent=2,allow_nan=False))
    result=dict(status='BLOCKED',physical_commands=0,transport_created=False,samples=[],camera_stream_timeout_s=CAMERA_STREAM_TIMEOUT_S,camera_inference_start_age_limit_s=.5)
    limits=yaml.safe_load((Path(__file__).parent.parent/'01_configs/safety_limits.yaml').read_text())
    age_limit=limits['max_action_age_sec']
    try:
        with urlopen('http://127.0.0.1:8766/health',timeout=3) as response:health=json.load(response)
        validate_identity(health,'openvla')
        if health.get('status')!='ready':raise RuntimeError('model_not_ready')
        save('model_health.json',health)
    except Exception as exc:
        result['reason']='model_preflight:'+str(exc);save('summary.json',result);print(json.dumps(result));return
    import rclpy
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import Image,JointState
    from dsr_msgs2.srv import GetRobotState
    os.environ['ROS_LOCALHOST_ONLY']='1';rclpy.init()
    node=rclpy.create_node('prediction_only_action_latency_probe')
    executor=SingleThreadedExecutor();executor.add_node(node)
    lock=threading.RLock();gate=JointStateStartup(time.monotonic());samples=[];cam=[None];errors=[];halt=threading.Event()
    def joint(m):
        now=time.monotonic();stamp=m.header.stamp.sec+m.header.stamp.nanosec/1e9
        row=dict(receive=now,source=stamp,header_age=node.get_clock().now().nanoseconds/1e9-stamp,
            valid=len(m.name)==len(m.position)==len(m.velocity)==6 and set(m.name)=={f'joint_{i}' for i in range(1,7)} and all(math.isfinite(v) for v in list(m.position)+list(m.velocity)),
            names=list(m.name),position=list(m.position),velocity=list(m.velocity))
        with lock:gate.sample(row);samples.append(row)
    def image(m):
        with lock:cam[0]=(m,time.monotonic())
    node.create_subscription(JointState,'/dsr01/joint_states',joint,qos_profile_sensor_data)
    node.create_subscription(Image,'/zed/zed_node/rgb/color/rect/image',image,qos_profile_sensor_data)
    # ONLY a scalar read-only service client. No motion/gripper/stop interfaces.
    state_client=node.create_client(GetRobotState,'/dsr01/system/get_robot_state')
    def spin():
        try:
            while not halt.is_set():executor.spin_once(timeout_sec=.005)
        except Exception as exc:errors.append(str(exc))
    thread=threading.Thread(target=spin);thread.start()
    def check():
        with lock:s=gate.status(time.monotonic());latest=cam[0]
        if errors:raise RuntimeError('executor_error:'+errors[0])
        if s['fault']:raise RuntimeError('jointstate:'+s['fault'])
        if s['phase']=='RUNTIME_READY' and latest and camera_stream_lost(latest[1],time.monotonic()):raise RuntimeError('camera_stream_loss')
        return s
    def snapshot():
        with lock:return cam[0]
    def predict():return predict_frame(select_fresh_frame(snapshot,check),lambda:node.get_clock().now().nanoseconds/1e9,check)
    def settle():
        with lock:cursor=[len(samples)]
        def drain():
            with lock:rs=[dict(r) for r in samples[cursor[0]:]];cursor[0]=len(samples);return rs
        return wait_for_settle(drain,check,clock=time.monotonic,sleep=time.sleep)
    def state():
        check()
        if not state_client.wait_for_service(timeout_sec=2):raise RuntimeError('robot_state_service_unavailable')
        future=state_client.call_async(GetRobotState.Request());end=time.monotonic()+2
        while not future.done() and time.monotonic()<end:check();time.sleep(.002)
        if not future.done():raise TimeoutError('robot_state_getter_timeout')
        response=future.result()
        if not response.success or response.robot_state!=1:raise RuntimeError('robot_not_STANDBY')
    try:
        while check()['phase']!='RUNTIME_READY':time.sleep(.005)
        warm_up(predict,check,save,max_action_age_s=age_limit,settle_s=SETTLE_CLEAN_SECONDS)
        with FsyncJsonlLogger(output/'timing.jsonl') as log:
            for i in range(count):
                settle();s=predict();timing=s['timing'];end=s['t1']
                archive_request_jpeg(output/f'camera_{i:03d}_model_input.jpg',s['jpeg'])
                log.append(dict(event='PREDICTION',sample=i,raw_action=s['raw'],camera_timing=timing,command_issued=False))
                logged=time.monotonic();state();state_done=time.monotonic();settle()
                log.append(dict(event='PRE_DISPATCH_SETTLE_COMPLETE',sample=i,command_issued=False))
                checked=time.monotonic();age=checked-s['t0']
                row=dict(sample=i,camera_timing=timing,required_logging_s=logged-end,state_check_s=state_done-logged,
                    settle_and_log_s=checked-state_done,action_age_at_would_dispatch_s=age,age_limit_s=age_limit,
                    age_gate_pass=0<=age<=age_limit,command_requested=False,command_issued=False,
                    source_monotonic=s['t0'],raw_action=s['raw'])
                result['samples'].append(row);log.append(row)
                deferred_artifacts(output,i,s['msg'],s['rgb'],s['jpeg']);check()
        result['status']='PASS' if all(r['age_gate_pass'] for r in result['samples']) else 'FAIL'
    except Exception as exc:result.update(status='FAIL',reason=str(exc))
    finally:
        halt.set();thread.join(3);executor.remove_node(node);executor.shutdown();node.destroy_node();rclpy.shutdown()
        save('summary.json',result);print(json.dumps(result,indent=2),flush=True)

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--output',required=True,type=Path);p.add_argument('--samples',type=int,default=5)
    a=p.parse_args();run(a.output,a.samples)
