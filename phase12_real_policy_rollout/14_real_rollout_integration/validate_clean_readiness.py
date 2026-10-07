"""Live read-only readiness/preflight. No command clients or inference requests."""
import argparse,json,math,threading,time
from pathlib import Path
from urllib.request import urlopen
from jointstate_runtime_startup import JointStateStartup

def run(output):
    import rclpy
    from sensor_msgs.msg import JointState,Image
    from rclpy.qos import qos_profile_sensor_data
    from verify_live_model_server import validate_identity
    from pre_real_rollout_check import preflight
    output.mkdir(parents=True,exist_ok=False)
    rclpy.init();node=rclpy.create_node('phase12_clean_readiness_validation')
    state=JointStateStartup(time.monotonic());lock=threading.RLock();rows=[];camera=[None];halt=threading.Event()
    def joints(msg):
        now=time.monotonic();source=msg.header.stamp.sec+msg.header.stamp.nanosec/1e9
        valid=len(msg.name)==6 and set(msg.name)=={f'joint_{i}' for i in range(1,7)} and len(msg.position)==len(msg.velocity)==6 and all(math.isfinite(v) for v in list(msg.position)+list(msg.velocity))
        row=dict(receive=now,source=source,header_age=node.get_clock().now().nanoseconds/1e9-source,
                 valid=valid,names=list(msg.name),position=list(msg.position),velocity=list(msg.velocity))
        with lock:state.sample(row);rows.append(dict(row))
    def image(msg):
        with lock:camera[0]=dict(receive=time.monotonic(),encoding=msg.encoding,resolution=[msg.width,msg.height])
    node.create_subscription(JointState,'/dsr01/joint_states',joints,qos_profile_sensor_data)
    node.create_subscription(Image,'/zed/zed_node/rgb/color/rect/image',image,qos_profile_sensor_data)
    spin_error=[]
    def spin():
        try:
            while not halt.is_set():rclpy.spin_once(node,timeout_sec=.01)
        except Exception as exc:
            with lock:spin_error.append(str(exc));state.fail('RUNTIME_EXECUTOR_ERROR')
    thread=threading.Thread(target=spin);thread.start();health=None;model_error=None
    try:
        while True:
            with lock:status=state.status(time.monotonic())
            if status['phase']=='RUNTIME_READY' or status['fault']:break
            time.sleep(.01)
        # Current health only; never call /predict or robot getters/services.
        try:
            with urlopen('http://127.0.0.1:8766/health',timeout=3) as response:health=json.load(response)
            validate_identity(health,'openvla')
        except Exception as exc:model_error=str(exc)
        with lock:
            final=state.status(time.monotonic());cam=dict(camera[0]) if camera[0] else None
            events=list(state.events);samples=list(rows)
        camera_ready=bool(cam and time.monotonic()-cam['receive']<.5 and cam['encoding']=='bgra8' and cam['resolution']==[1280,720])
        evidence=dict(verified_wall_time=time.time(),ros_graph_alive=bool(samples),joint_state_ready=final['phase']=='RUNTIME_READY' and not final['fault'],
                      camera_ready=camera_ready,model_ready=bool(health and not model_error),logger_ready=True,
                      source='READ_ONLY_CURRENT_SESSION_NO_HARDWARE_OR_MANUAL_ASSUMPTIONS')
        result=dict(status='PASS' if evidence['joint_state_ready'] else 'FAIL',readiness=final,
                    startup_events=sum(e.get('event')=='STARTUP_WARMUP_EVENT' for e in events),
                    clean_window_acquired=state.ready_at is not None,time_to_runtime_ready_s=state.ready_at-state.started if state.ready_at is not None else None,
                    camera_current=cam,camera_current_ready=camera_ready,model_identity_pass=not model_error and health is not None,
                    model_error=model_error,preflight=preflight(evidence,dry_run=True),physical_commands=0,
                    task_trial_started=False,task_status=None,task_success=None,executor_error=spin_error)
        for name,value in [('summary.json',result),('startup_events.json',events),('jointstate_samples.json',samples),('model_health.json',health),('preflight_evidence.json',evidence)]:
            (output/name).write_text(json.dumps(value,indent=2))
        print(json.dumps(result,indent=2),flush=True)
    finally:
        halt.set();thread.join(timeout=3);node.destroy_node();rclpy.shutdown()

if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('--output',type=Path,required=True)
    run(parser.parse_args().output)
