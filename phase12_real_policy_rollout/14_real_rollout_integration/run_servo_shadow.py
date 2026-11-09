"""Real camera/model + independent 20Hz servo SHADOW. No command publisher/IO."""
import argparse,json,math,os,threading,time
from pathlib import Path
import yaml
from run_arm_only_once import JointStateStartup,FsyncJsonlLogger,validate_identity
from run_real_rollout import SafetyPipeline,RuntimeState,CanonicalAction
from rollout_latency import predict_frame,select_fresh_frame
from camera_inference_timing import camera_stream_lost
from servo_experimental.controller import ServoShadowController
from urllib.request import urlopen


def run(output,duration):
    if not 1<=duration<=120:raise ValueError('shadow_duration_must_be_1_to_120s')
    output.mkdir(parents=True,exist_ok=False)
    def save(name,obj):(output/name).write_text(json.dumps(obj,indent=2,allow_nan=False))
    result=dict(scope='ARM_ONLY_SERVO_SHADOW',physical_commands=0,gripper_commands=0,
                transport_created=False,physical_servo_ready=False,predictions=0,servo_candidates=0)
    with urlopen('http://127.0.0.1:8766/health',timeout=3) as r:health=json.load(r)
    validate_identity(health,'openvla')
    if health.get('status')!='ready':raise RuntimeError('model_not_ready')
    save('model_health.json',health)
    import rclpy
    from sensor_msgs.msg import Image,JointState
    from rclpy.qos import qos_profile_sensor_data
    from dsr_msgs2.srv import GetCurrentPosx
    from rosidl_runtime_py.convert import message_to_ordereddict
    os.environ['ROS_LOCALHOST_ONLY']='1';rclpy.init();node=rclpy.create_node('servo_shadow_no_command_transport')
    lock=threading.RLock();halt=threading.Event();producer_stop=threading.Event();camera=[None];gate=JointStateStartup(time.monotonic());errors=[];fault=[None]
    def image(m):
        with lock:camera[0]=(m,time.monotonic())
    def joint(m):
        stamp=m.header.stamp.sec+m.header.stamp.nanosec/1e9
        valid=(set(m.name)=={f'joint_{i}' for i in range(1,7)} and len(m.name)==len(m.position)==len(m.velocity)==6 and all(math.isfinite(v) for v in list(m.position)+list(m.velocity)))
        with lock:gate.sample(dict(receive=time.monotonic(),source=stamp,header_age=node.get_clock().now().nanoseconds/1e9-stamp,valid=valid))
    node.create_subscription(Image,'/zed/zed_node/rgb/color/rect/image',image,qos_profile_sensor_data)
    node.create_subscription(JointState,'/dsr01/joint_states',joint,qos_profile_sensor_data)
    def spin():
        try:
            while not halt.is_set():rclpy.spin_once(node,timeout_sec=.005)
        except Exception as e:errors.append(str(e))
    spinner=threading.Thread(target=spin);spinner.start();producer=None;controller=None
    def check():
        with lock:s=gate.status(time.monotonic());cam=camera[0]
        if errors:raise RuntimeError('executor_error:'+errors[0])
        if s['fault']:raise RuntimeError('jointstate:'+s['fault'])
        if s['phase']=='RUNTIME_READY' and cam and camera_stream_lost(cam[1],time.monotonic()):raise RuntimeError('camera_stream_loss')
        return s
    def snapshot():
        with lock:return camera[0]
    try:
        while check()['phase']!='RUNTIME_READY':time.sleep(.005)
        # The only service client in this launcher is this existing read-only getter.
        client=node.create_client(GetCurrentPosx,'/dsr01/aux_control/get_current_posx')
        if not client.wait_for_service(timeout_sec=2):raise RuntimeError('current_pose_unavailable')
        req=GetCurrentPosx.Request();req.ref=0;f=client.call_async(req);end=time.monotonic()+2
        while not f.done() and time.monotonic()<end:check();time.sleep(.002)
        if not f.done():raise RuntimeError('current_pose_timeout')
        response=message_to_ordereddict(f.result())
        if not response['success']:raise RuntimeError('current_pose_failed')
        pose=response['task_pos_info'][0]['data'][:6]
        save('initial_pose.json',dict(controller_pose=pose,contract='BASE/mm/ZYZ_degree',formal_tcp_verified=False,physical_execution_authorized=False))
        limits=yaml.safe_load((Path(__file__).parent.parent/'01_configs/safety_limits.yaml').read_text())
        controller=ServoShadowController(pose,limits)
        pipeline=SafetyPipeline('openvla',operator_confirmed_initial_open=False,arm_only=True)
        save('contract.json',dict(rate_hz=20,production_limits=limits,policy_limiter_dt_s=pipeline.runtime.limiter.dt,
             stream_reference_source='/home/ubuntu/robot_ws/src/openvla_doosan_runtime/openvla_doosan_runtime/servol_stream_test_node.py',
             simulated_pose_not_physical_feedback=True,physical_publish_enabled=False,
             gripper_disabled=True,topic_candidate='/dsr01/servol_stream',vendor_ack_available=False,
             physical_blockers=['SERVO_MINIMUM_MOTION_NOT_VERIFIED','SERVO_STOP_PATH_NOT_VERIFIED','SERVO_BASE_REFERENCE_AND_TARGET_TIME_NOT_PHYSICALLY_VERIFIED']))
        with FsyncJsonlLogger(output/'predictions.jsonl') as predictions,FsyncJsonlLogger(output/'servo_candidates.jsonl') as servo:
            deadline=time.monotonic()+duration
            def predict():
                try:
                    while not halt.is_set() and not producer_stop.is_set() and time.monotonic()<deadline:
                        check();s=predict_frame(select_fresh_frame(snapshot,check),lambda:node.get_clock().now().nanoseconds/1e9,check)
                        if halt.is_set() or producer_stop.is_set() or time.monotonic()>=deadline:return
                        with lock:current=controller.pose()
                        action=CanonicalAction.from_vector(s['raw'],timestamp_monotonic=s['t0'],sequence_id='servo-shadow-'+str(result['predictions']),source_model='openvla')
                        d=pipeline.inspect(action,RuntimeState(time.monotonic(),tuple(x/1000 for x in current[:3]),tuple(current[3:]),'alignment'))
                        predictions.append(dict(raw_action=s['raw'],camera_timing=s['timing'],decision=d.as_dict(),candidate=d.candidate,command_issued=False))
                        if not d.accepted:raise RuntimeError('safety_rejection:'+','.join(d.reason))
                        check()
                        with lock:controller.set_target(d.candidate['target_pose_candidate_mm_zyz_deg'],s['t0'],time.monotonic())
                        result['predictions']+=1
                except Exception as e:
                    with lock:
                        fault[0]=str(e);controller.abort(str(e))
                    halt.set()
            producer=threading.Thread(target=predict);producer.start();next_tick=time.monotonic()
            while not halt.is_set() and time.monotonic()<deadline:
                check();now=time.monotonic()
                if now<next_tick:time.sleep(min(next_tick-now,.002));continue
                with lock:row=controller.tick(now)
                if row:servo.append(row);result['servo_candidates']+=1
                next_tick=now+controller.period
            producer_stop.set();producer.join(5)
            if fault[0]:raise RuntimeError(fault[0])
            result['status']='SERVO_SHADOW_PASS' if result['predictions'] and result['servo_candidates'] else 'SERVO_SHADOW_INVALID'
            if not result['predictions']:result['reason']='no_valid_prediction'
    except Exception as e:
        result.update(status='SERVO_SHADOW_INVALID',reason=str(e))
        if controller:controller.abort(str(e))
    finally:
        producer_stop.set()
        if producer:producer.join(5)
        halt.set()
        spinner.join(3);node.destroy_node();rclpy.shutdown()
        if fault[0]:result.update(status='SERVO_SHADOW_INVALID',reason=fault[0])
        result['would_request_stop_on_fault']=bool(controller and controller.aborted)
        result['physical_stop_executed']=False
        save('summary.json',result);print(json.dumps(result,indent=2))


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--output',required=True,type=Path);p.add_argument('--duration',type=float,default=30)
    p.add_argument('--execute',action='store_true',help='Blocked: physical servo requires separate validation')
    a=p.parse_args()
    if a.execute:p.error('SERVO_PHYSICAL_VALIDATION_REQUIRED: no physical servo transport is enabled')
    run(a.output,a.duration)
