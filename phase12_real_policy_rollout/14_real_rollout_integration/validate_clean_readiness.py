"""Live read-only readiness/preflight. No command clients or inference requests."""
import argparse,json,math,threading,time
from pathlib import Path
from urllib.request import urlopen
from jointstate_runtime_startup import JointStateStartup

def sample_hardware_ready(hardware):
    expected={'mode':('robot_mode',1),'state':('robot_state',1),'system':('robot_system',0)}
    return all(hardware.get(k,{}).get('response',{}).get(field)==value
               and hardware.get(k,{}).get('response',{}).get('success') is True
               for k,(field,value) in expected.items())

def run(output, runtime_duration=0., hardware_refresh=False, single_prediction=False):
    import rclpy
    from sensor_msgs.msg import JointState,Image
    from rclpy.qos import qos_profile_sensor_data
    from verify_live_model_server import validate_identity
    from pre_real_rollout_check import preflight
    output.mkdir(parents=True,exist_ok=False)
    __import__("os").environ["ROS_LOCALHOST_ONLY"] = "1"; rclpy.init();node=rclpy.create_node('phase12_clean_readiness_validation')
    state=JointStateStartup(time.monotonic());lock=threading.RLock();rows=[];camera=[None];latest_image=[None];halt=threading.Event()
    def joints(msg):
        now=time.monotonic();source=msg.header.stamp.sec+msg.header.stamp.nanosec/1e9
        valid=len(msg.name)==6 and set(msg.name)=={f'joint_{i}' for i in range(1,7)} and len(msg.position)==len(msg.velocity)==6 and all(math.isfinite(v) for v in list(msg.position)+list(msg.velocity))
        row=dict(receive=now,source=source,header_age=node.get_clock().now().nanoseconds/1e9-source,
                 valid=valid,names=list(msg.name),position=list(msg.position),velocity=list(msg.velocity))
        with lock:state.sample(row);rows.append(dict(row))
    def image(msg):
        with lock:
            camera[0]=dict(receive=time.monotonic(),encoding=msg.encoding,resolution=[msg.width,msg.height])
            latest_image[0]=(msg,camera[0]['receive'])
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
        runtime_start=time.monotonic()
        while not status['fault'] and time.monotonic()-runtime_start<runtime_duration:
            time.sleep(.01)
            with lock:status=state.status(time.monotonic())
        hardware={}
        if hardware_refresh and not status['fault']:
            import dsr_msgs2.srv as srv
            from rosidl_runtime_py.convert import message_to_ordereddict
            # Reviewed scalar read-only callbacks only; never pose/tool/motion APIs.
            for key,type_name,path in [('mode','GetRobotMode','/dsr01/system/get_robot_mode'),
                    ('state','GetRobotState','/dsr01/system/get_robot_state'),
                    ('system','GetRobotSystem','/dsr01/system/get_robot_system')]:
                with lock:status=state.status(time.monotonic())
                if status['fault']:
                    hardware[key]={'status':'NOT_CALLED_RUNTIME_FAULT'};continue
                service_type=getattr(srv,type_name)
                client=node.create_client(service_type,path)
                try:
                    if not client.wait_for_service(timeout_sec=1.):
                        hardware[key]={'status':'NOT_DISCOVERED'};continue
                    requested=time.time();started=time.monotonic()
                    future=client.call_async(service_type.Request())
                    while not future.done() and time.monotonic()-started<3.:
                        time.sleep(.01)
                        with lock:status=state.status(time.monotonic())
                        if status['fault']:break
                    if future.done():
                        try:hardware[key]={'status':'RESPONSE','response':message_to_ordereddict(future.result()),
                            'requested_at':requested,'latency_s':time.monotonic()-started,'provenance':'DRIVER_REPORTED'}
                        except Exception as exc:hardware[key]={'status':'ERROR','error':str(exc)}
                    else:hardware[key]={'status':'TIMEOUT_OR_RUNTIME_FAULT','requested_at':requested}
                finally:node.destroy_client(client)
            until=time.monotonic()+10.
            while time.monotonic()<until:
                time.sleep(.01)
                with lock:status=state.status(time.monotonic())
                if status['fault']:break
            (output/'hardware_getters.json').write_text(json.dumps(hardware,indent=2))
        # Current health only; never call /predict or robot getters/services.
        try:
            with urlopen('http://127.0.0.1:8766/health',timeout=3) as response:health=json.load(response)
            validate_identity(health,'openvla')
        except Exception as exc:model_error=str(exc)
        if single_prediction:
            import base64,hashlib,io
            from urllib.request import Request
            from live_observation import camera_rgb_image
            from run_joint_start_pose_once import ordered_degrees,TARGET,TOLERANCE_DEG
            with lock:
                joint=dict(rows[-1]) if rows else None
                prediction_gate=state.status(time.monotonic())
            actual=ordered_degrees(joint) if joint else None
            errors=[a-b for a,b in zip(actual,TARGET)] if actual else None
            position_match=bool(errors and max(map(abs,errors))<=TOLERANCE_DEG)
            save_pose=dict(current_joint_deg=actual,target_joint_deg=TARGET,error_deg=errors,
                max_abs_error_deg=max(map(abs,errors)) if errors else None,
                tolerance_deg=TOLERANCE_DEG,position_match=position_match,reset_executed=False,
                jointstate_source_timestamp=joint['source'] if joint else None)
            (output/'jointstate_start_pose_check.json').write_text(json.dumps(save_pose,indent=2))
            sample=dict(status='NOT_EXECUTED',raw_action=None,physical_commands=0)
            if not sample_hardware_ready(hardware):sample['reason']='HARDWARE_NOT_CURRENT_REAL_AUTO_STANDBY'
            elif not position_match:sample['reason']='START_POSE_OUTSIDE_EXISTING_TOLERANCE'
            elif model_error:sample['reason']='MODEL_HEALTH_FAILED'
            elif prediction_gate['fault']:sample['reason']='JOINTSTATE_RUNTIME_FAULT'
            else:
                try:
                    config=validate_identity(health,'openvla')
                    timeout=time.monotonic()+2.;selected=None
                    while time.monotonic()<timeout:
                        with lock:selected=latest_image[0]
                        if selected and 0<=time.monotonic()-selected[1]<=.01:break
                        time.sleep(.001)
                    if not selected or not 0<=time.monotonic()-selected[1]<=.01:raise ValueError('fresh_frame_timeout')
                    msg,received=selected
                    if msg.encoding!='bgra8' or [msg.width,msg.height]!=[1280,720]:raise ValueError('camera_contract')
                    rgb=camera_rgb_image(msg);buffer=io.BytesIO();rgb.save(buffer,format='JPEG',quality=95);jpeg=buffer.getvalue()
                    request=Request('http://127.0.0.1:8766/predict',data=json.dumps(dict(image_jpeg_base64=base64.b64encode(jpeg).decode(),instruction=config['instruction'])).encode(),headers={'Content-Type':'application/json'})
                    ros_start=node.get_clock().now().nanoseconds/1e9
                    started=time.monotonic()
                    if not 0<=started-received<.5:raise ValueError('camera_stale_at_inference_start')
                    with urlopen(request,timeout=10) as response:prediction=json.load(response)
                    finished=time.monotonic();ros_end=node.get_clock().now().nanoseconds/1e9;raw=prediction.get('action')
                    if not isinstance(raw,list) or len(raw)!=7 or not all(isinstance(x,(int,float)) and math.isfinite(x) for x in raw):raise ValueError('model_action_schema')
                    with lock:after=state.status(finished)
                    from camera_inference_timing import inference_timing
                    timing=inference_timing(received,started,finished,source_stamp=msg.header.stamp.sec+msg.header.stamp.nanosec/1e9,ros_start=ros_start,ros_end=ros_end)
                    camera_age=timing['camera_age_at_inference_start']
                    sample.update(timing)
                    sample.update(status='PASS' if camera_age<.5 and not after['fault'] else 'FAIL',raw_action=raw,
                        raw_translation_xyz=raw[:3],raw_translation_norm=math.dist(raw[:3],[0,0,0]),
                        raw_rotation_xyz=raw[3:6],raw_rotation_norm=math.dist(raw[3:6],[0,0,0]),gripper_closedness=raw[6],
                        instruction=config['instruction'],inference_latency_s=finished-started,camera_age_s=camera_age,
                        camera_failure=camera_age>=.5,jointstate_runtime=after,encoding=msg.encoding,resolution=[msg.width,msg.height],
                        camera_source_stamp=dict(sec=msg.header.stamp.sec,nanosec=msg.header.stamp.nanosec),
                        camera_receive_monotonic=received,source_to_response_age_s=node.get_clock().now().nanoseconds/1e9-(msg.header.stamp.sec+msg.header.stamp.nanosec/1e9),
                        tcp_source='FK_ESTIMATED_FLANGE',tcp_contract_verified=False,command_requested=False,
                        command_issued=False,delivered_action=None,executed_action=None)
                    rgb.save(output/'sample_camera_rgb.png')
                    (output/'sample_model_input.jpg').write_bytes(jpeg)
                    sample['source_image_sha256']=hashlib.sha256(bytes(msg.data)).hexdigest()
                    sample['model_jpeg_sha256']=hashlib.sha256(jpeg).hexdigest()
                except Exception as exc:sample.update(status='FAIL',reason=str(exc))
            (output/'sample_prediction.json').write_text(json.dumps(sample,indent=2))
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
        runtime_rows=[r for r in samples if state.ready_at is not None and r['receive']>=state.ready_at]
        result['current_session_runtime']=dict(duration_requested_s=runtime_duration,
            duration_observed_s=time.monotonic()-runtime_start, sample_count=len(runtime_rows),
            max_source_gap_s=max((r['source_gap'] for r in runtime_rows if r['source_gap'] is not None),default=None),
            max_receive_gap_s=max((r['receive_gap'] for r in runtime_rows if r['receive_gap'] is not None),default=None),
            source_gap_ge_100ms=sum((r['source_gap'] or 0)>=.1 for r in runtime_rows),
            receive_gap_ge_100ms=sum((r['receive_gap'] or 0)>=.1 for r in runtime_rows))
        graph=dict(topics=node.get_topic_names_and_types(),services=node.get_service_names_and_types())
        (output/'ros_graph_snapshot.json').write_text(json.dumps(graph,indent=2))
        for name,value in [('summary.json',result),('startup_events.json',events),('jointstate_samples.json',samples),('model_health.json',health),('preflight_evidence.json',evidence)]:
            (output/name).write_text(json.dumps(value,indent=2))
        print(json.dumps(result,indent=2),flush=True)
    finally:
        halt.set();thread.join(timeout=3);node.destroy_node();rclpy.shutdown()

if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--runtime-duration',type=float,default=0.)
    parser.add_argument('--hardware-refresh',action='store_true')
    parser.add_argument('--single-prediction',action='store_true')
    args=parser.parse_args()
    if args.runtime_duration<0:parser.error('--runtime-duration must be nonnegative')
    if args.single_prediction and not args.hardware_refresh:parser.error('--single-prediction requires --hardware-refresh')
    run(args.output,args.runtime_duration,args.hardware_refresh,args.single_prediction)
