"""Single-participant, getter-allowlisted final precheck; no physical capability."""
import base64,io,json,math,re,subprocess,threading,time
from pathlib import Path
from urllib.request import Request,urlopen
from jointstate_runtime_startup import JointStateStartup
from verify_live_model_server import validate_identity
from live_observation import camera_rgb_image
ROOT=Path(__file__).resolve().parent
SOURCE=Path('/home/ubuntu/robot_ws/src/doosan-robot2')
REFRESH={'mode':('GetRobotMode','/dsr01/system/get_robot_mode'),
         'state':('GetRobotState','/dsr01/system/get_robot_state'),
         'system':('GetRobotSystem','/dsr01/system/get_robot_system')}

def source_audit():
    cpp=(SOURCE/'dsr_controller2/src/dsr_controller2.cpp').read_text();entries=[]
    for name,callback in [('GetCurrentTcp','get_current_tcp_cb'),('GetCurrentTool','get_current_tool_cb'),
            ('GetCurrentPose','get_current_pose_cb'),('GetCurrentPosx','get_current_posx_cb'),
            ('GetCurrentToolFlangePosx','get_current_tool_flange_posx_cb'),
            ('GetRobotMode','get_robot_mode_cb'),('GetRobotState','get_robot_state_cb'),
            ('GetRobotSystem','get_robot_system_cb'),('GetLastAlarm','get_last_alarm_cb')]:
        at=cpp.find('auto '+callback);end=cpp.find('\n};',at)
        body=cpp[at:end+3] if at>=0 else 'NOT_FOUND'
        definitions=list(SOURCE.glob('dsr_msgs2/srv/**/'+name+'.srv'))
        risk='READ_ONLY_BUT_STABILITY_UNVERIFIED'
        if name in ('GetCurrentPose','GetCurrentPosx'):risk='UNSAFE_OR_NULL_RISK'
        if name=='GetCurrentToolFlangePosx':risk='READ_ONLY_BUT_STABILITY_UNVERIFIED'
        entries.append(dict(service_type=name,callback_line=cpp[:at].count('\n')+1 if at>=0 else None,
            callback_file=str(SOURCE/'dsr_controller2/src/dsr_controller2.cpp'),callback=body,
            srv_definition=definitions[0].read_text() if definitions else None,classification=risk,
            known_posx_feedback_loss=name=='GetCurrentPosx',called=False))
    return entries

def main():
    import rclpy,yaml
    from sensor_msgs.msg import JointState,Image
    from tf2_msgs.msg import TFMessage
    from rcl_interfaces.msg import Log
    from rclpy.qos import qos_profile_sensor_data,QoSProfile,ReliabilityPolicy,DurabilityPolicy
    from rosidl_runtime_py.utilities import get_message
    from rosidl_runtime_py.convert import message_to_ordereddict
    import dsr_msgs2.srv as services
    out=ROOT/'real_trials'/(time.strftime('%Y%m%d_%H%M%S')+'_final_hardware_tcp_safety_precheck');out.mkdir()
    def save(name,value):(out/(name+'.json')).write_text(json.dumps(value,indent=2))
    audit=source_audit();save('tcp_tool_sources',audit);save('cartesian_pose_sources',audit)
    # Search candidates only; never treat historical creation as current active binding.
    search=subprocess.run(['rg','-n','-i','--max-count','4','--glob','*.yaml','--glob','*.json','--glob','*.py','--glob','*.cpp','--glob','*.urdf',
        'Tool_v1|ConfigCreateTcp|ConfigCreateTool|tcp_offset|tool_offset|tool_frame|tcp_name',
        str(SOURCE),'/home/ubuntu/robot_ws/install','/home/ubuntu/robot_ws/src/openvla_doosan_runtime',str(ROOT.parent/'01_configs'),str(ROOT.parents[2]/'runtime_state')],capture_output=True,text=True,timeout=30)
    (out/'offset_candidate_search.txt').write_text(search.stdout)
    save('tcp_offset_sources',dict(verified=False,classification='CANDIDATE_ONLY',active_name_unknown=True,search_exit=search.returncode,stderr=search.stderr,search_file='offset_candidate_search.txt'))
    log_offsets={str(p):p.stat().st_size for p in Path('/home/ubuntu/.ros/log').glob('ros2_control_node_*.log')}
    save('driver_log_offsets',log_offsets)
    __import__("os").environ["ROS_LOCALHOST_ONLY"] = "1"; rclpy.init();node=rclpy.create_node('phase12_final_persistent_precheck')
    lock=threading.RLock();state=JointStateStartup(time.monotonic());rows=[];logs=[];latest=[None];frames={};messages={};subscriptions={}
    def joints(m):
        now=time.monotonic();source=m.header.stamp.sec+m.header.stamp.nanosec/1e9
        valid=len(m.name)==6 and len(set(m.name))==6 and set(m.name)=={f'joint_{i}' for i in range(1,7)} and len(m.position)==6 and len(m.velocity)==6 and all(math.isfinite(v) for v in list(m.position)+list(m.velocity))
        row=dict(receive=now,source=source,header_age=node.get_clock().now().nanoseconds/1e9-source,valid=valid,names=list(m.name),position=list(m.position),velocity=list(m.velocity))
        with lock:state.sample(row);rows.append(row)
    def camera(m):
        with lock:latest[0]=(m,time.monotonic())
    def tf(m):
        with lock:
            for t in m.transforms:frames[t.header.frame_id+'->'+t.child_frame_id]=message_to_ordereddict(t)
    def record(m,t):
        with lock:messages[t]=dict(receive=time.monotonic(),message=message_to_ordereddict(m))
    def log(m):
        if 'dsr' in m.name or 'controller' in m.name:
            with lock:logs.append(dict(receive=time.monotonic(),level=m.level,text=m.msg,name=m.name))
    node.create_subscription(JointState,'/dsr01/joint_states',joints,qos_profile_sensor_data)
    node.create_subscription(Image,'/zed/zed_node/rgb/color/rect/image',camera,qos_profile_sensor_data)
    node.create_subscription(TFMessage,'/tf',tf,qos_profile_sensor_data)
    node.create_subscription(TFMessage,'/tf_static',tf,QoSProfile(depth=100,reliability=ReliabilityPolicy.RELIABLE,durability=DurabilityPolicy.TRANSIENT_LOCAL))
    node.create_subscription(Log,'/rosout',log,1000)
    stop=threading.Event()
    def spin():
        while not stop.is_set():rclpy.spin_once(node,timeout_sec=.01)
    thread=threading.Thread(target=spin);thread.start()
    def status():
        with lock:return state.status(time.monotonic())
    def discover():
        topics=node.get_topic_names_and_types()
        for t,types in topics:
            if t not in subscriptions and t.startswith('/dsr01/') and types and (types[0].startswith('dsr_msgs2/msg/') or re.search('pose|tcp|tool|servo|authority|connection',t,re.I)):
                try:subscriptions[t]=node.create_subscription(get_message(types[0]),t,lambda m,t=t:record(m,t),qos_profile_sensor_data)
                except Exception as exc:messages[t]={'error':str(exc)}
        return dict(topics=topics,services=node.get_service_names_and_types())
    try:
        graph=discover()
        while True:
            s=status()
            if s['fault'] or s['phase']=='RUNTIME_READY':break
            time.sleep(.02)
        before=status();save('jointstate_runtime_before',before);graph=discover()
        cli=[]
        for cmd in (['ros2','topic','list','-t'],['ros2','service','list','-t']):
            try:
                p=subprocess.run(cmd,capture_output=True,text=True,timeout=12);cli.append(dict(command=cmd,exit=p.returncode,stdout=p.stdout,stderr=p.stderr))
            except subprocess.TimeoutExpired:cli.append(dict(command=cmd,timeout=True))
        graph['cli']=cli;save('ros_graph_snapshot',graph)
        # TCP/tool name getters intentionally never repeated; only3 allowed enum refreshes.
        values={};requests=0;inhibited=False
        for key,(type_name,endpoint) in REFRESH.items():
            result=dict(status='NOT_EXECUTED',endpoint=endpoint,provenance='UNKNOWN',before=status())
            offered=dict(node.get_service_names_and_types())
            if inhibited or status()['phase']!='RUNTIME_READY' or status()['fault']:
                result['reason']='runtime_gate_failed'
            elif endpoint not in offered:result['reason']='endpoint_not_discovered'
            else:
                client=node.create_client(getattr(services,type_name),endpoint)
                if not client.wait_for_service(timeout_sec=2):result['reason']='service_not_ready'
                elif status()['fault']:result['reason']='runtime_fault_before_call'
                else:
                    tick=time.monotonic();future=client.call_async(getattr(services,type_name).Request());requests+=1
                    result.update(request_wall=time.time(),request_monotonic=tick,request_count=1)
                    while not future.done() and time.monotonic()-tick<3 and not status()['fault']:time.sleep(.01)
                    if future.done():
                        try:
                            response=message_to_ordereddict(future.result())
                            result.update(status='RESPONSE',response=response,provenance='DRIVER_REPORTED')
                            if not response.get('success'):inhibited=True;result['provenance']='UNKNOWN'
                        except Exception as exc:result.update(status='ERROR',reason=str(exc));inhibited=True
                    else:result.update(status='TIMEOUT_OR_RUNTIME_FAULT');inhibited=True
                    result.update(latency_s=time.monotonic()-tick,after=status())
                node.destroy_client(client)
            values[key]=result
        save('mode_state',values)
        config=yaml.safe_load((ROOT/'configs/real_openvla.yaml').read_text());url=config['server_url']
        health=dict(status='FAIL')
        try:
            with urlopen(url+'/health',timeout=3) as f:h=json.load(f)
            validate_identity(h,'openvla');health.update(status='MODEL_HEALTH_PASS',health=h,sample_inference='NOT_EXECUTED')
        except Exception as exc:health['reason']=str(exc)
        camera_result=dict(status='FAIL')
        # Same persistent camera subscription; one current live image, no historical image.
        deadline=time.monotonic()+.5
        while time.monotonic()<deadline:
            with lock:c=latest[0]
            if c and time.monotonic()-c[1]<=.01:break
            time.sleep(.001)
        if c:
            image,received=c;camera_result.update(receive=received,age_s=time.monotonic()-received,encoding=image.encoding,resolution=[image.width,image.height])
            camera_result['status']='PASS' if camera_result['age_s']<.5 and image.encoding=='bgra8' and [image.width,image.height]==[1280,720] else 'FAIL'
            if health['status']=='MODEL_HEALTH_PASS' and camera_result['status']=='PASS':
                try:
                    rgb=camera_rgb_image(image);b=io.BytesIO();rgb.save(b,format='JPEG',quality=95);tick=time.monotonic()
                    req=Request(url+'/predict',data=json.dumps(dict(image_jpeg_base64=base64.b64encode(b.getvalue()).decode(),instruction=config['instruction'])).encode(),headers={'Content-Type':'application/json'})
                    with urlopen(req,timeout=3) as f:prediction=json.load(f)
                    vector=prediction.get('action');valid=isinstance(vector,list) and len(vector)==7 and all(isinstance(x,(int,float)) and math.isfinite(x) for x in vector) and prediction.get('fixture') is not True
                    health.update(sample_inference='PASS' if valid else 'FAIL',prediction=prediction,latency_s=time.monotonic()-tick)
                    camera_result.update(selected_frame_age_s=time.monotonic()-received,selected_frame_failure=time.monotonic()-received>=.5)
                    if not valid:health['status']='FAIL'
                    if camera_result['selected_frame_failure']:camera_result['status']='FAIL'
                except Exception as exc:health.update(status='FAIL',sample_inference='FAIL',reason=str(exc))
        save('camera_current',camera_result);save('model_health_current',health)
        end=time.monotonic()+10
        while time.monotonic()<end:status();time.sleep(.02)
        after=status();save('jointstate_runtime_after',after);save('jointstate_runtime_events',state.events);save('jointstate_samples',rows)
        file_delta=[]
        for path,offset in log_offsets.items():
            p=Path(path)
            if p.exists() and p.stat().st_size>offset:
                with p.open('rb') as f:f.seek(offset);data=f.read(200000).decode(errors='replace')
                file_delta.append(dict(path=path,offset=offset,delta=data))
        save('driver_file_log_delta',file_delta)
        with lock:raw=dict(messages=messages,tf=frames,logs=logs)
        save('hardware_raw_sources',raw)
        # Preserve raw topic candidates; a field existing in a schema is not a fresh value.
        save('tcp_contract',dict(classification='TCP_FLANGE_ONLY',active_tcp='UNKNOWN',active_tool='UNKNOWN',previous_name_responses_empty=True,tcp_tool_getter_requests=0,offset_verified=False,current_tcp_pose_verified=False,tf_candidates=frames))
        for name in ('connection_state','authority_state','servo_state','protective_stop_state','emergency_stop_state','gripper_state'):
            save(name,dict(value='UNKNOWN',provenance='UNKNOWN',reason='No validated fresh affirmative signal; raw candidates retained in hardware_raw_sources.json',gripper_output_disabled=True if name=='gripper_state' else None))
        save('manual_requirements',dict(operator='MANUAL_CONFIRMATION_REQUIRED',workspace='MANUAL_CONFIRMATION_REQUIRED',estop='MANUAL_CONFIRMATION_REQUIRED'))
        summary=dict(jointstate_runtime='PASS' if after['phase']=='RUNTIME_READY' and not after['fault'] else 'FAIL',jointstate_post='PASS' if after['phase']=='RUNTIME_READY' and not after['fault'] else 'FAIL',
            jointstate_before=before,jointstate_after=after,camera=camera_result,model_health=health['status'],mode_state=values,
            tcp_classification='TCP_FLANGE_ONLY',tcp_contract_verified=False,hardware_unknown=['connection','authority','servo','protective_stop','emergency_stop','physical_gripper_state'],
            getter_requests=requests,tcp_tool_getter_requests=0,gripper_command_disabled=True,minimum_motion_precheck='MINIMUM_MOTION_BLOCKED',physical_commands=0,next_allowed_stage='BLOCKED')
        save('summary',summary)
        (out/'FINAL_MINIMUM_MOTION_PRECHECK_REPORT.md').write_text('# Final integrated read-only precheck\n\n```json\n'+json.dumps(summary,indent=2)+'\n```\n\nNo physical motion, setters, gripper, Hold/Stop or Cartesian getter. TCP/tool name getters not repeated. One persistent JointState subscriber throughout; raw source candidates are not verified TCP. Runtime100 ms/camera0.5 s thresholds unchanged. Mode/state/system are timestamped DRFL getter snapshots, not continuous safety telemetry.\n')
        print(str(out),flush=True);print(json.dumps(summary,indent=2),flush=True)
    finally:
        stop.set();thread.join(timeout=3);node.destroy_node();rclpy.shutdown()

if __name__=='__main__':main()
