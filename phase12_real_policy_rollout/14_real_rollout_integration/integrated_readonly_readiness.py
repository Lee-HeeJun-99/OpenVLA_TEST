"""Integrated diagnosis only: no robot clients, publishers, setters or motion."""
import base64,csv,datetime,glob,hashlib,io,json,math,os,re,signal,sqlite3,subprocess,sys,threading,time
from pathlib import Path
from urllib.request import Request,urlopen
import rclpy
from rclpy.qos import QoSProfile,ReliabilityPolicy,DurabilityPolicy
from sensor_msgs.msg import JointState,Image
from rclpy.serialization import deserialize_message
from live_observation import camera_rgb_image
from verify_live_model_server import validate_identity
from model_health import validate_runtime_health
ROOT=Path(__file__).resolve().parent
sys.path[:0]=[str(ROOT.parent/'02_safety'),str(ROOT.parent/'03_shadow_mode')]
from canonical_action import CanonicalAction
from safety_pipeline import SafetyPipeline,RuntimeState

def main():
    import argparse
    parser=argparse.ArgumentParser();parser.add_argument('--joint-window',type=float,default=30);parser.add_argument('--skip-prediction',action='store_true');parser.add_argument('--suffix',default='integrated_rollout_readiness');args=parser.parse_args()
    out=ROOT/'real_trials'/datetime.datetime.now().strftime('%Y%m%d_%H%M%S_'+args.suffix);out.mkdir(exist_ok=False,parents=True)
    def save(name,value):
        with (out/name).open('x') as f:json.dump(value,f,indent=2)
    def run(args,timeout=8):
        begin=time.monotonic()
        try:
            p=subprocess.run(args,stdout=subprocess.PIPE,stderr=subprocess.STDOUT,text=True,timeout=timeout)
            return dict(command=args,output=p.stdout,returncode=p.returncode,duration=time.monotonic()-begin)
        except subprocess.TimeoutExpired as exc:return dict(command=args,timeout=True,output=(exc.stdout or b'').decode() if isinstance(exc.stdout,bytes) else exc.stdout)
    env={k:v for k,v in os.environ.items() if re.search('ROS|RMW|FAST|CYCLONE|DDS',k)}
    save('dds_environment.json',dict(environment=env,default_domain=0,network=[run(['ip','addr']),run(['ip','route'])],daemon=run(['ros2','daemon','status'])))
    save('environment.txt',dict(branch=run(['git','branch','--show-current']),head=run(['git','rev-parse','HEAD']),processes=run(['ps','-eo','pid,ppid,stat,pcpu,pmem,nlwp,args'])))
    graph={k:run(['ros2',k,'list']+(['-t'] if k in ('topic','service') else [])) for k in ('node','topic','service')};save('ros_graph.json',graph)
    from rclpy.utilities import get_rmw_implementation_identifier
    logoffset={Path(p):Path(p).stat().st_size for p in glob.glob('/home/ubuntu/.ros/log/ros2_control_node_*.log')}
    __import__("os").environ["ROS_LOCALHOST_ONLY"] = "1"; rclpy.init();node=rclpy.create_node('phase12_integrated_readonly_readiness');lock=threading.RLock();stop=threading.Event()
    rows={'best_effort':[],'reliable':[]};camera=[None];timeline=[];hosts=[]
    def joint(kind):
        def cb(m):
            now=time.monotonic();valid=len(m.name)==6 and set(m.name)=={f'joint_{i}' for i in range(1,7)} and len(m.position)==len(m.velocity)==6 and all(math.isfinite(v) for v in list(m.position)+list(m.velocity))
            source=m.header.stamp.sec+m.header.stamp.nanosec/1e9
            receive_ros=node.get_clock().now().nanoseconds/1e9
            with lock:rows[kind].append(dict(sequence=len(rows[kind]),receive=now,receive_wall=time.time(),receive_ros=receive_ros,source=source,names=list(m.name),position=list(m.position),velocity=list(m.velocity),valid=valid,header_fresh_same_ros_clock=0<=receive_ros-source<.1))
        return cb
    def cam(m):
        with lock:camera[0]=(m,time.monotonic())
    for kind,rel,dur in [('best_effort',ReliabilityPolicy.BEST_EFFORT,DurabilityPolicy.VOLATILE),('reliable',ReliabilityPolicy.RELIABLE,DurabilityPolicy.TRANSIENT_LOCAL)]:
        node.create_subscription(JointState,'/dsr01/joint_states',joint(kind),QoSProfile(depth=1000,reliability=rel,durability=dur))
    node.create_subscription(Image,'/zed/zed_node/rgb/color/rect/image',cam,QoSProfile(depth=1,reliability=ReliabilityPolicy.BEST_EFFORT))
    def spin():
        while not stop.is_set():rclpy.spin_once(node,timeout_sec=.02)
    worker=threading.Thread(target=spin);worker.start()
    def poll():
        while not stop.wait(.2):
            eps=[dict(node=e.node_name,namespace=e.node_namespace,gid=list(e.endpoint_gid),qos=str(e.qos_profile),type=e.topic_type) for e in node.get_publishers_info_by_topic('/dsr01/joint_states')]
            svcs=[(n,t) for n,t in node.get_service_names_and_types() if n.startswith('/dsr01/')]
            timeline.append(dict(monotonic=time.monotonic(),wall=time.time(),publishers=eps,services=svcs))
            hosts.append(dict(monotonic=time.monotonic(),wall=time.time(),load=str(os.getloadavg()),network=Path('/proc/net/dev').read_text(),memory=Path('/proc/meminfo').read_text(),cpu=Path('/proc/stat').read_text().splitlines()[0],driver_stat=Path('/proc/3017586/stat').read_text() if Path('/proc/3017586/stat').exists() else 'MISSING'))
    monitor=threading.Thread(target=poll);monitor.start()
    baglog=(out/'rosbag_stdout.txt').open('x');bag=subprocess.Popen(['ros2','bag','record','/dsr01/joint_states','-o',str(out/'jointstate_rosbag')],stdout=baglog,stderr=subprocess.STDOUT,start_new_session=True)
    started=time.monotonic();first=None;deadline=started+60
    while time.monotonic()<deadline:
        with lock:
            fresh={k:next((r for r in v if r['valid'] and r['header_fresh_same_ros_clock']),None) for k,v in rows.items()}
            if all(fresh.values()):
                first=max(r['receive'] for r in fresh.values());break
        time.sleep(.05)
    if first is not None:
        while time.monotonic()<first+args.joint_window:time.sleep(.05)
    measured_end=time.monotonic()
    def stats(data,begin=None,end=None):
        data=[d for d in data if (begin is None or d['receive']>=begin) and (end is None or d['receive']<=end)]
        sg=[b['source']-a['source'] for a,b in zip(data,data[1:])];rg=[b['receive']-a['receive'] for a,b in zip(data,data[1:])]
        duration=data[-1]['receive']-data[0]['receive'] if len(data)>1 else 0
        result=dict(count=len(data),coverage=duration,rate=(len(data)-1)/duration if duration else 0,max_source_gap=max(sg,default=None),max_receive_gap=max(rg,default=None),latest_age=measured_end-data[-1]['receive'] if data else None,invalid=sum(not d['valid'] for d in data),duplicate=sum(g==0 for g in sg),regression=sum(g<0 for g in sg),first_receive_delay=data[0]['receive']-started if data else None)
        result['pass']=duration>=args.joint_window-.1 and bool(sg) and all(0<g<.1 for g in sg) and all(0<g<.1 for g in rg) and result['invalid']==0 and result['latest_age']<.5
        return result
    with lock:measured={k:stats(v,first,measured_end) for k,v in rows.items()}
    gate=all(s['pass'] for s in measured.values());save('jointstate_clean_gate.json',dict(status='JOINTSTATE_CLEAN_GATE_PASS' if gate else 'JOINTSTATE_CLEAN_GATE_FAIL',subscribers=measured,discovery_wait_limit_s=60,first_both_connected=first,window_s=args.joint_window))
    # CLI cache comparison is obtained while the direct observer remains alive.
    save('cli_topic_info.json',run(['ros2','topic','info','-v','/dsr01/joint_states']))
    service_names={n for sample in timeline for n,_ in sample['services']}
    save('controller_manager_discovery.json',dict(services={n:('AVAILABLE_IN_DIRECT_GRAPH' if n in service_names else 'CONTROLLER_MANAGER_SERVICE_NOT_DISCOVERED') for n in ['/dsr01/controller_manager/list_controllers','/dsr01/controller_manager/list_hardware_interfaces']},service_gid='UNAVAILABLE_IN_RCLPY_HUMBLE_GRAPH_API',calls=0))
    # Model/camera-only diagnostics proceed even when joints failed. This never
    # marks integrated readiness PASS or supplies synthetic TCP/hardware state.
    health=None;config=None;pred=[];error=None;health_status='FAIL';pipeline=SafetyPipeline('openvla',operator_confirmed_initial_open=False)
    try:
        if args.skip_prediction:raise RuntimeError('SKIPPED_MODEL_OFF_FOR_JOINTSTATE_ONLY')
        with urlopen('http://127.0.0.1:8766/health',timeout=3) as f:health=json.load(f)
        config=validate_identity(health,'openvla');health_status='IDENTITY_PASS'
    except Exception as exc:error='health:'+str(exc)
    shadow_start=time.monotonic()
    with (out/'openvla_shadow.jsonl').open('x') as logfile:
        while config and time.monotonic()-shadow_start<30:
            try:
                health_begin=time.monotonic()
                with urlopen(config['server_url']+'/health',timeout=3) as f:current=json.load(f)
                validate_runtime_health(current,health)
                health_latency=time.monotonic()-health_begin
                snapshot_wait_begin=time.monotonic()
                waituntil=time.monotonic()+.5
                while time.monotonic()<waituntil:
                    with lock:selected=camera[0]
                    if selected and time.monotonic()-selected[1]<=.01:break
                    time.sleep(.001)
                if not selected or time.monotonic()-selected[1]>.01:raise TimeoutError('camera_snapshot_not_fresh')
                image,received=selected;snapshot=time.monotonic();rgb_begin=snapshot;rgb=camera_rgb_image(image);rgb_end=time.monotonic();buf=io.BytesIO();rgb.save(buf,format='JPEG',quality=95);jpeg=buf.getvalue();encode_end=time.monotonic();requesttime=time.monotonic()
                req=Request(config['server_url']+'/predict',data=json.dumps(dict(image_jpeg_base64=base64.b64encode(jpeg).decode(),instruction=config['instruction'])).encode(),headers={'Content-Type':'application/json'})
                http_begin=time.monotonic()
                with urlopen(req,timeout=1.2) as f:response_bytes=f.read()
                http_end=time.monotonic();raw=json.loads(response_bytes);decode_end=time.monotonic()
                if raw.get('fixture'):raise ValueError('fixture_forbidden')
                now=time.monotonic();action=CanonicalAction.from_vector(raw['action'],timestamp_monotonic=requesttime,sequence_id=str(len(pred)),source_model='openvla');health_status='MODEL_HEALTH_PASS'
                with lock:recent=rows['best_effort'][-1] if rows['best_effort'] else None
                joint_ok=bool(recent and now-recent['receive']<.5 and gate)
                # No flange/current TCP supplied as a verified TCP; origin is
                # explicitly invalid safety context, not a claimed measurement.
                decision=pipeline.inspect(action,RuntimeState(now,(0.,0.,0.),(0.,0.,0.),'unknown',camera_ok=now-received<.5,joint_state_ok=joint_ok,tcp_ok=False)).as_dict()
                safety_end=time.monotonic()
                record=dict(timestamp=time.time(),frame_id=len(pred),instruction=config['instruction'],raw_model_output=raw,canonical_action=action.as_dict(),translation_norm_m=math.dist(action.translation_m,(0,0,0)),rotation_norm_deg=math.degrees(math.dist(action.rotation_rotvec_rad,(0,0,0))),gripper_closedness=action.gripper_closedness,camera_age_s=now-received,snapshot_age_s=snapshot-received,inference_latency_s=now-requesttime,end_to_end_latency_s=now-received,safety=decision,jointstate_fresh=joint_ok,tcp_source='UNAVAILABLE',tcp_contract_verified=False,current_tcp=None,model_input_sha256=hashlib.sha256(jpeg).hexdigest(),source_image_sha256=hashlib.sha256(bytes(image.data)).hexdigest(),rgb_sha256=hashlib.sha256(rgb.tobytes()).hexdigest(),encoding=image.encoding,resolution=[image.width,image.height],command_issued=False,command_requested=False,executed_action=None,robot_delivered_command=None,delivered_action=None,diagnostic_scope='VISION_ONLY_NO_MOTION_READINESS')
                record['latency_breakdown']=dict(health_s=health_latency,snapshot_wait_s=snapshot-snapshot_wait_begin,frame_age_at_snapshot_s=snapshot-received,bgra_rgb_s=rgb_end-rgb_begin,jpeg_encode_s=encode_end-rgb_end,http_response_s=http_end-http_begin,response_decode_s=decode_end-http_end,safety_and_action_evaluation_s=safety_end-now,server_reported_inference_s=raw.get('inference_seconds'),hash_and_record_s=time.monotonic()-safety_end,server_gpu_only_s=None)
                record['latency_caveat']='Server inference_seconds times synchronized model call after processor/device transfer and lock acquisition; HTTP residual is not network-only. Research hashes are after response.'
                logfile.write(json.dumps(record)+'\n');logfile.flush();os.fsync(logfile.fileno());pred.append(record)
            except Exception as exc:error=str(exc);break
        logfile.write(json.dumps(dict(event='TERMINAL',error=error,command_issued=False))+'\n');logfile.flush();os.fsync(logfile.fileno())
    duration=time.monotonic()-shadow_start
    camera_failures=sum(r['camera_age_s']>=.5 for r in pred);anomalies=sum(r['translation_norm_m']>.004 or r['rotation_norm_deg']>4 for r in pred)
    save('model_health.json',dict(status=health_status,health=health,error=error))
    save('camera_validation.json',dict(status='PASS' if duration>=30 and pred and not error and camera_failures==0 else 'FAIL',duration_s=duration,count=len(pred),camera_failure=camera_failures,max_selected_frame_age_s=max((r['camera_age_s'] for r in pred),default=None),full_hardware_watchdog='NOT_VERIFIED',scope='CAMERA_MODEL_ONLY_DIAGNOSTIC'))
    save('openvla_shadow_summary.json',dict(status='INTEGRATED_SHADOW_NOT_READY' if not gate else 'TCP_UNVERIFIED_SHADOW_INCONCLUSIVE',predictions=len(pred),duration_s=duration,anomalies=anomalies,translation_max_m=max((r['translation_norm_m'] for r in pred),default=None),rotation_max_deg=max((r['rotation_norm_deg'] for r in pred),default=None),close_candidates=sum(r['gripper_closedness']>=.7 for r in pred),physical_commands=0,error=error))
    stop.set();worker.join(timeout=3);monitor.join(timeout=3);node.destroy_node();rclpy.shutdown()
    if bag.poll() is None:
        os.killpg(bag.pid,signal.SIGINT)
        try:bag.wait(timeout=8)
        except subprocess.TimeoutExpired:os.killpg(bag.pid,signal.SIGTERM);bag.wait(timeout=3)
    baglog.close()
    for kind,data in rows.items():
        with (out/('jointstate_'+kind+'.csv')).open('x',newline='') as f:
            keys=list(data[0]) if data else ['sequence','receive','receive_wall','source','names','position','velocity','valid'];w=csv.DictWriter(f,fieldnames=keys);w.writeheader();w.writerows(data)
    with (out/'host_stats.csv').open('x',newline='') as f:
        w=csv.DictWriter(f,fieldnames=list(hosts[0]) if hosts else ['wall']);w.writeheader();w.writerows(hosts)
    save('dds_discovery_timeline.json',dict(rmw=get_rmw_implementation_identifier(),poll_interval_s=.2,samples=timeline))
    newlogs=[]
    for p,offset in logoffset.items():
        with p.open(errors='replace') as f:f.seek(offset);text=f.read()
        if text:newlogs.append(str(p)+'\n'+text)
    (out/'driver_correlation.log').write_text('\n'.join(newlogs) or 'No new log bytes.\n')
    for name,value in {
        'tcp_getter_audit.json':dict(requests=0,classification='READ_ONLY_BUT_LIVE_STABILITY_UNVERIFIED',reason='Controller connection affirmative source unknown; all invocation prerequisites not satisfied.'),
        'active_tcp_tool.json':dict(active_tcp='UNKNOWN',active_tool='UNKNOWN'),
        'tcp_config_binding.json':dict(status='NOT_VERIFIED',historical_values_not_reused=True),
        'cartesian_pose_sources.json':dict(current_tcp=None,fk_flange_capability=True,measured_tcp=None),
        'tcp_contract.json':dict(classification='TCP_FLANGE_ONLY',verified=False),
        'robot_state.json':dict(connected='UNKNOWN',mode='UNKNOWN',authority='UNKNOWN',servo='UNKNOWN',protective_stop='UNKNOWN',emergency_stop='UNKNOWN',provenance='UNKNOWN'),
        'manual_confirmation_requirements.json':dict(operator='MANUAL_CONFIRMATION_REQUIRED',estop_access='MANUAL_CONFIRMATION_REQUIRED',workspace='MANUAL_CONFIRMATION_REQUIRED'),
        'minimum_motion_result.json':dict(authorized=False,executed=False,physical_pose_commands=0,requested_delta=None,protocol_delta_m=[.0005,0,0],observed_delta=None,result='BLOCKED',next_allowed_stage='BLOCKED')}.items():save(name,value)
    (out/'minimum_motion.jsonl').write_text(json.dumps(dict(event='NOT_EXECUTED',command_issued=False,executed_action=None,robot_delivered_command=None))+'\n')
    print(str(out),flush=True);print(json.dumps(dict(joint_gate=gate,subscribers=measured,predictions=len(pred),camera_failure=camera_failures,model_health=health_status,error=error,physical_commands=0),indent=2),flush=True)
if __name__=='__main__':main()
