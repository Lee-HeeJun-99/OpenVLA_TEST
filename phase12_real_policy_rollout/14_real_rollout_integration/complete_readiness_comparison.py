"""Feature-flagged read-only A/B/C/D experiment. Never creates command clients."""
import base64,hashlib,io,json,math,os,statistics,subprocess,threading,time,sys
from pathlib import Path
from urllib.request import Request,urlopen
from jointstate_runtime_startup import JointStateStartup
from final_hardware_tcp_precheck import source_audit
from verify_live_model_server import validate_identity
from live_observation import camera_rgb_image
ROOT=Path(__file__).resolve().parent
BUNDLE=ROOT.parents[2]
FULL={'camera','tf','tf_static','rosout','dynamic','cli','services','hash','http'}

def cli_commands(flags):
    return [['ros2','topic','list','-t']]+([['ros2','service','list','-t']] if 'services' in flags else [])

def main():
    import rclpy,yaml
    from rclpy.qos import qos_profile_sensor_data,QoSProfile,ReliabilityPolicy,DurabilityPolicy
    from sensor_msgs.msg import JointState,Image
    from tf2_msgs.msg import TFMessage
    from rcl_interfaces.msg import Log
    from rosidl_runtime_py.utilities import get_message
    from rosidl_runtime_py.convert import message_to_ordereddict
    out=ROOT/'real_trials'/(time.strftime('%Y%m%d_%H%M%S')+'_complete_rollout_readiness_audit');out.mkdir()
    def save(name,value):(out/(name+'.json')).write_text(json.dumps(value,indent=2))
    def command(cmd,timeout=5,cwd=None):
        try:
            p=subprocess.run(cmd,capture_output=True,text=True,timeout=timeout,cwd=cwd)
            return dict(command=cmd,exit=p.returncode,stdout=p.stdout,stderr=p.stderr)
        except subprocess.TimeoutExpired:return dict(command=cmd,timeout=True)
    env=dict(time_wall=time.time(),ros={k:os.getenv(k,'UNSET') for k in ('ROS_DOMAIN_ID','RMW_IMPLEMENTATION','ROS_LOCALHOST_ONLY')},
        git=[command(['git','branch','--show-current'],cwd=ROOT),command(['git','rev-parse','HEAD'],cwd=ROOT),command(['git','status','--short'],cwd=ROOT)],
        processes=command(['ps','-eo','pid,ppid,args']))
    save('00_environment',env)
    rclpy.init();node=rclpy.create_node('phase12_feature_flagged_readiness_audit')
    lock=threading.RLock();rows=[];images=[None];frames={};messages={};logs=[];graph=[];flags=set();subscriptions={};current=[JointStateStartup(time.monotonic())];run_id=['INIT']
    def js(m):
        now=time.monotonic();source=m.header.stamp.sec+m.header.stamp.nanosec/1e9
        valid=len(m.name)==6 and len(set(m.name))==6 and set(m.name)=={f'joint_{i}' for i in range(1,7)} and len(m.position)==6 and len(m.velocity)==6 and all(math.isfinite(v) for v in list(m.position)+list(m.velocity))
        row=dict(receive=now,source=source,header_age=node.get_clock().now().nanoseconds/1e9-source,valid=valid,names=list(m.name),position=list(m.position),velocity=list(m.velocity),run=run_id[0])
        with lock:current[0].sample(row);rows.append(row)
    def camera(m):
        with lock:images[0]=(m,time.monotonic())
    def tf(m):
        with lock:
            for t in m.transforms:frames[t.header.frame_id+'->'+t.child_frame_id]=message_to_ordereddict(t)
    def rosout(m):
        if 'dsr' in m.name or 'controller' in m.name:
            with lock:logs.append(dict(receive=time.monotonic(),level=m.level,name=m.name,text=m.msg))
    def sample(m,topic):
        with lock:
            old=messages.get(topic,{})
            messages[topic]=dict(count=old.get('count',0)+1,first_receive=old.get('first_receive',time.monotonic()),receive=time.monotonic(),message=message_to_ordereddict(m))
    node.create_subscription(JointState,'/dsr01/joint_states',js,qos_profile_sensor_data)
    halt=threading.Event()
    def spin():
        while not halt.is_set():rclpy.spin_once(node,timeout_sec=.01)
    observer=threading.Thread(target=spin);observer.start()
    def status():
        with lock:return current[0].status(time.monotonic())
    def set_flags(wanted):
        nonlocal flags
        for key in list(subscriptions):
            feature=key.split(':')[0]
            if feature not in wanted:node.destroy_subscription(subscriptions.pop(key))
        for key in wanted-flags:
            if key=='camera':subscriptions[key]=node.create_subscription(Image,'/zed/zed_node/rgb/color/rect/image',camera,qos_profile_sensor_data)
            if key=='tf':subscriptions[key]=node.create_subscription(TFMessage,'/tf',tf,qos_profile_sensor_data)
            if key=='tf_static':subscriptions[key]=node.create_subscription(TFMessage,'/tf_static',tf,QoSProfile(depth=100,reliability=ReliabilityPolicy.RELIABLE,durability=DurabilityPolicy.TRANSIENT_LOCAL))
            if key=='rosout':subscriptions[key]=node.create_subscription(Log,'/rosout',rosout,1000)
        flags=set(wanted)
    def discover():
        topics=node.get_topic_names_and_types();services=node.get_service_names_and_types() if 'services' in flags else []
        info=dict(receive=time.monotonic(),topics=topics,services=services)
        for t,types in topics:
            key='dynamic:'+t
            if key not in subscriptions and t.startswith('/dsr01/') and types and types[0].startswith('dsr_msgs2/msg/'):
                subscriptions[key]=node.create_subscription(get_message(types[0]),t,lambda m,t=t:sample(m,t),qos_profile_sensor_data)
        graph.append(info)
    config=yaml.safe_load((ROOT/'configs/real_openvla.yaml').read_text());url=config['server_url'];health={};predictions=[];children=[];results=[];host=[]
    def healthcheck():
        nonlocal health
        with urlopen(url+'/health',timeout=3) as response:health=json.load(response)
        validate_identity(health,'openvla');save('18_model_health',dict(status='MODEL_HEALTH_PASS',health=health,time_wall=time.time()))
    def predict(tag):
        deadline=time.monotonic()+.5
        while time.monotonic()<deadline:
            with lock:c=images[0]
            if c and time.monotonic()-c[1]<=.01:break
            time.sleep(.001)
        if not c:return dict(run=tag,event='INPUT_FAILURE',reason='camera_unavailable',command_issued=False)
        m,received=c;start=time.monotonic()
        try:
            rgb=camera_rgb_image(m);b=io.BytesIO();rgb.save(b,format='JPEG',quality=95);jpeg=b.getvalue()
            req=Request(url+'/predict',data=json.dumps(dict(image_jpeg_base64=base64.b64encode(jpeg).decode(),instruction=config['instruction'])).encode(),headers={'Content-Type':'application/json'})
            tick=time.monotonic()
            with urlopen(req,timeout=2) as response:raw=json.load(response)
            end=time.monotonic();a=raw.get('action');valid=isinstance(a,list) and len(a)==7 and all(isinstance(x,(int,float)) and math.isfinite(x) for x in a) and raw.get('fixture') is not True
            result=dict(run=tag,event='PREDICTION',receive=end,frame_receive=received,camera_age=end-received,encoding=m.encoding,resolution=[m.width,m.height],
                action=a,valid=valid,translation_mm=math.sqrt(sum(x*x for x in a[:3]))*1000 if valid else None,rotation_deg=math.degrees(math.sqrt(sum(x*x for x in a[3:6]))) if valid else None,
                gripper=a[6] if valid else None,inference_latency=end-tick,jointstate=status(),tcp_source='FK_ESTIMATED_FLANGE',tcp_contract_verified=False,
                command_requested=False,command_issued=False,executed_action=None,robot_delivered_command=None,
                matched_pair_id=None,condition=None,observation_gap_score=None,action_gap_translation=None,action_gap_rotation=None,action_gap_gripper=None)
            if 'hash' in flags:result.update(source_hash=hashlib.sha256(bytes(m.data)).hexdigest(),model_input_hash=hashlib.sha256(jpeg).hexdigest())
        except Exception as exc:result=dict(run=tag,event='MODEL_FAILURE',reason=str(exc),jointstate=status(),command_issued=False,executed_action=None)
        predictions.append(result)
        with (out/'19_live_shadow.jsonl').open('a') as f:f.write(json.dumps(result)+'\n')
        return result
    def host_sample():
        # Low-rate host-only observation, not ROS graph polling.
        net=Path('/proc/net/dev').read_text();load=os.getloadavg()
        host.append(dict(receive=time.monotonic(),run=run_id[0],load=load,network=net))
    def run(name,wanted,duration=60):
        set_flags(wanted)
        with lock:run_id[0]=name;current[0]=JointStateStartup(time.monotonic());offset=len(rows)
        print('BEGIN '+name,flush=True)
        while True:
            s=status()
            if s['fault'] or s['phase']=='RUNTIME_READY':break
            time.sleep(.02)
        # No restart within an experimental window, even after a runtime fault.
        t0=current[0].ready_at
        if t0 is None:
            result=dict(run=name,status='DISCOVERY_NOT_STABILIZED',state=s,flags=sorted(wanted));results.append(result);save(name,result);print(json.dumps(result),flush=True);return result
        t1=t0+duration;next_graph=t0;next_cli=t0;next_host=t0
        while time.monotonic()<t1:
            now=time.monotonic();status()
            if now>=next_host:host_sample();next_host=now+1
            if 'dynamic' in flags and now>=next_graph:discover();next_graph=now+1
            elif 'services' in flags and now>=next_graph:graph.append(dict(receive=now,services=node.get_service_names_and_types()));next_graph=now+1
            if 'cli' in flags and now>=next_cli:
                graph.append(dict(receive=now,cli=[command(cmd) for cmd in cli_commands(flags)]));next_cli=now+5
            if 'http' in flags:predict(name)
            else:time.sleep(.01)
        with lock:data=[r for r in rows[offset:] if t0<=r['receive']<t1];warm=[r for r in rows[offset:] if r['receive']<t0];events=list(current[0].events)
        sg=[r['source_gap'] for r in data if r['source_gap'] is not None];rg=[r['receive_gap'] for r in data if r['receive_gap'] is not None]
        cov=data[-1]['receive']-data[0]['receive'] if len(data)>1 else 0
        runtime_events=[e for e in events if e.get('event')=='RUNTIME_EVENT' and e.get('receive_after',t0)>=t0]
        passed=cov>=duration-.1 and bool(data) and all(r['valid'] for r in data) and all(0<x<.1 for x in sg+rg) and not status()['fault']
        result=dict(run=name,status='PASS' if passed else 'FAIL',duration=duration,t0=t0,t1=t1,first_fresh_delay=current[0].first_fresh-current[0].started,
            warmup_event_count=sum(e.get('event')=='STARTUP_WARMUP_EVENT' for e in events),runtime_message_count=len(data),coverage=cov,rate=(len(data)-1)/cov if cov else 0,
            max_source_gap=max(sg,default=None),max_receive_gap=max(rg,default=None),max_header_age=max((r['header_age'] for r in data),default=None),
            runtime_event_count=len(runtime_events),runtime_events=runtime_events,invalid=sum(not r['valid'] for r in data),duplicate=sum(x==0 for x in sg),regression=sum(x<0 for x in sg),state=status(),flags=sorted(wanted),
            host_load_max=max((x['load'][0] for x in host if x['run']==name),default=None),process_cpu=command(['ps','-eo','pid,pcpu,pmem,nlwp,args']))
        (out/(name+'_samples.json')).write_text(json.dumps(rows[offset:],indent=2));save(name,result);results.append(result);print(json.dumps({k:v for k,v in result.items() if k not in ('process_cpu','runtime_events')}),flush=True)
        return result
    try:
        if '--followup-without-services' in sys.argv:
            set_flags({'camera'});healthcheck()
            result=run('corrected_without_services',FULL-{'services'},duration=60)
            save('corrected_service_isolation',dict(result=result,graph=graph,messages=messages,physical_commands=0,new_experimental_process=True))
            print('CORRECTED_FOLLOWUP '+str(out),flush=True)
            return
        a=run('01_joint_run_a',set())
        camera_log=(out/'camera_restore.log').open('w')
        children.append(subprocess.Popen(['ros2','launch','zed_wrapper','zed_camera.launch.py','camera_model:=zed2i'],stdout=camera_log,stderr=subprocess.STDOUT))
        time.sleep(8)
        b=run('02_joint_run_b',{'camera'})
        model_log=(out/'model_restore.log').open('w')
        children.append(subprocess.Popen(['./scripts/serve.sh','vanilla','8766','0','127.0.0.1'],cwd=BUNDLE,stdout=model_log,stderr=subprocess.STDOUT))
        deadline=time.monotonic()+120
        while True:
            try:healthcheck();break
            except Exception as exc:
                if time.monotonic()>deadline:save('18_model_health',dict(status='FAIL',reason=str(exc)));break
                time.sleep(1)
        if health:
            cold=predict('COLD_START_BEFORE_C_RUNTIME');save('cold_start_prediction',cold)
        c=run('03_joint_run_c',{'camera','http'})
        d=run('04_joint_run_d',FULL)
        if d['status']!='PASS':
            for feature in ('tf','tf_static','rosout','dynamic','cli','services','hash','http'):
                run('perturb_without_'+feature,FULL-{feature},duration=30)
        save('05_joint_observer_comparison',results)
        primary=[a['status'],b['status'],c['status'],d['status']]
        classification='JOINTSTATE_RUNTIME_STABLE' if primary==['PASS']*4 else ('JOINTSTATE_OBSERVER_SENSITIVE' if primary[:3]==['PASS']*3 and primary[3]!='PASS' else 'JOINTSTATE_STILL_INCONCLUSIVE')
        save('06_jointstate_final_classification',dict(classification=classification,association_not_causation=True))
        save('07_robotstate_topics',dict(graph=graph,messages=messages,tf=frames,logs=logs));save('host_stats',host)
        save('08_tcp_sources',source_audit());save('09_tcp_contract',dict(classification='TCP_FLANGE_ONLY',active_tcp='UNKNOWN',active_tool='UNKNOWN',offset_verified=False,getter_requests=0))
        save('10_cartesian_pose',dict(classification='FLANGE_ONLY',measured_tcp_verified=False))
        for index,key in [('11','connection'),('12','authority'),('13','servo'),('14','protective_stop'),('15','emergency_stop'),('16','gripper')]:
            save(index+'_'+key,dict(value='UNKNOWN',provenance='UNKNOWN',gripper_output_disabled=True,reason='No validated fresh source; schema/TF/event silence not proof'))
        runtime_predictions=[p for p in predictions if p.get('run')=='03_joint_run_c'];valid=[p for p in runtime_predictions if p.get('valid')]
        camera_fail=sum(p.get('camera_age',1)>=.5 for p in runtime_predictions)
        model_fail=sum(not p.get('valid',False) for p in runtime_predictions)
        anomalies=sum(p['translation_mm']>4 or p['rotation_deg']>4 for p in valid)
        shadow=dict(predictions=len(runtime_predictions),camera_failure=camera_fail,max_selected_age=max((p['camera_age'] for p in valid),default=None),
            model_failures=model_fail,action_anomalies=anomalies,translation_median=statistics.median([p['translation_mm'] for p in valid]) if valid else None,
            translation_max=max((p['translation_mm'] for p in valid),default=None),rotation_max=max((p['rotation_deg'] for p in valid),default=None),
            jointstate=c['status'],status='PASS' if c['status']=='PASS' and valid and camera_fail==model_fail==anomalies==0 else 'FAIL',physical_commands=0)
        save('17_camera_validation',shadow);save('20_live_shadow_summary',shadow)
        save('21_manual_requirements',dict(operator='REQUIRED',workspace='REQUIRED',estop='REQUIRED'))
        final=dict(verdict='MINIMUM_MOTION_BLOCKED_MULTIPLE',jointstate_classification=classification,run_results=primary,shadow=shadow,
            blockers=['TCP_OFFSET_AND_CURRENT_POSE_UNVERIFIED','HARDWARE_STATE_UNKNOWN','MANUAL_CONFIRMATION_REQUIRED']+(['JOINTSTATE_RUNTIME_FAIL'] if primary!=['PASS']*4 else []),
            physical_commands=0,getter_requests=0,next_allowed_stage='BLOCKED')
        save('22_minimum_motion_readiness',final)
        (out/'COMPLETE_ROLLOUT_READINESS_REPORT.md').write_text('# Complete read-only comparison\n\n```json\n'+json.dumps(final,indent=2)+'\n```\n\nOne persistent JointState subscriber across feature-flag runs. Each separate experimental condition has new FIRST_FRESH +10 s warm-up and fixed runtime; failed windows never restarted. Driver preserved. Camera/model restored after minimal run. Perturbation windows30 s are shorter than A-D60 s. No getter, motion, setter, gripper, Hold/Stop clients/publishers.\n')
        print('FINAL '+str(out)+' '+json.dumps(final),flush=True)
    finally:
        halt.set();observer.join(timeout=3);node.destroy_node();rclpy.shutdown()
        # Camera/model stay restored; do not restart or terminate driver.

if __name__=='__main__':main()
