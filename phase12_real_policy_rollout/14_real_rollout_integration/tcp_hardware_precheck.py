"""Controlled getter-only audit with one persistent JointState subscriber.

No motion client, publisher, setter, or unstable Cartesian getter is created.
"""
import json,math,time,threading,subprocess
from pathlib import Path
from jointstate_runtime_startup import JointStateStartup

SERVICES={
    'tcp_getter_result':('/dsr01/tcp/get_current_tcp','GetCurrentTcp'),
    'tool_getter_result':('/dsr01/tool/get_current_tool','GetCurrentTool'),
    'robot_mode_result':('/dsr01/system/get_robot_mode','GetRobotMode'),
    'robot_state_result':('/dsr01/system/get_robot_state','GetRobotState'),
    'robot_system_result':('/dsr01/system/get_robot_system','GetRobotSystem'),
}

def main():
    import rclpy
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import JointState,Image
    from rcl_interfaces.msg import Log
    import dsr_msgs2.srv as srv
    from rosidl_runtime_py.convert import message_to_ordereddict
    root=Path(__file__).resolve().parent
    output=root/'real_trials'/(time.strftime('%Y%m%d_%H%M%S')+'_tcp_hardware_precheck')
    output.mkdir(parents=True,exist_ok=False)
    def save(name,data):(output/(name+'.json')).write_text(json.dumps(data,indent=2))
    __import__("os").environ["ROS_LOCALHOST_ONLY"] = "1"; rclpy.init();node=rclpy.create_node('phase12_persistent_tcp_hardware_precheck')
    lock=threading.RLock();state=JointStateStartup(time.monotonic());rows=[];logs=[];cameras=[]
    log_files={p:str(p.stat().st_size) for p in Path('/home/ubuntu/.ros/log').glob('**/*.log')}
    def joint(m):
        now=time.monotonic();source=m.header.stamp.sec+m.header.stamp.nanosec/1e9
        valid=len(m.name)==6 and len(set(m.name))==6 and set(m.name)=={f'joint_{i}' for i in range(1,7)} and len(m.position)==6 and len(m.velocity)==6 and all(math.isfinite(x) for x in list(m.position)+list(m.velocity))
        row=dict(receive=now,source=source,header_age=node.get_clock().now().nanoseconds/1e9-source,valid=valid,names=list(m.name),position=list(m.position),velocity=list(m.velocity))
        with lock:state.sample(row);rows.append(row)
    def log(m):
        if 'dsr' in m.name or 'controller' in m.name:
            with lock:logs.append(dict(receive=time.monotonic(),name=m.name,level=m.level,text=m.msg))
    def camera(m):
        with lock:cameras.append(dict(receive=time.monotonic(),encoding=m.encoding,width=m.width,height=m.height))
    node.create_subscription(JointState,'/dsr01/joint_states',joint,qos_profile_sensor_data)
    node.create_subscription(Log,'/rosout',log,1000)
    node.create_subscription(Image,'/zed/zed_node/rgb/color/rect/image',camera,qos_profile_sensor_data)
    halt=threading.Event()
    def spin():
        while not halt.is_set():rclpy.spin_once(node,timeout_sec=.01)
    thread=threading.Thread(target=spin);thread.start()
    def status():
        with lock:return state.status(time.monotonic())
    def bad_logs():
        with lock:return [r for r in logs if r['level']>=40 or any(k in r['text'].lower() for k in ('disconnect','movej_cb','movel_cb','set_robot_mode_cb','set_current_tcp_cb','set_current_tool_cb'))]
    try:
        while True:
            s=status()
            if s['fault'] or s['phase']=='RUNTIME_READY':break
            time.sleep(.02)
        save('jointstate_runtime_before_getters',s)
        driver=subprocess.run(['pgrep','-a','-f','^/opt/ros/humble/lib/controller_manager/ros2_control_node'],capture_output=True,text=True).stdout.strip()
        offered=dict(node.get_service_names_and_types())
        pre=dict(jointstate=s,driver_process=driver,services=offered,timestamp_wall=time.time(),
            rosout_count=len(logs),bad_logs=bad_logs(),driver_log_offsets={str(k):v for k,v in log_files.items()},
            motion_authorized=False)
        save('tcp_tool_getter_precheck',pre)
        results={};inhibit=False
        for key,(endpoint,type_name) in SERVICES.items():
            before=status();client=None
            result=dict(endpoint=endpoint,request_count=0,status='NOT_EXECUTED',before=before)
            if inhibit or before['phase']!='RUNTIME_READY' or before['fault'] or not driver or bad_logs():
                result['reason']='runtime_or_driver_log_gate_failed';inhibit=True
            elif endpoint not in offered:
                result['reason']='service_not_discovered'
            else:
                client=node.create_client(getattr(srv,type_name),endpoint)
                # Discovery only; no polling of getters and no retries.
                if not client.wait_for_service(timeout_sec=2):result['reason']='service_not_ready'
                elif status()['fault'] or bad_logs():result['reason']='gate_changed';inhibit=True
                else:
                    started=time.monotonic();log_index=len(logs)
                    result.update(request_count=1,request_wall=time.time(),request_monotonic=started,rosout_offset=log_index)
                    future=client.call_async(getattr(srv,type_name).Request())
                    while not future.done() and time.monotonic()-started<3 and not status()['fault'] and not bad_logs():time.sleep(.01)
                    result['latency_s']=time.monotonic()-started
                    if future.done():
                        try:
                            response=message_to_ordereddict(future.result());result.update(status='RESPONSE',response=response)
                            if not response.get('success',False):inhibit=True
                        except Exception as exc:result.update(status='ERROR',reason=str(exc));inhibit=True
                    else:result.update(status='TIMEOUT_OR_RUNTIME_FAULT',reason='Client3s_or_runtime_fault; no retry');inhibit=True
                    # Retain identical subscriber for post-call observation.
                    until=time.monotonic()+1
                    while time.monotonic()<until:status();time.sleep(.01)
                    result.update(after=status(),driver_log_delta=logs[log_index:])
                    if status()['fault'] or bad_logs():inhibit=True
                node.destroy_client(client)
            results[key]=result;save(key,result)
        # Fixed10 s post-getter window. Never reset or hide a fault.
        post_start=time.monotonic()
        while time.monotonic()-post_start<10:status();time.sleep(.02)
        after=status();post=[r for r in rows if r['receive']>=post_start]
        save('jointstate_runtime_after_getters',dict(**after,post_observation_s=time.monotonic()-post_start,sample_count=len(post)))
        tcp=results['tcp_getter_result'].get('response',{}).get('info')
        tool=results['tool_getter_result'].get('response',{}).get('info')
        # Exact name search is evidence collection, not proof of current controller binding.
        matches={}
        for name in {tcp,tool}-{None,''}:
            search=subprocess.run(['rg','-n','-F','--max-count','3','--glob','*.yaml','--glob','*.json','--glob','*.py','--glob','*.cpp',name,
                '/home/ubuntu/robot_ws/src/doosan-robot2','/home/ubuntu/robot_ws/src/openvla_doosan_runtime',str(root.parent/'01_configs')],capture_output=True,text=True)
            matches[name]=search.stdout[:20000]
        save('tcp_config_binding',dict(active_tcp_name=tcp,active_tool_name=tool,candidate_matches=matches,verified=False,reason='Search hits alone are not live controller binding'))
        save('tcp_contract',dict(classification='TCP_NAME_ONLY' if tcp else 'TCP_UNKNOWN',active_tcp_name=tcp or 'UNKNOWN',active_tool_name=tool or 'UNKNOWN',offset_verified=False,source='FK_ESTIMATED_FLANGE_ONLY',tcp_contract_verified=False))
        save('cartesian_pose_result',dict(status='NOT_VERIFIED',get_current_posx='NOT_CALLED_KNOWN_FEEDBACK_LOSS_RISK',get_current_pose='NOT_CALLED_VENDOR_STABILITY_UNVERIFIED_NULL_POINTER_DEREFERENCE_WITHOUT_CHECK',fk_flange_not_actual_tcp=True))
        hardware=dict(controller_connection='UNKNOWN',robot_mode=results['robot_mode_result'].get('response',{}).get('robot_mode','UNKNOWN'),robot_state=results['robot_state_result'].get('response',{}).get('robot_state','UNKNOWN'),authority='UNKNOWN',servo='UNKNOWN',protective_stop='UNKNOWN',emergency_stop='UNKNOWN',provenance='CONTROLLER_REPORTED_FOR_SUCCESSFUL_RESPONSES_ONLY')
        save('hardware_state_summary',hardware)
        save('manual_requirements',dict(operator_present='MANUAL_CONFIRMATION_REQUIRED',workspace_clear='MANUAL_CONFIRMATION_REQUIRED',estop_accessible='MANUAL_CONFIRMATION_REQUIRED'))
        save('jointstate_runtime_events',state.events);save('jointstate_all_samples',rows);save('driver_log_delta',logs)
        summary=dict(jointstate_runtime='PASS' if after['phase']=='RUNTIME_READY' and not after['fault'] else 'FAIL',before=s,after=after,
            getter_requests=sum(r['request_count'] for r in results.values()),active_tcp=tcp or 'UNKNOWN',active_tool=tool or 'UNKNOWN',tcp_offset='NOT_VERIFIED',current_tcp_pose='NOT_VERIFIED',hardware=hardware,
            camera_passive_samples=len(cameras),previous_shadow='HISTORICAL_PASS_NOT_CURRENT_MOTION_GATE',
            gripper_command_disabled=True,gripper_pulse_disabled=True,model_gripper_ignored=True,
            minimum_motion_precheck='MINIMUM_MOTION_BLOCKED',physical_commands=0,next_allowed_stage='BLOCKED')
        save('summary',summary)
        (output/'MINIMUM_MOTION_PRECHECK_REPORT.md').write_text('# Read-only minimum-motion precheck\n\n```json\n'+json.dumps(summary,indent=2)+'\n```\n\nSame JointState subscriber retained through startup, all conditional getter calls and10 s post-call observation. No motion/setter/Hold/Stop client created. Unknown safety states are not PASS. Search results are candidates, not current TCP binding.\n')
        print(str(output),flush=True);print(json.dumps(summary,indent=2),flush=True)
    finally:
        halt.set();thread.join(timeout=3);node.destroy_node();rclpy.shutdown()

if __name__=='__main__':main()
