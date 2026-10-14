"""One explicitly approved arm integration step via existing Controller/sink."""
import argparse,base64,hashlib,io,json,math,threading,time
from pathlib import Path
from run_real_rollout import Controller,CanonicalAction,SafetyPipeline,RuntimeState,FsyncJsonlLogger,Authorization,RealDoosanCommandSink,RosServiceTransport
from concurrent_logger import ConcurrentLogger
from jointstate_runtime_startup import JointStateStartup
from live_observation import camera_rgb_image
from camera_inference_timing import inference_timing
from verify_live_model_server import validate_identity
from jointstate_flange_fk import A0509FlangeFK,reorder_joint_state
from action_rate_limiter import CanonicalActionRateLimiter,_scale_to
from urllib.request import Request,urlopen
TARGET=[-182.55277,-1.07182,-41.00585,1.25662,-113.50009,1.49975]


def require_operator_confirmation(approval,now):
    if not all(approval.get(k) is True for k in ('operator_present','workspace_clear','estop_accessible','robot_stationary','tcp_arm_only_acceptance','explicit_one_arm_command')):
        raise PermissionError('actual_operator_confirmation_required')
    if not 0<=now-approval['confirmed_wall_time']<=60:raise PermissionError('fresh_operator_approval_expired')


def run(output,approval):
    require_operator_confirmation(approval,time.time())
    import os,rclpy
    from sensor_msgs.msg import JointState,Image
    from rclpy.qos import qos_profile_sensor_data
    from dsr_msgs2.srv import GetRobotMode,GetRobotState,GetRobotSystem,GetCurrentPosx
    from rosidl_runtime_py.convert import message_to_ordereddict
    from scipy.spatial.transform import Rotation
    output.mkdir(parents=True,exist_ok=False)
    def save(name,value):(output/name).write_text(json.dumps(value,indent=2,allow_nan=False))
    save('operator_confirmation.json',approval)
    configs={str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in (Path(__file__).resolve().parent.parent/'01_configs').glob('*.yaml')}
    save('config_snapshot.json',configs)
    save('tcp_execution_acceptance.json',dict(formal_classification='TCP_ZERO_OFFSET_SUPPORTED_NOT_VERIFIED',tcp_contract_verified=False,arm_only_execution_accepted=True,source='EXPLICIT_USER_REQUEST',active_tcp_name_directly_observed=False,basis='ZERO_OFFSET_CONSISTENCY_NOT_FORMAL_VERIFICATION'))
    os.environ['ROS_LOCALHOST_ONLY']='1';rclpy.init();node=rclpy.create_node('approved_arm_only_once')
    lock=threading.RLock();halt=threading.Event();gate=JointStateStartup(time.monotonic());rows=[];camera=[None];controller=[None];abort_threads=[]
    def joints(msg):
        now=time.monotonic();stamp=msg.header.stamp.sec+msg.header.stamp.nanosec/1e9
        valid=len(msg.name)==6 and set(msg.name)=={f'joint_{i}' for i in range(1,7)} and len(msg.position)==len(msg.velocity)==6 and all(math.isfinite(v) for v in list(msg.position)+list(msg.velocity))
        row=dict(receive=now,source=stamp,header_age=node.get_clock().now().nanoseconds/1e9-stamp,names=list(msg.name),position=list(msg.position),velocity=list(msg.velocity),valid=valid)
        with lock:gate.sample(row);rows.append(row)
    def image(msg):
        with lock:camera[0]=(msg,time.monotonic())
    node.create_subscription(JointState,'/dsr01/joint_states',joints,qos_profile_sensor_data)
    node.create_subscription(Image,'/zed/zed_node/rgb/color/rect/image',image,qos_profile_sensor_data)
    def status():
        with lock:return gate.status(time.monotonic())
    def abort_fault(reason):
        c=controller[0]
        if c and not c.aborted:
            # Keep executor spinning while existing abort sink waits for ACK.
            thread=threading.Thread(target=lambda:c.abort(reason));abort_threads.append(thread);thread.start()
    def spin():
        try:
            while not halt.is_set():
                rclpy.spin_once(node,timeout_sec=.005)
                s=status()
                if s['fault']:abort_fault('jointstate_runtime_fault:'+s['fault'])
        except Exception as exc:
            with lock:gate.fail('executor_error:'+str(exc))
            abort_fault('executor_error')
    thread=threading.Thread(target=spin);thread.start()
    def check():
        s=status()
        if s['fault']:raise RuntimeError('jointstate_runtime_fault:'+s['fault'])
        return s
    def getter(name,kind,**fields):
        check();client=node.create_client(kind,name);sent=time.monotonic()
        try:
            if not client.wait_for_service(timeout_sec=2):raise TimeoutError('getter_endpoint:'+name)
            request=kind.Request()
            for key,value in fields.items():setattr(request,key,value)
            future=client.call_async(request);deadline=time.monotonic()+2
            while not future.done() and time.monotonic()<deadline:check();time.sleep(.002)
            if not future.done():raise TimeoutError('getter_response:'+name)
            response=message_to_ordereddict(future.result());check()
            if response.get('success') is not True:raise RuntimeError('getter_unsuccessful:'+name)
            return dict(response=response,latency_s=time.monotonic()-sent,observed_wall_time=time.time(),provenance='DRIVER_REPORTED')
        finally:node.destroy_client(client)
    fk=A0509FlangeFK('/home/ubuntu/robot_ws/src/doosan-robot2/dsr_description2/urdf/a0509.urdf')
    error=None;receipt=None;raw=None;before=None;after=None;record=None;hardware={};outcome='NOT_EXECUTED';model_requested=False
    try:
        while status()['phase']!='RUNTIME_READY':check();time.sleep(.005)
        for key,kind in [('mode',GetRobotMode),('state',GetRobotState),('system',GetRobotSystem)]:
            hardware[key]=getter('/dsr01/system/get_robot_'+key,kind)
            if hardware[key]['response']['robot_'+key]!=dict(mode=1,state=1,system=0)[key]:raise ValueError('hardware_not_REAL_AUTO_STANDBY')
        save('hardware_current.json',hardware)
        with lock:before=dict(rows[-1])
        positions=dict(zip(before['names'],before['position']));pose_error=max(abs(math.degrees(positions[f'joint_{i+1}'])-TARGET[i]) for i in range(6))
        if pose_error>.5 or any(v!=0 for v in before['velocity']):raise ValueError('start_pose_or_stationary_failed')
        save('jointstate_before.json',before)
        current=getter('/dsr01/aux_control/get_current_posx',GetCurrentPosx,ref=0)
        cp=current['response']['task_pos_info'][0]['data'][:6]
        if len(cp)!=6 or not all(math.isfinite(v) for v in cp):raise ValueError('current_pose_invalid')
        current.update(pose=cp,frame='BASE',units='mm/degrees',orientation='ZYZ',engineering_accepted_arm_only=True);save('current_pose.json',current)
        with urlopen('http://127.0.0.1:8766/health',timeout=2) as response:health=json.load(response)
        model=validate_identity(health,'openvla');save('model_health.json',health)
        deadline=time.monotonic()+2
        while time.monotonic()<deadline:
            check()
            with lock:selected=camera[0]
            if selected and 0<=time.monotonic()-selected[1]<=.01:break
            time.sleep(.001)
        else:raise TimeoutError('fresh_camera_timeout')
        msg,received=selected
        if msg.encoding!='bgra8' or [msg.width,msg.height]!=[1280,720]:raise ValueError('camera_contract')
        rgb=camera_rgb_image(msg);buffer=io.BytesIO();rgb.save(buffer,format='JPEG',quality=95);jpeg=buffer.getvalue()
        request=Request('http://127.0.0.1:8766/predict',data=json.dumps(dict(image_jpeg_base64=base64.b64encode(jpeg).decode(),instruction=model['instruction'])).encode(),headers={'Content-Type':'application/json'})
        ros_start=node.get_clock().now().nanoseconds/1e9;started=time.monotonic()
        if not 0<=started-received<.5:raise ValueError('camera_stale_at_inference_start')
        check()
        model_requested=True
        with urlopen(request,timeout=1.2) as response:prediction=json.load(response)
        finished=time.monotonic();ros_end=node.get_clock().now().nanoseconds/1e9;raw=prediction['action']
        action=CanonicalAction.from_vector(raw,timestamp_monotonic=started,sequence_id='arm_one',source_model='openvla')
        timing=inference_timing(received,started,finished,source_stamp=msg.header.stamp.sec+msg.header.stamp.nanosec/1e9,ros_start=ros_start,ros_end=ros_end)
        save('prediction.json',dict(raw_action=raw,raw_response=prediction,**timing,source_image_sha256=hashlib.sha256(bytes(msg.data)).hexdigest(),model_input_sha256=hashlib.sha256(jpeg).hexdigest(),physical_gripper_disabled=True));rgb.save(output/'camera_frame.png')
        check();now=time.monotonic();runtime=RuntimeState(now,tuple(v/1000 for v in cp[:3]),tuple(cp[3:]),'alignment')
        decision=SafetyPipeline('openvla',operator_confirmed_initial_open=False,arm_only=True).inspect(action,runtime)
        if not decision.accepted:raise ValueError('safety_rejection:'+str(decision.reason))
        limiter=CanonicalActionRateLimiter();limited=limiter.limit(raw);speed,_=_scale_to(tuple(v/limiter.dt for v in raw[:3]),limiter.tv)
        save('action_pipeline.json',dict(raw_action=raw,post_hard_gate_action=raw,post_velocity_translation_m=[v*limiter.dt for v in speed],post_acceleration_action=list(limited.limited_action),final_arm_candidate=list(limited.limited_action[:6]),actual_gripper_command=None))
        target=decision.candidate['target_pose_candidate_mm_zyz_deg'];save('target_pose.json',dict(current_pose=cp,target_pose=target,limited_arm_delta=list(limited.limited_action[:6]),raw_model_delta=raw[:6],ref=0,mode=0))
        print(json.dumps(dict(current_pose=cp,target_pose=target,gripper='DISABLED',maximum_arm_commands=1)),flush=True)
        require_operator_confirmation(approval,time.time())
        if any(hashlib.sha256(Path(p).read_bytes()).hexdigest()!=h for p,h in configs.items()):raise ValueError('config_changed')
        with lock:before=dict(rows[-1])
        if any(v!=0 for v in before['velocity']):raise ValueError('robot_not_stationary_before_dispatch')
        check()
        with FsyncJsonlLogger(output/'command_events.jsonl') as logger:
            log=ConcurrentLogger(logger)
            sink=RealDoosanCommandSink(lambda:RosServiceTransport(node),Authorization(True,True,True,'MOTION_ENABLED'))
            c=Controller('openvla','short_horizon',sink,log,dry_run=False,arm_only_integration=True)
            obs=dict(receive_monotonic=time.monotonic(),tcp_m_abc=[v/1000 for v in cp[:3]]+cp[3:],phase='alignment',robot_state_ok=True,camera_ok=True,joint_state_ok=True,tcp_ok=True,model_ok=True,communication_ok=True,live_execution_clock=True,tcp_source='CONTROLLER_REPORTED_ZERO_OFFSET_ENGINEERING_ACCEPTED',tcp_contract_verified=False,raw_model_output=prediction)
            save('physical_command_request.json',dict(pos=target,ref=0,mode=0,vel=[20,20],acc=[20,20],time=0,radius=0,blend_type=0,sync_type=0,physical_gripper_disabled=True))
            controller[0]=c
            record=c.step(raw,obs,inference_time=started,chunk_id='one',chunk_index=0)
            receipt=record.get('command_receipt');save('physical_command_ack.json',receipt or record)
            deadline=time.monotonic()+2
            while time.monotonic()<deadline:check();time.sleep(.005)
            post=getter('/dsr01/system/get_robot_state',GetRobotState)
            if post['response']['robot_state']!=1:raise RuntimeError('robot_not_STANDBY_after_motion')
            with lock:after=dict(rows[-1])
            fb=fk.compute(reorder_joint_state(before['names'],before['position']));fa=fk.compute(reorder_joint_state(after['names'],after['position']))
            observed=(__import__('numpy').array(fa['position_m'])-__import__('numpy').array(fb['position_m']))*1000
            angle=math.degrees((Rotation.from_matrix(fa['rotation_matrix'])*Rotation.from_matrix(fb['rotation_matrix']).inv()).magnitude())
            save('observed_motion.json',dict(requested_arm_delta_m_rad=limited.limited_action[:6],observed_fk_flange_delta_mm=observed.tolist(),observed_orientation_geodesic_deg=angle,fk_before=fb,fk_after=fa,observed_joint_delta_rad=[dict(zip(after['names'],after['position']))[name]-value for name,value in zip(before['names'],before['position'])],operator_physical_confirmation='PENDING',direction_scale_automatically_passed=False))
            save('jointstate_after.json',after)
            outcome='PHYSICAL_CONFIRMATION_PENDING' if receipt and receipt['state']=='completed' and not c.aborted else 'ARM_ONLY_SHORT_HORIZON_FAIL'
    except Exception as exc:
        error=str(exc)
        if model_requested:outcome='ARM_ONLY_SHORT_HORIZON_INVALID'
        if controller[0]:
            abort_fault(error)
            outcome='ARM_ONLY_SHORT_HORIZON_INVALID'
    finally:
        for worker in abort_threads:worker.join(4)
        c=controller[0];events=c.sink.events if c else []
        save('jointstate_during_test.json',dict(rows=rows,status=status(),events=gate.events))
        result=dict(evaluation_scope='SYSTEM_INTEGRATION_TEST',status=outcome,reason=error,task_success=None,prediction_count=int(model_requested),physical_arm_commands=sum(e['command_type']=='pose' and e['sent_at'] is not None for e in events),physical_gripper_commands=sum(e['command_type']=='gripper' and e['sent_at'] is not None for e in events),physical_hold_stop_commands=sum(e['command_type'] in ('hold','stop') and e['sent_at'] is not None for e in events),command_events=events,raw_action=raw,estop_used=None,unexpected_motion=None,operator_result_confirmation_required=outcome=='PHYSICAL_CONFIRMATION_PENDING',auto_retry=False,auto_second_step=False)
        save('trial_summary.json',result)
        halt.set();thread.join(2);node.destroy_node();rclpy.shutdown()
        print(json.dumps(result,indent=2))


if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('--output',type=Path,required=True);parser.add_argument('--approval',type=Path,required=True)
    args=parser.parse_args();run(args.output,json.loads(args.approval.read_text()))
