"""Explicitly approved, bounded ServoL arm-only commissioning test. No policy IO."""
import argparse,json,math,os,threading,time
from pathlib import Path
import numpy as np
import yaml
from run_arm_only_once import JointStateStartup,TARGET,FsyncJsonlLogger
from camera_inference_timing import camera_stream_lost


def target_pose(current,workspace):
    p=np.asarray(current,dtype=float)
    if p.shape!=(6,) or not np.all(np.isfinite(p)):raise ValueError('invalid_current_pose')
    target=p.copy();target[0]+=.5
    for pose in (p,target):
        if any(not workspace[k][0]<=pose[i]/1000<=workspace[k][1] for i,k in enumerate(('x','y','z'))):
            raise ValueError('workspace_rejection')
    return target.tolist()


def ramp(current,target,dt):
    if not 0<dt<=.1:raise ValueError('servo_tick_gap')
    p=np.asarray(current,dtype=float).copy();t=np.asarray(target,dtype=float);d=t[:3]-p[:3]
    n=np.linalg.norm(d)
    if n:p[:3]+=d*min(1.,5.*dt/n)
    return p.tolist()


def run(output):
    print('실제 ServoL 최소 테스트: BASE X +0.5mm, 20Hz, 최대 0.5초 1회. Gripper/MoveLine 없음.')
    print('ServoL topic에는 명령 ACK가 없습니다. 종료 시 기존 MoveStop HOLD(3) ACK를 확인합니다.')
    print('현재 TCP는 formal VERIFIED가 아닙니다. 기존 zero-offset engineering basis를 사용합니다.')
    for label in ['Operator present','Workspace clear','E-stop immediately accessible','Robot stationary',
                  'Authority/servo 정상 및 protective/emergency stop clear 확인',
                  '다른 motion/servo 실행기 없음',
                  'ServoL BASE/mm/ZYZ 계약과 zero-offset engineering basis 승인',
                  '위 최소 물리 테스트 1회 승인']:
        if input(label+' [yes/NO]: ').strip().lower()!='yes':
            print('NOT_EXECUTED: 승인 없음');return
    approval_wall=time.time()
    output.mkdir(parents=True,exist_ok=False)
    def save(name,obj):(output/name).write_text(json.dumps(obj,indent=2,allow_nan=False))
    save('operator_confirmation.json',dict(confirmed_wall_time=approval_wall,all_explicit_confirmations=True))
    limits=yaml.safe_load((Path(__file__).parent.parent/'01_configs/safety_limits.yaml').read_text())
    save('contract.json',dict(scope='SERVO_MINIMUM_SYSTEM_INTEGRATION_TEST',base_x_delta_mm=.5,
        rate_hz=20,stream_duration_s=.5,command_time_s=.1,velocity=[10.,2.],acceleration=[20.,5.],
        target_rate_limit_mm_s=5.,gripper_commands=0,stop_mode=3,
        vendor_ack_available=False,tcp_formal_verified=False,production_limits=limits,
        source_example='/home/ubuntu/robot_ws/src/openvla_doosan_runtime/openvla_doosan_runtime/servol_stream_test_node.py'))
    import rclpy
    from rclpy.qos import qos_profile_sensor_data,QoSProfile,ReliabilityPolicy,HistoryPolicy
    from sensor_msgs.msg import JointState,Image
    from dsr_msgs2.msg import ServolStream
    from dsr_msgs2.srv import GetRobotSystem,GetRobotMode,GetRobotState,GetCurrentPosx,MoveStop
    from rosidl_runtime_py.convert import message_to_ordereddict
    os.environ['ROS_LOCALHOST_ONLY']='1';rclpy.init();node=rclpy.create_node('approved_servol_minimum_once')
    lock=threading.RLock();halt=threading.Event();gate=JointStateStartup(time.monotonic())
    joints=[];camera=[None];fault=[None];active=[False];stop_future=[None];stop_requested=[False]
    result=dict(status='NOT_EXECUTED',servo_publish_attempts=0,gripper_commands=0,move_line_commands=0,
                stop_commands=0,vendor_ack=None,unexpected_motion=None,estop_used=None,auto_retry=False)
    stop_client=node.create_client(MoveStop,'/dsr01/motion/move_stop')
    def stop_once():
        with lock:
            if stop_requested[0] or not active[0]:return
            stop_requested[0]=True
            if not stop_client.service_is_ready():result['stop_error']='stop_service_unavailable';return
            req=MoveStop.Request();req.stop_mode=3
            result['stop_requested_monotonic']=time.monotonic()
            try:stop_future[0]=stop_client.call_async(req);result['stop_commands']+=1
            except Exception as e:result['stop_error']=str(e)
    def fail(reason):
        with lock:
            if fault[0] is None:fault[0]=reason
        stop_once()
    def joint(m):
        now=time.monotonic();stamp=m.header.stamp.sec+m.header.stamp.nanosec/1e9
        valid=(len(m.name)==len(m.position)==len(m.velocity)==6 and set(m.name)=={f'joint_{i}' for i in range(1,7)} and all(math.isfinite(v) for v in list(m.position)+list(m.velocity)))
        row=dict(receive=now,source=stamp,header_age=node.get_clock().now().nanoseconds/1e9-stamp,valid=valid,names=list(m.name),position=list(m.position),velocity=list(m.velocity))
        with lock:gate.sample(row);joints.append(row)
    def image(m):
        with lock:camera[0]=(time.monotonic(),m.width,m.height,m.encoding)
    node.create_subscription(JointState,'/dsr01/joint_states',joint,qos_profile_sensor_data)
    node.create_subscription(Image,'/zed/zed_node/rgb/color/rect/image',image,qos_profile_sensor_data)
    def spin():
        try:
            while not halt.is_set():
                rclpy.spin_once(node,timeout_sec=.005)
                with lock:s=gate.status(time.monotonic());cam=camera[0];running=active[0]
                if s['fault']:fail('jointstate:'+s['fault'])
                if running and camera_stream_lost(None if cam is None else cam[0],time.monotonic()):fail('camera_stream_loss')
                if (output/'operator_abort.request').exists():fail('operator_abort')
        except Exception as e:fail('executor_error:'+str(e))
    spinner=threading.Thread(target=spin);spinner.start()
    def check():
        with lock:s=gate.status(time.monotonic())
        if s['fault']:fail('jointstate:'+s['fault'])
        if fault[0]:raise RuntimeError(fault[0])
        return s
    def getter(kind,path,**fields):
        c=node.create_client(kind,path)
        try:
            if not c.wait_for_service(timeout_sec=2):raise RuntimeError('getter_unavailable:'+path)
            req=kind.Request()
            for k,v in fields.items():setattr(req,k,v)
            f=c.call_async(req);end=time.monotonic()+2
            while not f.done() and time.monotonic()<end:check();time.sleep(.002)
            if not f.done():raise RuntimeError('getter_timeout:'+path)
            r=message_to_ordereddict(f.result())
            if r.get('success') is not True:raise RuntimeError('getter_failed:'+path)
            check();return r
        finally:node.destroy_client(c)
    try:
        while check()['phase']!='RUNTIME_READY':time.sleep(.005)
        hardware={}
        for name,kind,value in [('system',GetRobotSystem,0),('mode',GetRobotMode,1),('state',GetRobotState,1)]:
            r=getter(kind,'/dsr01/system/get_robot_'+name);hardware[name]=r
            if r['robot_'+name]!=value:raise RuntimeError('hardware_not_REAL_AUTO_STANDBY')
        with lock:before=dict(joints[-1]);cam=camera[0]
        actual=dict(zip(before['names'],before['position']))
        error=max(abs(math.degrees(actual[f'joint_{i+1}'])-q) for i,q in enumerate(TARGET))
        if error>.5 or any(v!=0 for v in before['velocity']):raise RuntimeError('start_pose_or_stationary_failed')
        if cam is None or time.monotonic()-cam[0]>=.5 or cam[1:]!=(1280,720,'bgra8'):raise RuntimeError('camera_preflight_failed')
        if not stop_client.wait_for_service(timeout_sec=2):raise RuntimeError('stop_service_unavailable')
        if node.count_publishers('/dsr01/servol_stream'):raise RuntimeError('another_servo_publisher_exists')
        pose=getter(GetCurrentPosx,'/dsr01/aux_control/get_current_posx',ref=0)['task_pos_info'][0]['data'][:6]
        target=target_pose(pose,limits['workspace_m']);save('request.json',dict(current_pose=pose,target_pose=target,hardware=hardware,start_pose_error_deg=error,jointstate=before))
        print('CURRENT:',pose,'TARGET:',target,'BASE X +0.5mm',flush=True)
        if not 0<=time.time()-approval_wall<=60:raise RuntimeError('operator_approval_expired')
        publisher=node.create_publisher(ServolStream,'/dsr01/servol_stream',QoSProfile(depth=1,reliability=ReliabilityPolicy.RELIABLE,history=HistoryPolicy.KEEP_LAST))
        end=time.monotonic()+2
        while publisher.get_subscription_count()<1 and time.monotonic()<end:check();time.sleep(.01)
        if publisher.get_subscription_count()!=1:raise RuntimeError('servo_subscription_count_not_one')
        with lock:cam=camera[0]
        if cam is None or time.monotonic()-cam[0]>=.5:raise RuntimeError('camera_preflight_failed')
        if not 0<=time.time()-approval_wall<=60:raise RuntimeError('operator_approval_expired')
        with FsyncJsonlLogger(output/'servo_requests.jsonl') as logger:
            start=time.monotonic();next_tick=start;previous=start-.05;command=list(pose)
            for sequence in range(10):
                while time.monotonic()<next_tick:check();time.sleep(.001)
                check();now=time.monotonic();command=ramp(command,target,now-previous);previous=now
                if now-start>=.5:break
                logger.append(dict(sequence=sequence,requested_pose=command,time=.1,vel=[10.,2.],acc=[20.,5.],command_ack=None))
                check()
                if time.monotonic()-now>=.1:raise RuntimeError('logger_or_timer_stall')
                with lock:
                    check()
                    if publisher.get_subscription_count()!=1:raise RuntimeError('servo_subscriber_lost')
                    msg=ServolStream();msg.pos=command;msg.vel=[10.,2.];msg.acc=[20.,5.];msg.time=.1
                    active[0]=True;result['servo_publish_attempts']+=1;publisher.publish(msg)
                next_tick=start+(sequence+1)*.05
            while time.monotonic()<start+.5:check();time.sleep(.001)
        stop_once()
        result['status']='SERVO_MINIMUM_OPERATOR_OBSERVATION_REQUIRED'
    except KeyboardInterrupt:fail('operator_abort');result.update(status='OPERATOR_ABORT',reason='operator_abort')
    except Exception as e:
        fail(str(e));result.update(status='INVALID' if active[0] else 'NOT_EXECUTED',reason=fault[0])
    finally:
        stop_once()
        end=time.monotonic()+3
        while stop_future[0] is not None and not stop_future[0].done() and time.monotonic()<end:time.sleep(.005)
        if active[0]:
            f=stop_future[0]
            if f is not None and f.done():
                try:result['stop_response']=message_to_ordereddict(f.result());result['stop_ack']=result['stop_response']['success'] is True
                except Exception as e:result['stop_error']=str(e);result['stop_ack']=False
            else:result['stop_ack']=False
            if not result.get('stop_ack'):result['status']='SERVO_MINIMUM_FAIL';print('Stop ACK 미확인: 실제 로봇을 확인하고 이상 시 E-stop을 사용하세요.',flush=True)
            time.sleep(.3)
            if not fault[0] and result.get('stop_ack'):
                try:
                    after=getter(GetCurrentPosx,'/dsr01/aux_control/get_current_posx',ref=0)['task_pos_info'][0]['data'][:6]
                    result['controller_pose_after']=after;result['observed_translation_mm']=[after[i]-pose[i] for i in range(3)]
                    result['final_robot_state']=getter(GetRobotState,'/dsr01/system/get_robot_state')
                except Exception as e:result['postcheck_error']=str(e);result['status']='SERVO_MINIMUM_FAIL'
        if fault[0]:
            result['first_fault']=fault[0]
            if result['status']=='SERVO_MINIMUM_OPERATOR_OBSERVATION_REQUIRED':
                result.update(status='INVALID',reason=fault[0])
        halt.set();spinner.join(3)
        save('jointstate.json',dict(samples=joints,status=gate.status(time.monotonic())))
        node.destroy_node();rclpy.shutdown();save('summary.json',result);print(json.dumps(result,indent=2),flush=True)


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--output',type=Path,default=Path(__file__).parent/'real_trials'/time.strftime('%Y%m%d_%H%M%S_servo_minimum'))
    run(p.parse_args().output)
