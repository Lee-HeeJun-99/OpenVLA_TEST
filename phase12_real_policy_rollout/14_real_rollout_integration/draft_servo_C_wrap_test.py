"""DRAFT physical +/-3deg C-wrap test. CREATED ONLY, NOT PHYSICALLY VALIDATED.

Default invocation does not import ROS or construct any transport. Execution
requires an explicit flag, reviewed site contract and fresh operator prompts.
No model, gripper, home, recovery, retry or automatic reverse motion.
"""
import argparse
import json
import math
import os
from pathlib import Path
import signal
import threading
import time
import numpy as np
import yaml
from scipy.spatial.transform import Rotation
from servo_experimental.bounded_segment import normalized_zyz
from servo_experimental.reference_replay import pose_distance,rotation

ROOT=Path(__file__).resolve().parent
DURATION_S=8.
AMPLITUDE_DEG=3.
RATE_HZ=20.


def validate_site_contract(contract):
    required=('reviewed_for_this_test','safe_pose_verified','controller_joint_limits_verified',
              'stop_path_verified','tcp_offset_verified','servo_transport_profile_verified')
    if not all(contract.get(k) is True for k in required):
        raise PermissionError('unverified_site_or_stop_contract')
    for name in ('safe_joint_deg','joint_lower_deg','joint_upper_deg','tcp_offset_mm_zyz_deg'):
        a=np.asarray(contract[name],dtype=float)
        if a.shape!=(6,) or not np.all(np.isfinite(a)):raise ValueError('invalid_'+name)
    lo=np.array(contract['joint_lower_deg']);hi=np.array(contract['joint_upper_deg'])
    margin=float(contract['joint_margin_deg'])
    if not math.isfinite(margin) or margin<=0 or np.any(lo+margin>=hi-margin):
        raise ValueError('invalid_joint_margin')
    q=np.array(contract['safe_joint_deg'])
    if np.any(q<=lo+margin) or np.any(q>=hi-margin):raise ValueError('safe_pose_near_joint_limit')
    for name in ('following_translation_mm','following_rotation_deg','minimum_abs_sin_B'):
        value=float(contract[name])
        if not math.isfinite(value) or value<=0:raise ValueError('invalid_'+name)
    if contract['minimum_abs_sin_B']>=1:raise ValueError('invalid_singularity_contract')
    if not contract.get('controller_version') or not contract.get('evidence_source'):
        raise ValueError('missing_contract_provenance')


def prepare_geometry(initial_pose,direction,contract):
    validate_site_contract(contract)
    p=np.asarray(initial_pose,dtype=float)
    if p.shape!=(6,) or not np.all(np.isfinite(p)):raise ValueError('invalid_initial_pose')
    if direction not in (-1,1):raise ValueError('direction_must_be_plus_or_minus_one')
    if abs(math.sin(math.radians(p[4])))<contract['minimum_abs_sin_B']:
        raise ValueError('initial_orientation_too_close_to_Euler_singularity')
    c=float((p[5]+180)%360-180)
    # Operator supplies an already safe near-boundary pose; never jog to it.
    if (direction==1 and not 177<c<180) or (direction==-1 and not -180<c<-177):
        raise ValueError('safe_initial_C_does_not_cross_selected_boundary')
    p=p.copy();p[5]=c
    return p


def candidate_pose(initial,t,direction,representation,previous_abc):
    u=float(np.clip(t/DURATION_S,0,1))
    # Analytic minimum-jerk time law, not velocity integration. Peak angular
    # speed 0.704deg/s, peak acceleration ~0.271deg/s² at duration8s.
    phase=10*u**3-15*u**4+6*u**5
    derivative=(30*u**2-60*u**3+30*u**4)/DURATION_S if 0<u<1 else 0.
    abc=np.array(initial[3:],dtype=float);abc[2]+=direction*AMPLITUDE_DEG*phase
    r=rotation(abc)
    if representation=='normalized':abc=normalized_zyz(r,previous_abc)
    elif representation!='unwrapped':raise ValueError('unknown_representation')
    # Changing intrinsic C is a TOOL-z rotation; Jacobian forecast needs BASE.
    world_axis=rotation(initial[3:]).apply([0.,0.,1.])
    angular_velocity_deg_s=world_axis*(direction*AMPLITUDE_DEG*derivative)
    return np.r_[initial[:3],abc].tolist(),angular_velocity_deg_s


def execute_draft(args,contract):
    validate_site_contract(contract)
    for label in ('Operator present','Workspace clear','E-stop accessible',
                  'Authority/servo normal, protective/emergency stops clear',
                  'Robot stationary at reviewed safe pose','No other motion executor',
                  'Explicit ONE low-speed 3deg test, no auto-return'):
        if input(label+' [yes/NO]: ').strip().lower()!='yes':
            raise PermissionError('fresh_operator_confirmation_missing')
    confirmed=time.monotonic()
    # ROS is deliberately imported only after explicit opt-in and approval.
    import rclpy
    from rclpy.signals import SignalHandlerOptions
    from rclpy.qos import qos_profile_sensor_data,QoSProfile,ReliabilityPolicy
    from sensor_msgs.msg import JointState
    from dsr_msgs2.msg import ServolStream
    from dsr_msgs2.srv import GetRobotSystem,GetRobotMode,GetRobotState,GetCurrentPosx,MoveStop
    from rosidl_runtime_py.convert import message_to_ordereddict
    from run_arm_only_once import JointStateStartup
    from speedl_experimental.joint_safety import JointRangePredictor,ordered_joint_state
    from servo_experimental.stop_evidence import evaluate_stop
    limits=yaml.safe_load((ROOT.parent/'01_configs/safety_limits.yaml').read_text())
    transport=yaml.safe_load((ROOT/'configs/servo_episode.yaml').read_text())['servo_transport']
    if contract.get('approved_transport')!=transport:
        raise PermissionError('site_contract_transport_does_not_match_existing_config')
    args.output.mkdir(parents=True,exist_ok=False)
    log=(args.output/'events.jsonl').open('x')
    def record(event):
        log.write(json.dumps(event,allow_nan=False)+'\n');log.flush();os.fsync(log.fileno())
    os.environ['ROS_LOCALHOST_ONLY']='1'
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node=rclpy.create_node('DRAFT_reviewed_servo_C_wrap_single_test')
    lock=threading.RLock();halt=threading.Event();rows=[];fault=[]
    gate=JointStateStartup(time.monotonic());published=0;history=[];stop_response=None
    def joint(msg):
        try:
            row=dict(receive=time.monotonic(),source=msg.header.stamp.sec+msg.header.stamp.nanosec/1e9,
                names=list(msg.name),position=list(msg.position),velocity=list(msg.velocity))
            row['header_age']=node.get_clock().now().nanoseconds/1e9-row['source']
            row['valid']=(len(row['names'])==len(row['position'])==len(row['velocity'])==6
                and set(row['names'])=={f'joint_{i}' for i in range(1,7)}
                and all(math.isfinite(x) for x in row['position']+row['velocity']))
            with lock:gate.sample(row);rows.append(row)
        except Exception as exc:fault.append(str(exc))
    node.create_subscription(JointState,'/dsr01/joint_states',joint,qos_profile_sensor_data)
    def spin():
        try:
            while not halt.is_set():rclpy.spin_once(node,timeout_sec=.005)
        except Exception as exc:fault.append(str(exc))
    spinner=threading.Thread(target=spin);spinner.start()
    def getter(kind,name,**fields):
        client=node.create_client(kind,name)
        try:
            if not client.wait_for_service(timeout_sec=1):raise RuntimeError('service_unavailable:'+name)
            req=kind.Request()
            for k,v in fields.items():setattr(req,k,v)
            future=client.call_async(req);deadline=time.monotonic()+1
            while not future.done() and time.monotonic()<deadline:time.sleep(.002)
            if not future.done():raise RuntimeError('service_timeout:'+name)
            result=message_to_ordereddict(future.result())
            if result.get('success') is not True:raise RuntimeError('service_ack_failure:'+name)
            return result
        finally:node.destroy_client(client)
    def fresh():
        with lock:
            status=gate.status(time.monotonic());row=dict(rows[-1]) if rows else None
        if fault or status['fault'] or status['phase']!='RUNTIME_READY' or row is None:
            raise RuntimeError('jointstate_not_clean:'+str(fault or status['fault']))
        return row
    urdf='/home/ubuntu/robot_ws/src/doosan-robot2/dsr_description2/urdf/a0509.urdf'
    cfg=yaml.safe_load((ROOT/'configs/joint_safety.yaml').read_text())
    margin=contract['joint_margin_deg']
    cfg.update(lower_deg=(np.array(contract['joint_lower_deg'])+margin).tolist(),
        upper_deg=(np.array(contract['joint_upper_deg'])-margin).tolist(),controller_limits_verified=True)
    predictor=JointRangePredictor(urdf,cfg)
    def guard_joints(row,velocity,target):
        q=np.rad2deg(ordered_joint_state(row))
        if np.any(q<=cfg['lower_deg']) or np.any(q>=cfg['upper_deg']):
            raise RuntimeError('joint_limit_margin')
        # Nominal FK/Jacobian forecast is additional protection, not vendor IK proof.
        flange,_=predictor.model.matrix_and_jacobian(np.deg2rad(q))
        target_tcp=np.eye(4);target_tcp[:3,3]=np.array(target[:3])/1000
        target_tcp[:3,:3]=rotation(target[3:]).as_matrix()
        target_flange=target_tcp@np.linalg.inv(offset)
        flange_abc=normalized_zyz(Rotation.from_matrix(target_flange[:3,:3]),target[3:])
        flange_target=np.r_[target_flange[:3,3]*1000,flange_abc]
        lever=(flange@offset)[:3,3]-flange[:3,3]
        flange_v_mm_s=-np.cross(np.deg2rad(velocity),lever)*1000
        proof=predictor.assess(row,np.r_[flange_v_mm_s,velocity].tolist(),now=time.monotonic(),target_pose=flange_target)
        if not proof['allowed']:raise RuntimeError('joint_forecast:'+proof['reason'])
        return q
    outcome='NOT_EXECUTED';reason=None;stop_at=None
    try:
        deadline=time.monotonic()+120
        while time.monotonic()<deadline:
            with lock:ready=gate.status(time.monotonic())['phase']=='RUNTIME_READY'
            if ready:break
            if fault:raise RuntimeError('jointstate_fault')
            time.sleep(.005)
        row=fresh()
        if np.max(abs(np.rad2deg(ordered_joint_state(row))-contract['safe_joint_deg']))>.5 or any(v!=0 for v in row['velocity']):
            raise RuntimeError('not_at_reviewed_safe_stationary_pose')
        for kind,name,key,value in ((GetRobotSystem,'system','robot_system',0),(GetRobotMode,'mode','robot_mode',1),(GetRobotState,'state','robot_state',1)):
            if getter(kind,'/dsr01/system/get_robot_'+name)[key]!=value:raise RuntimeError('hardware_preflight')
        if time.monotonic()-confirmed>=60:raise PermissionError('operator_confirmation_expired')
        initial=getter(GetCurrentPosx,'/dsr01/aux_control/get_current_posx',ref=0)['task_pos_info'][0]['data'][:6]
        initial=prepare_geometry(initial,args.direction,contract)
        if node.count_publishers('/dsr01/servol_stream'):raise RuntimeError('other_servo_publisher')
        publisher=node.create_publisher(ServolStream,'/dsr01/servol_stream',QoSProfile(depth=1,reliability=ReliabilityPolicy.RELIABLE))
        deadline=time.monotonic()+2
        while publisher.get_subscription_count()!=1 and time.monotonic()<deadline:time.sleep(.01)
        if publisher.get_subscription_count()!=1:raise RuntimeError('subscriber_count')
        # Require a reviewed controller TCP transform, never assume flange=TCP.
        offset=np.eye(4);offset[:3,3]=np.array(contract['tcp_offset_mm_zyz_deg'][:3])/1000
        offset[:3,:3]=rotation(contract['tcp_offset_mm_zyz_deg'][3:]).as_matrix()
        history=[(time.monotonic(),initial.tolist())];started=time.monotonic();previous_abc=initial[3:]
        next_tick=started;last_state=0.
        while True:
            now=time.monotonic()
            if now<next_tick:time.sleep(min(.002,next_tick-now));continue
            if now-next_tick>=.1:raise RuntimeError('stream_tick_stall')
            row=fresh();elapsed=now-started
            candidate,velocity=candidate_pose(initial,elapsed,args.direction,args.representation,previous_abc)
            for i,key in enumerate(('x','y','z')):
                if not limits['workspace_m'][key][0]<=candidate[i]/1000<=limits['workspace_m'][key][1]:
                    raise RuntimeError('workspace_rejection')
            q=guard_joints(row,velocity,candidate)
            tf,_=predictor.model.matrix_and_jacobian(np.deg2rad(q));tcp=tf@offset
            actual=np.r_[tcp[:3,3]*1000,normalized_zyz(Rotation.from_matrix(tcp[:3,:3]),previous_abc)]
            reference=history[0][1]
            for stamp,pose in reversed(history):
                if stamp<=row['receive']:reference=pose;break
            et,er=pose_distance(actual,reference)
            if et>contract['following_translation_mm'] or er>contract['following_rotation_deg']:
                raise RuntimeError('following_error_limit')
            if now-last_state>=.2:
                if getter(GetRobotState,'/dsr01/system/get_robot_state')['robot_state'] not in (1,2):
                    raise RuntimeError('robot_abnormal_state')
                last_state=time.monotonic()
            record(dict(event='REQUEST',now=now,pose=candidate,joint_deg=q.tolist(),following_mm=et,following_deg=er))
            if time.monotonic()-now>=.1:raise RuntimeError('pre_publish_stall')
            fresh()
            if publisher.get_subscription_count()!=1:raise RuntimeError('subscriber_lost')
            msg=ServolStream();msg.pos=candidate;msg.vel=transport['velocity_mm_s_deg_s'];msg.acc=transport['acceleration_mm_s2_deg_s2'];msg.time=transport['target_time_s']
            published+=1  # Includes delivery-unknown publication exceptions.
            publisher.publish(msg);history.append((time.monotonic(),candidate));previous_abc=np.array(candidate[3:])
            record(dict(event='PUBLISH_RETURNED_NOT_VENDOR_ACK',now=time.monotonic()))
            if elapsed>=DURATION_S:outcome='OPERATOR_OBSERVATION_PENDING';break
            next_tick=now+1/RATE_HZ
    except BaseException as exc:
        outcome='INVALID' if not isinstance(exc,KeyboardInterrupt) else 'OPERATOR_ABORT';reason=str(exc)
    finally:
        stop_proof=None
        if published:
            stop_at=time.monotonic()
            try:
                stop_response=getter(MoveStop,'/dsr01/motion/move_stop',stop_mode=3)
                deadline=time.monotonic()+2
                while True:
                    state=getter(GetRobotState,'/dsr01/system/get_robot_state');state_at=time.monotonic()
                    with lock:stop_rows=list(rows)
                    stop_proof=evaluate_stop(ack=True,requested_at=stop_at,robot_state=state,
                        state_received_at=state_at,joint_rows=stop_rows,now=time.monotonic())
                    if stop_proof['confirmed'] or time.monotonic()>=deadline:break
                    time.sleep(.05)
                if not stop_proof['confirmed']:outcome='INVALID';reason='stop_not_confirmed'
            except Exception as exc:
                outcome='INVALID';reason='STOP_FAILURE:'+str(exc)
                print('Stop unconfirmed. Check robot; use E-stop immediately if unsafe.',flush=True)
        halt.set();spinner.join(2)
        result=dict(status=outcome,reason=reason,mode=args.representation,direction=args.direction,
            requested_rotation_deg=AMPLITUDE_DEG,args_duration_s=DURATION_S,publish_attempts=published,
            stop_response=stop_response,stop_confirmation=stop_proof,auto_retry=False,auto_return=False,
            additional_commands_after_abort=0,physical_validation='NOT_ESTABLISHED_BY_SCRIPT')
        try:record(dict(event='SUMMARY',**result))
        finally:
            log.close();node.destroy_node();rclpy.shutdown()


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--execute-physical-test',action='store_true')
    parser.add_argument('--contract',type=Path)
    parser.add_argument('--output',type=Path)
    parser.add_argument('--representation',choices=('normalized','unwrapped'))
    parser.add_argument('--direction',type=int,choices=(-1,1))
    args=parser.parse_args()
    if not args.execute_physical_test:
        print('DRAFT_NOT_EXECUTED: explicit opt-in and reviewed site contract required.');return
    if any(getattr(args,name) is None for name in ('contract','output','representation','direction')):
        parser.error('contract/output/representation/direction required')
    signal.signal(signal.SIGTERM,lambda signum,frame:(_ for _ in ()).throw(KeyboardInterrupt()))
    execute_draft(args,json.loads(args.contract.read_text()))


if __name__=='__main__':main()
