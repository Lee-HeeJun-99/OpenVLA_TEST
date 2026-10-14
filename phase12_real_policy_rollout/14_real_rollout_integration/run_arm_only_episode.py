"""One bounded arm-only episode. No gripper initialization, IO or task outcome."""
import argparse
import base64
import hashlib
import io
import json
import math
import os
import threading
import time
from contextlib import ExitStack
from pathlib import Path
from urllib.request import Request, urlopen
import yaml
from run_arm_only_once import (TARGET, Controller, Authorization, RosServiceTransport,
    FsyncJsonlLogger, JointStateStartup, camera_rgb_image, inference_timing,
    validate_identity, A0509FlangeFK, reorder_joint_state, _scale_to)
from arm_only_episode_contract import ArmOnlyEpisodeSink, approved_episode_limits, gripper_intent
from settle_window import wait_for_settle, SETTLE_CLEAN_SECONDS, SETTLE_TIMEOUT_SECONDS


def require_approval(approval):
    fields=('operator_present','workspace_clear','estop_accessible','robot_stationary',
            'tcp_arm_only_acceptance','explicit_one_episode')
    if not all(approval.get(k) is True for k in fields):
        raise PermissionError('fresh_arm_only_episode_approval_required')
    if not 0 <= time.time()-approval.get('confirmed_wall_time',0) <= 60:
        raise PermissionError('fresh_operator_approval_expired')


def start_pose_error_deg(names,positions):
    ordered=reorder_joint_state(names,positions)
    if set(ordered)!={f'joint_{i}' for i in range(1,7)}:
        raise ValueError('start_pose_missing_joints')
    return max(abs(math.degrees(ordered[f'joint_{i+1}'])-target) for i,target in enumerate(TARGET))


def run(output, approval, requested_duration_s=None):
    require_approval(approval)
    max_steps,max_duration=approved_episode_limits(approval,requested_duration_s)
    import rclpy
    from sensor_msgs.msg import JointState, Image
    from rclpy.qos import qos_profile_sensor_data
    from dsr_msgs2.srv import GetRobotMode, GetRobotState, GetRobotSystem, GetCurrentPosx
    from rosidl_runtime_py.convert import message_to_ordereddict
    output.mkdir(parents=True,exist_ok=False)
    def save(name,data):
        (output/name).write_text(json.dumps(data,indent=2,allow_nan=False))
    save('operator_confirmation.json',approval)
    save('episode_limits.json',dict(max_steps=max_steps,max_duration_s=max_duration,
        requested_duration_s=requested_duration_s,production_default_changed=False))
    save('gripper_disabled_contract.json',dict(physical_open=False,physical_close=False,
        physical_gripper_abort=False,tool_DO_writes=False,abort_behavior='UNVERIFIED / UNUSED'))
    save('tcp_execution_basis.json',dict(formal_classification='TCP_ZERO_OFFSET_SUPPORTED_NOT_VERIFIED',
        formal_verified=False,engineering_acceptance=True,source='EXPLICIT_OPERATOR_ARM_ONLY_APPROVAL'))
    config_paths=list((Path(__file__).parent.parent/'01_configs').glob('*.yaml'))+[
        Path(__file__).parent/'configs/full_task.yaml',Path(__file__).parent/'settle_window.py']
    frozen={str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in config_paths}
    save('config_snapshot.json',frozen)
    os.environ['ROS_LOCALHOST_ONLY']='1'
    rclpy.init();node=rclpy.create_node('arm_only_full_episode_once')
    lock=threading.RLock();halt=threading.Event();abort_workers=[]
    gate=JointStateStartup(time.monotonic());samples=[];camera=[None];control=[None]
    def joint(msg):
        now=time.monotonic();stamp=msg.header.stamp.sec+msg.header.stamp.nanosec/1e9
        valid=len(msg.name)==6 and set(msg.name)=={f'joint_{i}' for i in range(1,7)} and len(msg.position)==len(msg.velocity)==6 and all(math.isfinite(v) for v in list(msg.position)+list(msg.velocity))
        row=dict(receive=now,source=stamp,header_age=node.get_clock().now().nanoseconds/1e9-stamp,
                 names=list(msg.name),position=list(msg.position),velocity=list(msg.velocity),valid=valid)
        with lock:gate.sample(row);samples.append(row)
    def image(msg):
        with lock:camera[0]=(msg,time.monotonic())
    node.create_subscription(JointState,'/dsr01/joint_states',joint,qos_profile_sensor_data)
    node.create_subscription(Image,'/zed/zed_node/rgb/color/rect/image',image,qos_profile_sensor_data)
    def status():
        with lock:return gate.status(time.monotonic())
    def fault(reason):
        c=control[0]
        if c and not c.aborted:
            worker=threading.Thread(target=lambda:c.abort(reason));abort_workers.append(worker);worker.start()
    def spin():
        try:
            while not halt.is_set():
                rclpy.spin_once(node,timeout_sec=.005)
                s=status()
                if s['fault']:fault('jointstate_runtime_fault:'+s['fault'])
                if (output/'operator_abort.request').exists():fault('operator_abort')
                with lock:latest=camera[0]
                if control[0] and (latest is None or time.monotonic()-latest[1]>=.5):fault('camera_stream_loss')
        except Exception as exc:
            with lock:gate.fail('executor_error:'+str(exc))
            fault('executor_error')
    spinner=threading.Thread(target=spin);spinner.start()
    def check():
        s=status()
        if s['fault']:raise RuntimeError('jointstate_runtime_fault:'+s['fault'])
        if control[0] and control[0].aborted:raise RuntimeError(control[0].abort_reason)
        return s
    def getter(name,kind,**fields):
        check();client=node.create_client(kind,name)
        try:
            if not client.wait_for_service(timeout_sec=2):raise TimeoutError('getter_endpoint:'+name)
            req=kind.Request()
            for key,value in fields.items():setattr(req,key,value)
            future=client.call_async(req);deadline=time.monotonic()+2
            while not future.done() and time.monotonic()<deadline:check();time.sleep(.002)
            if not future.done():raise TimeoutError('getter_response:'+name)
            result=message_to_ordereddict(future.result());check()
            if result.get('success') is not True:raise RuntimeError('getter_failed:'+name)
            return result
        finally:node.destroy_client(client)
    def settle():
        # Consume every new callback, not just whichever latest sample happens
        # to be visible when the main thread polls.
        with lock:cursor=[len(samples)]
        def drain():
            with lock:
                batch=[dict(r) for r in samples[cursor[0]:]]
                cursor[0]=len(samples)
                return batch
        return wait_for_settle(drain,check,clock=time.monotonic,sleep=time.sleep)
    sink=None;error=None;started=None;completed=0;attempts=0;records=[];termination=None
    outcome='NOT_EXECUTED';fk=A0509FlangeFK('/home/ubuntu/robot_ws/src/doosan-robot2/dsr_description2/urdf/a0509.urdf')
    try:
        while status()['phase']!='RUNTIME_READY':check();time.sleep(.005)
        hardware={}
        for key,kind in [('mode',GetRobotMode),('state',GetRobotState),('system',GetRobotSystem)]:
            hardware[key]=getter('/dsr01/system/get_robot_'+key,kind)
            if hardware[key]['robot_'+key]!=dict(mode=1,state=1,system=0)[key]:raise ValueError('hardware_not_REAL_AUTO_STANDBY')
        with lock:before=dict(samples[-1])
        pose_error=start_pose_error_deg(before['names'],before['position'])
        save('start_pose_check.json',dict(jointstate=before,hardware=hardware,target_joint_deg=TARGET,
             max_error_deg=pose_error,tolerance_deg=.5,within_tolerance=pose_error<=.5))
        if pose_error>.5 or any(v!=0 for v in before['velocity']):raise ValueError('start_pose_or_stationary_failed')
        with urlopen('http://127.0.0.1:8766/health',timeout=2) as response:health=json.load(response)
        validate_identity(health,'openvla')
        if health.get('status')!='ready':raise ValueError('model_not_ready')
        save('model_health.json',health)
        with lock:cam=camera[0]
        if cam is None or time.monotonic()-cam[1]>=.5:raise ValueError('camera_preflight_failed')
        save('preflight.json',dict(hardware=hardware,jointstate=check(),start_pose_error_deg=pose_error,
             camera_ready=True,model_ready=True,max_steps=max_steps,max_duration_s=max_duration,
             physical_gripper_disabled=True,task_success_not_applicable=True,
             settle_policy=dict(clean_seconds=SETTLE_CLEAN_SECONDS,timeout_seconds=SETTLE_TIMEOUT_SECONDS,
                                velocity_criterion='SIX_FINITE_EXACT_ZERO_VELOCITIES')))
        require_approval(approval)
        sink=ArmOnlyEpisodeSink(lambda:RosServiceTransport(node),Authorization(True,True,True,'MOTION_ENABLED'))
        # One synchronous loop; logger requests are fsynced before each dispatch.
        with ExitStack() as stack:
            logs={name:stack.enter_context(FsyncJsonlLogger(output/f'{name}.jsonl')) for name in
                  ('runtime_events','observations','predictions','action_pipeline','arm_commands','jointstate')}
            log=logs['runtime_events']
            log.append(dict(event='LOGGER_PREFLIGHT',scope='ARM_ONLY_FULL_EPISODE_SYSTEM_INTEGRATION_TEST',max_steps=max_steps,max_duration_s=max_duration))
            c=Controller('openvla','full_task',sink,log,dry_run=False,execution_mode='ARM_ONLY_FULL_EPISODE')
            control[0]=c;started=time.monotonic()
            for step in range(max_steps):
                if time.monotonic()-started>=max_duration:termination='EXISTING_EPISODE_DURATION_LIMIT';break
                check()
                if any(hashlib.sha256(Path(p).read_bytes()).hexdigest()!=h for p,h in frozen.items()):raise RuntimeError('config_changed')
                _,pre_observation_settle=settle()
                if time.monotonic()-started>=max_duration:termination='EXISTING_EPISODE_DURATION_LIMIT';break
                current=getter('/dsr01/aux_control/get_current_posx',GetCurrentPosx,ref=0)['task_pos_info'][0]['data'][:6]
                if len(current)!=6 or not all(math.isfinite(v) for v in current):raise ValueError('invalid_current_pose')
                deadline=time.monotonic()+2
                while time.monotonic()<deadline:
                    check()
                    with lock:selected=camera[0]
                    if selected and 0<=time.monotonic()-selected[1]<=.01:break
                    time.sleep(.001)
                else:raise TimeoutError('fresh_camera_timeout')
                msg,received=selected
                if msg.encoding!='bgra8' or [msg.width,msg.height]!=[1280,720]:raise ValueError('camera_contract')
                rgb=camera_rgb_image(msg);buf=io.BytesIO();rgb.save(buf,format='JPEG',quality=95);jpeg=buf.getvalue()
                req=Request('http://127.0.0.1:8766/predict',data=json.dumps(dict(image_jpeg_base64=base64.b64encode(jpeg).decode(),instruction='Pick up the orange cube.')).encode(),headers={'Content-Type':'application/json'})
                ros_start=node.get_clock().now().nanoseconds/1e9;t0=time.monotonic()
                if not 0<=t0-received<.5:raise ValueError('camera_stale_at_inference_start')
                if time.monotonic()-started>=max_duration:termination='EXISTING_EPISODE_DURATION_LIMIT';break
                check();attempts+=1
                with urlopen(req,timeout=1.2) as response:prediction=json.load(response)
                t1=time.monotonic();raw=prediction['action']
                if len(raw)!=7 or not all(math.isfinite(v) for v in raw):raise ValueError('invalid_model_action')
                intent=gripper_intent(raw[6])
                timing=inference_timing(received,t0,t1,source_stamp=msg.header.stamp.sec+msg.header.stamp.nanosec/1e9,
                                       ros_start=ros_start,ros_end=node.get_clock().now().nanoseconds/1e9)
                with lock:before=dict(samples[-1])
                log.append(dict(event='PREDICTION',step_id=step,raw_action=raw,logical_gripper_intent=intent,
                                physical_gripper_command='NONE',camera_timing=timing,jointstate=before))
                logs['observations'].append(dict(step_id=step,current_pose=current,jointstate=before,
                    camera_timing=timing,source_image_sha256=hashlib.sha256(bytes(msg.data)).hexdigest(),model_input_sha256=hashlib.sha256(jpeg).hexdigest()))
                logs['predictions'].append(dict(step_id=step,raw_action=raw,translation_norm_m=math.dist(raw[:3],[0,0,0]),
                    rotation_norm_rad=math.dist(raw[3:6],[0,0,0]),raw_gripper_prediction=raw[6],logical_gripper_intent=intent,
                    physical_gripper_command='NONE',camera_timing=timing))
                rgb.save(output/f'camera_{step:03d}.png')
                observation=dict(receive_monotonic=time.monotonic(),tcp_m_abc=[v/1000 for v in current[:3]]+current[3:],
                    phase='alignment',robot_state_ok=True,camera_ok=True,joint_state_ok=True,tcp_ok=True,
                    model_ok=True,communication_ok=True,live_execution_clock=True,
                    tcp_source='CONTROLLER_ZERO_OFFSET_ENGINEERING_ACCEPTED',tcp_contract_verified=False,raw_model_output=prediction)
                # No dispatch after the existing episode deadline, including slow inference.
                if time.monotonic()-started>=max_duration:termination='EXISTING_EPISODE_DURATION_LIMIT';break
                # Current state is checked again after inference, before dispatch.
                if getter('/dsr01/system/get_robot_state',GetRobotState)['robot_state']!=1:
                    raise RuntimeError('robot_not_STANDBY_before_dispatch')
                before,pre_dispatch_settle=settle()
                log.append(dict(event='SETTLE_CONFIRMED',step_id=step,stage='PRE_DISPATCH',
                    evidence=pre_dispatch_settle,jointstate=before))
                if time.monotonic()-started>=max_duration:termination='EXISTING_EPISODE_DURATION_LIMIT';break
                check();record=c.step(raw,observation,inference_time=t0,chunk_id=f'episode-step-{step}',chunk_index=0)
                records.append(dict(step_id=step,raw_action=raw,camera_timing=timing,logical_gripper_intent=intent,
                                    pre_observation_settle=pre_observation_settle,pre_dispatch_settle=pre_dispatch_settle,
                                    physical_gripper_command='NONE',source_image_sha256=hashlib.sha256(bytes(msg.data)).hexdigest(),
                                    model_input_sha256=hashlib.sha256(jpeg).hexdigest(),record=record,jointstate_before=before))
                check()
                if not record.get('command_receipt') or record['command_receipt']['state']!='completed':raise RuntimeError('arm_ack_failure')
                limiter=c.pipeline.runtime.limiter
                v_t,_=_scale_to(tuple(v/limiter.dt for v in raw[:3]),limiter.tv)
                v_r,_=_scale_to(tuple(v/limiter.dt for v in raw[3:6]),limiter.rv)
                logs['action_pipeline'].append(dict(step_id=step,RAW_MODEL_ACTION=raw,POST_HARD_GATE=raw,
                    POST_VELOCITY_LIMIT=[v*limiter.dt for v in v_t+v_r]+[raw[6]],
                    POST_ACCELERATION_LIMIT=record['filtered_action'],FINAL_DELIVERED_ARM_ACTION=record['filtered_action'][:6],
                    physical_gripper_command='NONE'))
                logs['arm_commands'].append(dict(step_id=step,receipt=record['command_receipt']))
                post=getter('/dsr01/system/get_robot_state',GetRobotState)
                if post['robot_state']!=1:raise RuntimeError('robot_not_STANDBY_after_motion')
                after,post_command_settle=settle()
                records[-1].update(jointstate_after=after,robot_state_after=post,
                    post_command_settle=post_command_settle,
                    fk_before=fk.compute(reorder_joint_state(before['names'],before['position'])),
                    fk_after=fk.compute(reorder_joint_state(after['names'],after['position'])))
                logs['jointstate'].append(dict(step_id=step,before=before,after=after,
                    fk_before=records[-1]['fk_before'],fk_after=records[-1]['fk_after'],robot_state=post))
                completed+=1
            else:termination='EXISTING_EPISODE_STEP_LIMIT'
            check();outcome='OPERATOR_RESULT_CONFIRMATION_PENDING'
    except KeyboardInterrupt:
        error='OPERATOR_ABORT';outcome='OPERATOR_ABORT';fault(error)
    except Exception as exc:
        error=str(exc);outcome='OPERATOR_ABORT' if error=='operator_abort' else 'ARM_ONLY_FULL_EPISODE_INVALID' if started is not None else 'NOT_EXECUTED';fault(error)
    finally:
        for worker in abort_workers:worker.join(4)
        halt.set();spinner.join(2)
        events=sink.events if sink else []
        result=dict(status=outcome,reason=error,evaluation_scope='ARM_ONLY_FULL_EPISODE_SYSTEM_INTEGRATION_TEST',
            termination=termination,task_success=None,task_performance_denominator_included=False,
            steps_attempted=attempts,steps_completed=completed,duration_s=None if started is None else time.monotonic()-started,
            arm_commands=sum(e['command_type']=='pose' and e['sent_at'] is not None for e in events),
            physical_gripper_commands=sum(e['command_type']=='gripper' and e['sent_at'] is not None for e in events),
            stop_commands=sum(e['command_type'] in ('hold','stop') and e['sent_at'] is not None for e in events),
            gripper_abort_behavior='UNVERIFIED / UNUSED',unexpected_motion=None,estop_used=None,
            auto_retry=False,auto_home=False,auto_recovery=False,auto_next_episode=False)
        try:
            save('trial_summary.json',result)
            save('episode_records.json',records)
            save('jointstate.json',dict(samples=samples,events=gate.events,status=status()))
            save('arm_commands.json',events)
        finally:node.destroy_node();rclpy.shutdown()
        print(json.dumps(result,indent=2))


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--output',type=Path,required=True);p.add_argument('--approval',type=Path,required=True)
    p.add_argument('--max-duration-s',type=float)
    args=p.parse_args();run(args.output,json.loads(args.approval.read_text()),args.max_duration_s)
