"""One existing +0.5mm MoveLine and existing AbortBoundary during observed motion."""
import argparse,json,math,threading,time
from pathlib import Path
from run_real_rollout import Authorization,RealDoosanCommandSink,RosServiceTransport,AbortBoundary
from jointstate_runtime_startup import JointStateStartup
from workspace_contract import PHASE12_DATA_DERIVED_WORKSPACE


def run(output,approved_time,delta_mm=.5,complete_once=False):
    if not math.isfinite(delta_mm) or not 0 < delta_mm <= 10:raise ValueError('stop_test_distance_outside_explicitly_approved_range')
    if not 0<=time.time()-approved_time<=60:raise PermissionError('fresh_operator_approval_expired')
    import os,rclpy
    from sensor_msgs.msg import JointState
    from rclpy.qos import qos_profile_sensor_data
    from dsr_msgs2.srv import GetRobotMode,GetRobotState,GetRobotSystem,GetCurrentPosx
    from rosidl_runtime_py.convert import message_to_ordereddict
    output.mkdir(parents=True,exist_ok=False)
    def save(name,value):(output/name).write_text(json.dumps(value,indent=2))
    os.environ['ROS_LOCALHOST_ONLY']='1';rclpy.init();node=rclpy.create_node('approved_physical_stop_once')
    halt=threading.Event();lock=threading.RLock();gate=JointStateStartup(time.monotonic());rows=[]
    def joint(msg):
        now=time.monotonic();stamp=msg.header.stamp.sec+msg.header.stamp.nanosec/1e9
        valid=len(msg.name)==6 and set(msg.name)=={f'joint_{i}' for i in range(1,7)} and len(msg.position)==len(msg.velocity)==6 and all(math.isfinite(v) for v in list(msg.position)+list(msg.velocity))
        row=dict(receive=now,source=stamp,header_age=node.get_clock().now().nanoseconds/1e9-stamp,names=list(msg.name),position=list(msg.position),velocity=list(msg.velocity),valid=valid)
        with lock:gate.sample(row);rows.append(row)
    node.create_subscription(JointState,'/dsr01/joint_states',joint,qos_profile_sensor_data)
    class NoGripperSink(RealDoosanCommandSink):
        def _send(self,kind,request,**kw):
            if kind=='gripper':raise PermissionError('stop_test_gripper_disabled')
            return super()._send(kind,request,**kw)
    sink=NoGripperSink(lambda:RosServiceTransport(node),Authorization(True,True,True,'MOTION_ENABLED'));boundary=AbortBoundary(sink)
    started_motion=threading.Event();watch_stop_threads=[]
    def spin():
        while not halt.is_set():
            rclpy.spin_once(node,timeout_sec=.005)
            with lock:s=gate.status(time.monotonic())
            if s['fault'] and started_motion.is_set() and boundary.state=='RUNNING':
                worker=threading.Thread(target=lambda:boundary.abort('jointstate_runtime_fault'));watch_stop_threads.append(worker);worker.start()
    thread=threading.Thread(target=spin);thread.start()
    def check():
        with lock:s=gate.status(time.monotonic())
        if s['fault']:raise RuntimeError('jointstate_runtime_fault:'+s['fault'])
        return s
    def getter(name,kind,**fields):
        check();client=node.create_client(kind,name)
        try:
            if not client.wait_for_service(timeout_sec=2):raise TimeoutError('getter_endpoint')
            req=kind.Request()
            for key,value in fields.items():setattr(req,key,value)
            future=client.call_async(req);deadline=time.monotonic()+2
            while not future.done() and time.monotonic()<deadline:check();time.sleep(.002)
            if not future.done():raise TimeoutError('getter_response')
            response=message_to_ordereddict(future.result())
            if response.get('success') is not True:raise ValueError('getter_failure')
            return response
        finally:node.destroy_client(client)
    receipt=[];motion_thread=None;before_stop=None;stop_result=None;hardware={};error=None;final_state=None;zero_samples=[]
    try:
        while check()['phase']!='RUNTIME_READY':time.sleep(.005)
        for key,kind in [('mode',GetRobotMode),('state',GetRobotState),('system',GetRobotSystem)]:
            hardware[key]=getter('/dsr01/system/get_robot_'+key,kind)
            if hardware[key]['robot_'+key]!=dict(mode=1,state=1,system=0)[key]:raise RuntimeError('hardware_not_REAL_AUTO_STANDBY')
        with lock:baseline=dict(rows[-1])
        if any(v!=0 for v in baseline['velocity']):raise RuntimeError('not_stationary')
        cp=getter('/dsr01/aux_control/get_current_posx',GetCurrentPosx,ref=0)['task_pos_info'][0]['data'][:6];target=list(cp);target[0]+=delta_mm
        if not PHASE12_DATA_DERIVED_WORKSPACE.inspect([v/1000 for v in target[:3]]).accepted:raise RuntimeError('workspace_reject')
        if not 0<=time.time()-approved_time<=60:raise PermissionError('fresh_operator_approval_expired')
        save('motion_request.json',dict(current_pose=cp,target_pose=target,delta_mm=[delta_mm,0,0],rotation_delta=0,ref=0,mode=0,velocity=[20,20],acceleration=[20,20],engineering_tcp_acceptance='zero_offset_consistency_for_bounded_integration_test'))
        check();started_motion.set()
        motion_thread=threading.Thread(target=lambda:receipt.append(sink.send_pose(target)));motion_thread.start()
        deadline=time.monotonic()+2
        while time.monotonic()<deadline:
            check()
            with lock:latest=dict(rows[-1])
            event=next((e for e in sink.events if e['command_type']=='pose'),None)
            if event and event['sent_at'] is not None and latest['receive']>event['sent_at'] and any(v!=0 for v in latest['velocity']):
                before_stop=latest;break
            if not motion_thread.is_alive():break
            time.sleep(.001)
        if not complete_once:
            if before_stop is None:raise RuntimeError('LIVE_MOTION_NOT_OBSERVED_BEFORE_COMMAND_COMPLETED; no stop proof or auto retry')
            stop_result=boundary.abort('operator_approved_physical_stop_test')
        motion_thread.join(4)
        if motion_thread.is_alive():raise RuntimeError('motion_ack_did_not_terminate')
        if complete_once and (not receipt or receipt[0].get('state')!='completed' or receipt[0].get('result',{}).get('success') is not True):raise RuntimeError('single_motion_ack_failure')
        deadline=time.monotonic()+2
        while time.monotonic()<deadline:check();time.sleep(.005)
        with lock:zero_samples=[dict(r) for r in rows if sink.events[-1]['ack_at'] is not None and r['receive']>=sink.events[-1]['ack_at']]
        final_state=getter('/dsr01/system/get_robot_state',GetRobotState)['robot_state']
        if final_state!=1:raise RuntimeError('final_state_not_STANDBY:'+str(final_state))
        if not zero_samples or any(any(v!=0 for v in r['velocity']) for r in zero_samples[-20:]):raise RuntimeError('standstill_not_confirmed')
        if not complete_once:
            try:sink.send_pose(target)
            except RuntimeError:pass
            else:raise RuntimeError('post_abort_command_not_inhibited')
    except Exception as exc:
        error=str(exc)
        if motion_thread and motion_thread.is_alive() and boundary.state=='RUNNING':stop_result=boundary.abort(error)
    finally:
        if motion_thread:motion_thread.join(4)
        for worker in watch_stop_threads:worker.join(4)
        stops=[e for e in sink.events if e['command_type'] in ('hold','stop') and e['sent_at'] is not None]
        latency=dict(service_request_to_ack_s=[e['ack_latency'] for e in stops],observed_last_moving_sample=before_stop,
                     observed_standstill_sample=zero_samples[-1] if zero_samples else None,ack_is_not_physical_stop_latency=True)
        result=dict(status='OPERATOR_STOP_CONFIRMATION_PENDING' if not error and before_stop and stops and stops[0]['state']=='completed' else 'PHYSICAL_STOP_PATH_FAIL',error=error,physical_arm_commands=sum(e['command_type']=='pose' and e['sent_at'] is not None for e in sink.events),physical_stop_commands=len(stops),physical_gripper_commands=0,stop_result=stop_result,motion_receipt=receipt,events=sink.events,final_robot_state=final_state,commands_after_abort=sum(1 for e in sink.events if e['command_type']=='pose' and e['sent_at'] is not None and stops and e['sent_at']>stops[0]['requested_at']),operator_estop_used=None,auto_retry=False)
        if complete_once:
            result['status']='SINGLE_MOTION_OPERATOR_CONFIRMATION_PENDING' if not error else 'SINGLE_MOTION_FAIL'
            result['scope']='BOUNDED_10MM_COMMAND_PATH_TEST_NOT_STOP_VALIDATION'
        save('hardware_current.json',hardware);save('physical_stop_test.json',result);save('stop_latency.json',latency)
        save('jointstate_runtime.json',dict(status=gate.status(time.monotonic()),events=gate.events,rows=rows))
        halt.set();thread.join(2);node.destroy_node();rclpy.shutdown();print(json.dumps(result,indent=2))


if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('--output',type=Path,required=True);parser.add_argument('--approved-time',type=float,required=True)
    parser.add_argument('--delta-mm',type=float,default=.5)
    args=parser.parse_args();run(args.output,args.approved_time,args.delta_mm)
