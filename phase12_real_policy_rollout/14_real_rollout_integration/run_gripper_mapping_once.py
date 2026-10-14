"""Explicit gripper calibration only; arm/Stop methods are denied."""
import argparse,json,math,threading,time
from pathlib import Path
from run_real_rollout import Authorization,RealDoosanCommandSink,RosServiceTransport
from jointstate_runtime_startup import JointStateStartup


def run(output,state,confirmed_wall_time,pulse_duration=1.):
    if pulse_duration not in (1.,4.) or (state=='OPEN' and pulse_duration!=1.):raise ValueError('unsupported_existing_pulse_contract')
    if not 0<=time.time()-confirmed_wall_time<=60:raise PermissionError('fresh_operator_approval_expired')
    import os,rclpy
    from sensor_msgs.msg import JointState
    from rclpy.qos import qos_profile_sensor_data
    from dsr_msgs2.srv import GetRobotMode,GetRobotState,GetRobotSystem
    from rosidl_runtime_py.convert import message_to_ordereddict
    output.mkdir(parents=True,exist_ok=False)
    def save(name,value):(output/name).write_text(json.dumps(value,indent=2))
    os.environ['ROS_LOCALHOST_ONLY']='1';rclpy.init();node=rclpy.create_node('approved_gripper_calibration_once')
    gate=JointStateStartup(time.monotonic());lock=threading.RLock();halt=threading.Event();rows=[]
    def callback(msg):
        now=time.monotonic();stamp=msg.header.stamp.sec+msg.header.stamp.nanosec/1e9
        valid=len(msg.name)==6 and set(msg.name)=={f'joint_{i}' for i in range(1,7)} and len(msg.position)==len(msg.velocity)==6 and all(math.isfinite(v) for v in list(msg.position)+list(msg.velocity))
        row=dict(receive=now,source=stamp,header_age=node.get_clock().now().nanoseconds/1e9-stamp,names=list(msg.name),position=list(msg.position),velocity=list(msg.velocity),valid=valid)
        with lock:gate.sample(row);rows.append(row)
    node.create_subscription(JointState,'/dsr01/joint_states',callback,qos_profile_sensor_data)
    def spin():
        while not halt.is_set():
            rclpy.spin_once(node,timeout_sec=.005)
            with lock:gate.status(time.monotonic())
    thread=threading.Thread(target=spin);thread.start()
    def check():
        with lock:s=gate.status(time.monotonic())
        if s['fault']:raise RuntimeError('jointstate_runtime_fault:'+s['fault'])
        return s
    class CalibrationSink(RealDoosanCommandSink):
        def _send(self,kind,request,**kwargs):
            if kind!='gripper' or request['index']!=index or request['value'] not in (0,1):raise PermissionError('calibration_IO_only')
            return super()._send(kind,request,**kwargs)
    index=2 if state=='CLOSE' else 1
    sink=CalibrationSink(lambda:RosServiceTransport(node),Authorization(True,True,True,'MOTION_ENABLED'))
    active=False;error=None;hardware={};on=None;off=None
    try:
        while check()['phase']!='RUNTIME_READY':time.sleep(.005)
        for key,kind in [('mode',GetRobotMode),('state',GetRobotState),('system',GetRobotSystem)]:
            client=node.create_client(kind,'/dsr01/system/get_robot_'+key)
            try:
                if not client.wait_for_service(timeout_sec=2):raise TimeoutError('hardware_endpoint')
                future=client.call_async(kind.Request());deadline=time.monotonic()+2
                while not future.done() and time.monotonic()<deadline:check();time.sleep(.002)
                if not future.done():raise TimeoutError('hardware_response')
                result=message_to_ordereddict(future.result());hardware[key]=result
                if result.get('success') is not True or result.get('robot_'+key)!=dict(mode=1,state=1,system=0)[key]:raise RuntimeError('hardware_not_REAL_AUTO_STANDBY')
            finally:node.destroy_client(client)
        if any(v!=0 for v in rows[-1]['velocity']):raise RuntimeError('robot_not_stationary')
        if not 0<=time.time()-confirmed_wall_time<=60:raise PermissionError('fresh_operator_approval_expired')
        check();active=True
        on=sink._send('gripper',dict(index=index,value=1))
        if on['state']!='completed':raise RuntimeError('gripper_activation_ack_failed')
        deadline=time.monotonic()+pulse_duration
        while time.monotonic()<deadline:check();time.sleep(.005)
    except Exception as exc:error=str(exc)
    finally:
        if active:
            off=sink._send('gripper',dict(index=index,value=0),emergency=True)
            if off['state']!='completed':error='gripper_deactivation_ack_failed'
        if not error:
            deadline=time.monotonic()+2
            try:
                while time.monotonic()<deadline:check();time.sleep(.005)
            except Exception as exc:error=str(exc)
        save('hardware_current.json',hardware)
        save('jointstate_runtime.json',dict(rows=rows,status=gate.status(time.monotonic()),events=gate.events))
        result=dict(status='PHYSICAL_RESULT_CONFIRMATION_PENDING' if active and not error else 'NOT_EXECUTED' if not active else 'GRIPPER_MAPPING_FAILED',error=error,requested_logical_state=state,hardware_output_index=index,activation_value=1,deactivation_value=0,pulse_duration_s=pulse_duration,logical_command_count=int(active),io_service_requests=len(sink.events),events=sink.events,physical_arm_commands=0,physical_stop_commands=0,actual_physical_state=None,polarity_globally_confirmed=False,abort_jaw_behavior_verified=False,operator_confirmation_wall_time=confirmed_wall_time,source='existing runtime/config close pulse4s or historical collection pulse1s; channel/value unchanged',open_pulse_count_this_test=1 if state=='OPEN' else None,historical_open_pulse_count=2)
        save('gripper_mapping_test.json',result)
        halt.set();thread.join(2);node.destroy_node();rclpy.shutdown();print(json.dumps(result,indent=2))


if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('--output',type=Path,required=True);parser.add_argument('--state',choices=['OPEN','CLOSE'],required=True);parser.add_argument('--confirmed-wall-time',type=float,required=True);parser.add_argument('--pulse-duration',type=float,choices=[1.,4.],default=1.)
    args=parser.parse_args();run(args.output,args.state,args.confirmed_wall_time,args.pulse_duration)
