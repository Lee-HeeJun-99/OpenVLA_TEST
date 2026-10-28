"""Explicit IO-only OFF/OFF + interlocked OPEN calibration, never pulse/arm."""
import json,math,threading,time
from pathlib import Path
from run_real_rollout import Authorization,RealDoosanCommandSink,RosServiceTransport
from jointstate_runtime_startup import JointStateStartup


class OpenCalibrationSink(RealDoosanCommandSink):
    approved_sequence=((1,0),(2,0),(1,1),(1,0),(1,1),(1,0))
    def _send(self,kind,request,**kwargs):
        if hasattr(self,'before_write_check'):self.before_write_check()
        index=len(self.events)
        if (kind!='gripper' or kwargs.get('emergency') or index>=len(self.approved_sequence)
                or (request.get('index'),request.get('value'))!=self.approved_sequence[index]):
            raise PermissionError('approved_initialization_and_OPEN_only')
        return super()._send(kind,request,**kwargs)


class CloseCalibrationSink(OpenCalibrationSink):
    approved_sequence=((2,1),(2,0))


class ResumeOpenCalibrationSink(OpenCalibrationSink):
    approved_sequence=((1,1),(1,0),(1,1),(1,0))


def run(output,approved_time,state='OPEN',prior_open_evidence=None,resume_open=False):
    if state not in ('OPEN','CLOSE'):raise ValueError('invalid_calibration_state')
    if resume_open:
        if state!='OPEN' or prior_open_evidence is None:raise PermissionError('prior_initialization_required')
        prior=json.loads(Path(prior_open_evidence).read_text())
        if ((prior.get('initialization') or {}).get('state')!='completed'
                or (prior.get('transition') or {}).get('command_mode')!='operator_timed_pulse'):
            raise PermissionError('prior_current_contract_initialization_required')
    if state=='CLOSE':
        if prior_open_evidence is None or json.loads(Path(prior_open_evidence).read_text()).get('observed_physical_state')!='OPEN':
            raise PermissionError('actual_prior_OPEN_confirmation_required')
        prior=json.loads(Path(prior_open_evidence).with_name('gripper_level_open_result.json').read_text())
        if (prior.get('transition') or {}).get('command_mode')!='operator_timed_pulse':
            raise PermissionError('prior_OPEN_must_use_current_pulse_contract')
    if not 0<=time.time()-approved_time<=60:raise PermissionError('fresh_operator_approval_expired')
    import os,rclpy
    from sensor_msgs.msg import JointState
    from rclpy.qos import qos_profile_sensor_data
    from dsr_msgs2.srv import GetRobotMode,GetRobotState,GetRobotSystem,GetToolDigitalOutput
    from rosidl_runtime_py.convert import message_to_ordereddict
    output.mkdir(parents=True,exist_ok=False)
    def save(name,data):(output/name).write_text(json.dumps(data,indent=2))
    os.environ['ROS_LOCALHOST_ONLY']='1';rclpy.init();node=rclpy.create_node('gripper_level_open_calibration_once')
    lock=threading.RLock();halt=threading.Event();gate=JointStateStartup(time.monotonic());rows=[]
    def joint(msg):
        now=time.monotonic();stamp=msg.header.stamp.sec+msg.header.stamp.nanosec/1e9
        valid=len(msg.name)==6 and set(msg.name)=={f'joint_{i}' for i in range(1,7)} and len(msg.position)==len(msg.velocity)==6 and all(math.isfinite(v) for v in list(msg.position)+list(msg.velocity))
        row=dict(receive=now,source=stamp,header_age=node.get_clock().now().nanoseconds/1e9-stamp,names=list(msg.name),position=list(msg.position),velocity=list(msg.velocity),valid=valid)
        with lock:gate.sample(row);rows.append(row)
    node.create_subscription(JointState,'/dsr01/joint_states',joint,qos_profile_sensor_data)
    def spin():
        while not halt.is_set():
            rclpy.spin_once(node,timeout_sec=.005)
            with lock:gate.status(time.monotonic())
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
            for k,v in fields.items():setattr(req,k,v)
            future=client.call_async(req);deadline=time.monotonic()+2
            while not future.done() and time.monotonic()<deadline:check();time.sleep(.002)
            if not future.done():raise TimeoutError('getter_response')
            result=message_to_ordereddict(future.result())
            if result.get('success') is not True:raise RuntimeError('getter_failed')
            return result
        finally:node.destroy_client(client)
    cfg=dict(command_mode='operator_timed_pulse',polarity_confirmed=True,
        gripper_open_output_index=1,gripper_closed_output_index=2,
        gripper_open_hardware_value=1,gripper_closed_hardware_value=1,inactive_hardware_value=0,
        open_first_high_hold_s=1.3,open_second_high_hold_s=.5,close_high_hold_s=.5,
        open_between_pulses_low_hold_s=.2)
    cls=ResumeOpenCalibrationSink if resume_open else OpenCalibrationSink if state=='OPEN' else CloseCalibrationSink
    sink=cls(lambda:RosServiceTransport(node),Authorization(True,True,True,'MOTION_ENABLED'),gripper_config=cfg)
    sink.before_write_check=check
    hardware={};readback={};pre_readback={};error=None;initialization=None;opening=None
    try:
        while check()['phase']!='RUNTIME_READY':time.sleep(.005)
        for key,kind in [('mode',GetRobotMode),('state',GetRobotState),('system',GetRobotSystem)]:
            hardware[key]=getter('/dsr01/system/get_robot_'+key,kind)
            if hardware[key]['robot_'+key]!=dict(mode=1,state=1,system=0)[key]:raise RuntimeError('hardware_not_REAL_AUTO_STANDBY')
        with lock:latest=rows[-1]
        if any(v!=0 for v in latest['velocity']):raise RuntimeError('arm_not_stationary')
        if not 0<=time.time()-approved_time<=60:raise PermissionError('fresh_operator_approval_expired')
        check()
        if resume_open:
            for index in (1,2):pre_readback[str(index)]=getter('/dsr01/io/get_tool_digital_output',GetToolDigitalOutput,index=index)
            if [pre_readback[str(i)]['value'] for i in (1,2)]!=[0,0]:
                raise RuntimeError('outputs_not_OFF_OFF_before_resumed_OPEN')
            sink.gripper_outputs_initialized=True
        elif state=='OPEN':
            initialization=sink.initialize_gripper_outputs()
            if initialization['state']!='completed':raise RuntimeError('initialization_ACK_failed')
        else:
            # Prior OFF/OFF + actual OPEN was confirmed; never repeat reset or
            # OPEN before this separately approved CLOSE transition.
            for index in (1,2):pre_readback[str(index)]=getter('/dsr01/io/get_tool_digital_output',GetToolDigitalOutput,index=index)
            if [pre_readback[str(i)]['value'] for i in (1,2)]!=[0,0]:
                raise RuntimeError('outputs_not_OFF_OFF_before_CLOSE')
            sink.gripper_outputs_initialized=True;sink.last_gripper=False
        check()
        # Calibration-only call: normal level sequence is tested, not an
        # unverified abort contract. Production send_gripper remains gated.
        opening=sink._send_timed_gripper(state=='CLOSE')
        if opening['state']!='completed':raise RuntimeError(state+'_ACK_failed')
        for index in (1,2):readback[str(index)]=getter('/dsr01/io/get_tool_digital_output',GetToolDigitalOutput,index=index)
        if [readback[str(i)]['value'] for i in (1,2)]!=[0,0]:raise RuntimeError(state+'_electrical_readback_mismatch')
        deadline=time.monotonic()+2
        while time.monotonic()<deadline:check();time.sleep(.005)
    except Exception as exc:error=str(exc);sink.inhibit()
    finally:
        halt.set();thread.join(2)
        result=dict(status='OPERATOR_'+state+'_CONFIRMATION_PENDING' if not error else 'CALIBRATION_FAILED',
            requested_state=state,error=error,initialization=initialization,transition=opening,electrical_readback=readback,
            pre_readback=pre_readback,resumed_without_reset=resume_open,
            physical_arm_commands=0,physical_stop_commands=0,physical_CLOSE_commands=int(state=='CLOSE' and opening is not None),
            io_write_count=sum(e['sent_at'] is not None for e in sink.events),events=sink.events,
            normal_sequence_trailing_OFF=True,physical_jaw_state=None,abort_behavior_verified=False,
            calibration_only=True,production_gate_bypassed_for_rollout=False,
            operator_approval_time=approved_time,automatic_retry=False)
        save('hardware_current.json',hardware);save('jointstate_runtime.json',dict(rows=rows,status=gate.status(time.monotonic()),events=gate.events))
        save('gripper_level_'+state.lower()+'_result.json',result)
        node.destroy_node();rclpy.shutdown();print(json.dumps(result,indent=2))
