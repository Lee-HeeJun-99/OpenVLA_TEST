"""Controlled read-only pose crosscheck; no robot commands or TCP assumptions."""
import argparse,json,math,time,sys
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'03_shadow_mode'))
from jointstate_runtime_startup import JointStateStartup
from jointstate_flange_fk import A0509FlangeFK,reorder_joint_state


def compare(a,b):
    import numpy as np
    from scipy.spatial.transform import Rotation
    delta=np.array(b[:3])-np.array(a[:3])
    ra=Rotation.from_euler('ZYZ',a[3:],degrees=True)
    rb=Rotation.from_euler('ZYZ',b[3:],degrees=True)
    return dict(delta_mm=delta.tolist(),translation_error_mm=float(np.linalg.norm(delta)),
                orientation_error_deg=math.degrees((rb*ra.inv()).magnitude()))


def run(output):
    import os,rclpy
    from sensor_msgs.msg import JointState
    from rclpy.qos import qos_profile_sensor_data
    from rosidl_runtime_py.convert import message_to_ordereddict
    from dsr_msgs2.srv import GetCurrentToolFlangePosx,GetCurrentPosx,GetRobotMode,GetRobotState,GetRobotSystem
    from scipy.spatial.transform import Rotation
    output.mkdir(parents=True,exist_ok=False)
    def save(name,value): (output/name).write_text(json.dumps(value,indent=2,allow_nan=False))
    save('pendant_tcp_evidence.json',dict(source='PENDANT_TCP_SETTINGS',provenance='USER_TRANSCRIBED_CANDIDATE',offset_mm_deg=[0]*6,active_tcp_name=None,current_tcp_pose=None,image_attached=False,confirmation_time_unknown=True))
    save('getter_source_audit.json',dict(source='/home/ubuntu/robot_ws/src/doosan-robot2/dsr_controller2/src/dsr_controller2.cpp',flange=dict(classification='SAFE_READ_ONLY',callback='get_current_tool_flange_posx_cb',underlying='g_stDrState.fCurrentToolPosx',cache_freshness_marker=False,request_ref_ignored=True,pose_kind='FLANGE_ONLY'),posx=dict(classification='READ_ONLY_BUT_STABILITY_UNVERIFIED',callback='get_current_posx_cb',underlying='Drfl->get_current_posx(req->ref)',null_check=True,prior_feedback_risk=True,operator_authorized_controlled_once=True),convention=dict(request_ref=0,reference='BASE',translation='mm',orientation='degrees',rotation='intrinsic ZYZ',project_contract='02_safety/action_contract.py')))
    os.environ['ROS_LOCALHOST_ONLY']='1';rclpy.init();node=rclpy.create_node('zero_offset_tcp_controlled_once')
    gate=JointStateStartup(time.monotonic());rows=[];results={};comparisons={}
    def callback(msg):
        now=time.monotonic();source=msg.header.stamp.sec+msg.header.stamp.nanosec/1e9
        valid=len(msg.name)==6 and set(msg.name)=={f'joint_{i}' for i in range(1,7)} and len(msg.position)==len(msg.velocity)==6 and all(math.isfinite(v) for v in list(msg.position)+list(msg.velocity))
        row=dict(receive=now,source=source,header_age=node.get_clock().now().nanoseconds/1e9-source,valid=valid,names=list(msg.name),position=list(msg.position),velocity=list(msg.velocity))
        gate.sample(row);rows.append(row)
    node.create_subscription(JointState,'/dsr01/joint_states',callback,qos_profile_sensor_data)
    def tick():
        rclpy.spin_once(node,timeout_sec=.005)
        if gate.status(time.monotonic())['fault']:raise RuntimeError('TCP_VALIDATION_INVALID_RUNTIME:'+str(gate.fault))
    def call(name,kind,**fields):
        pre=gate.status(time.monotonic())
        if pre['fault'] or pre['phase']!='RUNTIME_READY':raise RuntimeError('TCP_VALIDATION_INVALID_RUNTIME')
        client=node.create_client(kind,name);started=time.monotonic();sent=False
        try:
            while not client.service_is_ready() and time.monotonic()-started<3:tick()
            if not client.service_is_ready():raise RuntimeError('service_endpoint_timeout')
            request=kind.Request()
            for key,value in fields.items():setattr(request,key,value)
            started=time.monotonic();future=client.call_async(request);sent=True
            while not future.done() and time.monotonic()-started<3:tick()
            if not future.done():raise RuntimeError('service_response_timeout')
            response=message_to_ordereddict(future.result())
            if response.get('success') is not True:raise RuntimeError('service_unsuccessful_response')
            return dict(response=response,latency_s=time.monotonic()-started,pre_jointstate=pre,request_fields=fields,sent_once=sent,request_wall_time=time.time())
        finally:node.destroy_client(client)
    error=None;fk=None;before=None
    try:
        while gate.phase!='RUNTIME_READY':tick()
        before=gate.status(time.monotonic());save('jointstate_before_getters.json',before)
        for key,kind,field in [('mode',GetRobotMode,'robot_mode'),('state',GetRobotState,'robot_state'),('system',GetRobotSystem,'robot_system')]:
            results[key]=call('/dsr01/system/get_robot_'+key,kind)
            if results[key]['response'][field]!=dict(mode=1,state=1,system=0)[key]:raise RuntimeError('hardware_not_REAL_AUTO_STANDBY')
        if any(v!=0 for v in rows[-1]['velocity']):raise RuntimeError('stationary_not_supported_by_zero_joint_velocity')
        row=dict(rows[-1]);save('jointstate_snapshot.json',row)
        fk=A0509FlangeFK('/home/ubuntu/robot_ws/src/doosan-robot2/dsr_description2/urdf/a0509.urdf').compute(reorder_joint_state(row['names'],row['position']))
        fk['position_mm']=[v*1000 for v in fk['position_m']]
        fk['orientation_zyz_deg']=Rotation.from_matrix(fk['rotation_matrix']).as_euler('ZYZ',degrees=True).tolist()
        fk['joint_source_timestamp']=row['source'];fk['joint_receive_timestamp']=row['receive'];save('fk_flange_pose.json',fk)
        for key,name,kind in [('flange','get_current_tool_flange_posx',GetCurrentToolFlangePosx),('posx','get_current_posx',GetCurrentPosx)]:
            result=call('/dsr01/aux_control/'+name,kind,ref=0);results[key]=result
            response=result['response'];pose=response['pos'] if key=='flange' else response['task_pos_info'][0]['data'][:6]
            if len(pose)!=6 or not all(math.isfinite(v) for v in pose):raise RuntimeError('invalid_cartesian_response')
            result.update(pose=pose,reference='BASE_REQUESTED',units='mm/deg',orientation_convention='ZYZ',usable=False)
            deadline=time.monotonic()+10
            while time.monotonic()<deadline:tick()
            result['post_jointstate']=gate.status(time.monotonic());result['usable']=True
            save('controller_flange_pose.json' if key=='flange' else 'controller_current_posx.json',result)
        pose=fk['position_mm']+fk['orientation_zyz_deg']
        comparisons=dict(fk_vs_flange=compare(pose,results['flange']['pose']),fk_vs_posx=compare(pose,results['posx']['pose']),controller_flange_vs_posx=compare(results['flange']['pose'],results['posx']['pose']),acceptance_tolerance=None,active_binding_verified=False)
    except Exception as exc:
        error=str(exc)
        for key in ('flange','posx'):
            if key in results:results[key]['usable']=False;results[key]['discard_reason']=error
    finally:
        for key,filename in [('flange','controller_flange_pose.json'),('posx','controller_current_posx.json')]:save(filename,results.get(key,dict(status='NOT_EXECUTED',reason=error)))
        save('hardware_state.json',{k:results.get(k) for k in ('mode','state','system')})
        after=gate.status(time.monotonic());save('post_getter_jointstate_health.json',dict(before=before,after=after,events=gate.events,sample_count=len(rows),runtime_events=sum(r['phase']=='RUNTIME_READY' and ((r.get('source_gap') or 0)>=.1 or (r.get('receive_gap') or 0)>=.1) for r in rows)))
        save('jointstate_all_samples.json',rows);save('tcp_pose_comparison.json',comparisons)
        classification='TCP_UNVERIFIED' if error else 'TCP_ZERO_OFFSET_SUPPORTED_NOT_VERIFIED'
        summary=dict(classification=classification,tcp_contract_verified=False,error=error,physical_commands=0,next='BLOCKED',active_tcp_name=None,acceptance_tolerance=None)
        save('summary.json',summary)
        node.destroy_node();rclpy.shutdown()
    print(json.dumps(summary,indent=2))


if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('--output',type=Path,required=True)
    run(parser.parse_args().output)
