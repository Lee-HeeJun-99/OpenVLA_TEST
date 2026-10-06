"""Passive audit: subscriptions/graph only; never creates a service client."""
import datetime, hashlib, json, math, re, subprocess, time
from pathlib import Path
import rclpy
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import JointState, Image
from std_msgs.msg import String
from tf2_msgs.msg import TFMessage
from rosidl_runtime_py.utilities import get_message
from rosidl_runtime_py.convert import message_to_ordereddict
import dsr_msgs2.srv as services

ROOT = Path(__file__).resolve().parent
SOURCE = Path('/home/ubuntu/robot_ws/src/doosan-robot2')

def main():
    out = ROOT/'real_trials'/datetime.datetime.now().strftime('%Y%m%d_%H%M%S_automatic_hardware_state_audit')
    out.mkdir(parents=True, exist_ok=False)
    def save(name, value):
        with (out/name).open('x') as f:
            json.dump(value, f, indent=2)
    graph=[]
    for command in [['ros2','node','list'],['ros2','topic','list','-t'],['ros2','service','list','-t'],['ros2','action','list','-t']]:
        try:
            p=subprocess.run(command,capture_output=True,text=True,timeout=12)
            graph.append({'command':command,'exit_code':p.returncode,'stdout':p.stdout,'stderr':p.stderr})
        except subprocess.TimeoutExpired:
            graph.append({'command':command,'timeout':True})
    (out/'ros_graph_snapshot.txt').write_text(json.dumps(graph,indent=2))
    cpp=(SOURCE/'dsr_controller2/src/dsr_controller2.cpp').read_text()
    header=(SOURCE/'dsr_common2/include/DRFLEx.h').read_text()
    entries=[]
    for name,cb,drfl in [('GetCurrentTcp','get_current_tcp_cb','get_tcp'),('GetCurrentTool','get_current_tool_cb','get_tool'),('GetCurrentPosx','get_current_posx_cb','get_current_posx'),('GetCurrentPose','get_current_pose_cb','get_current_pose'),('GetRobotMode','get_robot_mode_cb','get_robot_mode'),('GetRobotState','get_robot_state_cb','get_robot_state'),('GetRobotSystem','get_robot_system_cb','get_robot_system'),('GetControlMode','get_control_mode_cb','get_control_mode')]:
        srv=getattr(services,name)
        at=cpp.find('auto '+cb)
        end=cpp.find('\n};',at)
        creation=[line for line in cpp.splitlines() if 'create_service<' in line and '::'+name+'>' in line]
        entries.append(dict(type='dsr_msgs2/srv/'+name,request=srv.Request.get_fields_and_field_types(),response=srv.Response.get_fields_and_field_types(),source_srv=str(next(SOURCE.glob('dsr_msgs2/srv/**/'+name+'.srv'))),callback_file=str(SOURCE/'dsr_controller2/src/dsr_controller2.cpp'),callback_line=cpp[:at].count('\n')+1,callback=cpp[at:end+3],service_registration=creation,drfl=drfl,wrapper=[line.strip() for line in header.splitlines() if re.search(r'\b'+drfl+r'\(',line)],classification='READ_ONLY_BUT_LIVE_STABILITY_UNVERIFIED',called=False,reason='Visible callback is getter-only; vendor implementation/stability not proven. Posx has prior feedback-loss correlation.'))
    save('read_only_interface_classification.json',dict(getters=entries,forbidden_classes=['STATE_CHANGING','UNKNOWN'],setter_classification='STATE_CHANGING',getter_requests=0))
    (out/'doosan_getter_source_audit.md').write_text('# Getter source audit\n\nNo getter was invoked: none meets SAFE_READ_ONLY live-stability evidence.\nGetCurrentTcp/GetCurrentTool return only string info; success=true is hardcoded, not an independent SDK success check.\n\n'+ '\n\n'.join('## '+e['type']+'\n\n```json\n'+json.dumps(e,indent=2)+'\n```' for e in entries))
    rclpy.init(); node=rclpy.create_node('phase12_automatic_passive_hardware_audit')
    joints=[]; images=[]; frames={}; descriptions=[]; states={}; logs=[]
    canonical=[f'joint_{i}' for i in range(1,7)]
    def js(m):
        names=list(m.name); indices={n:i for i,n in enumerate(names)}
        missing=any(n not in indices for n in canonical)
        invalid=missing or any(len(v)!=len(names) or not all(math.isfinite(v[indices[n]]) for n in canonical) for v in (m.position,m.velocity))
        joints.append(dict(receive=time.monotonic(),receive_ros_ns=node.get_clock().now().nanoseconds,source_ns=m.header.stamp.sec*10**9+m.header.stamp.nanosec,names=names,position=list(m.position),velocity=list(m.velocity),invalid=invalid,missing=missing))
    def image(m):
        images.append(dict(receive=time.monotonic(),source_ns=m.header.stamp.sec*10**9+m.header.stamp.nanosec,width=m.width,height=m.height,encoding=m.encoding,hash=hashlib.sha256(bytes(m.data)).hexdigest()))
    def tf(m):
        for t in m.transforms:
            frames[t.header.frame_id+'->'+t.child_frame_id]=message_to_ordereddict(t)
    qos=QoSProfile(depth=1000,reliability=ReliabilityPolicy.BEST_EFFORT,durability=DurabilityPolicy.VOLATILE)
    node.create_subscription(JointState,'/dsr01/joint_states',js,qos)
    node.create_subscription(Image,'/zed/zed_node/rgb/color/rect/image',image,qos)
    node.create_subscription(TFMessage,'/tf',tf,qos)
    node.create_subscription(TFMessage,'/tf_static',tf,QoSProfile(depth=100,reliability=ReliabilityPolicy.RELIABLE,durability=DurabilityPolicy.TRANSIENT_LOCAL))
    node.create_subscription(String,'/dsr01/robot_description',lambda m:descriptions.append(m.data),QoSProfile(depth=1,reliability=ReliabilityPolicy.RELIABLE,durability=DurabilityPolicy.TRANSIENT_LOCAL))
    # Subscribe only to real types present in graph; no invented status topic.
    for topic,types in node.get_topic_names_and_types():
        if any(t.startswith('dsr_msgs2/msg/') for t in types):
            node.create_subscription(get_message(types[0]),topic,lambda m,t=topic:states.setdefault(t,[]).append(dict(receive=time.monotonic(),message=message_to_ordereddict(m))),qos)
    from rcl_interfaces.msg import Log
    node.create_subscription(Log,'/rosout',lambda m:logs.append(message_to_ordereddict(m)) if 'dsr' in m.name or 'controller' in m.name else None,qos)
    start=time.monotonic()
    while time.monotonic()-start<20:
        rclpy.spin_once(node,timeout_sec=.01)
    now=time.monotonic()
    endpoints={t:[dict(node=e.node_namespace+'/'+e.node_name,type=e.topic_type,qos=str(e.qos_profile)) for e in node.get_publishers_info_by_topic(t)] for t in ['/dsr01/joint_states','/dsr01/robot_state','/dsr01/robot_disconnection','/dsr01/error','/doosan/current_pose','/tf','/tf_static']}
    direct_services=node.get_service_names_and_types()
    def stats(rows):
        r=[b['receive']-a['receive'] for a,b in zip(rows,rows[1:])];s=[(b['source_ns']-a['source_ns'])/1e9 for a,b in zip(rows,rows[1:])]
        return dict(count=len(rows),first_receive_delay=rows[0]['receive']-start if rows else None,receive_rate=(len(rows)-1)/(rows[-1]['receive']-rows[0]['receive']) if len(rows)>1 else 0,latest_age=now-rows[-1]['receive'] if rows else None,max_source_gap=max(s,default=None),max_receive_gap=max(r,default=None),source_duplicate=sum(v==0 for v in s),source_regression=sum(v<0 for v in s),gap_ge_100ms=sum(v>=.1 for v in s),invalid_count=sum(row.get('invalid',False) for row in rows),clock_domains=dict(source='ROS_HEADER',receive='HOST_MONOTONIC'))
    j=stats(joints);j['observed_orders']=list({tuple(row['names']) for row in joints});j['effort']='UNSUPPORTED_NOT_VALIDATED';j['status']='PASS' if len(joints)>1 and now-joints[0]['receive']>=19 and j['invalid_count']==0 and j['source_duplicate']==j['source_regression']==0 and j['max_source_gap']<.1 and j['max_receive_gap']<.1 and j['latest_age']<.5 else 'FAIL'
    c=stats(images);c['latest_metadata']=images[-1] if images else None;c['status']='PASS' if len(images)>1 and now-images[0]['receive']>=19 and c['latest_age']<.5 and c['max_receive_gap']<.5 and c['source_regression']==0 else 'FAIL';c['model_selected_frame_age_validated']=False
    save('jointstate_status.json',j);save('camera_status.json',c);save('raw_observation.json',dict(joints=joints,camera=images,driver_logs=logs,states=states,endpoints=endpoints,direct_services=direct_services))
    save('active_tcp_tool.json',dict(active_tcp_name='UNKNOWN',active_tool_name='UNKNOWN',requests=0,service_presence={n:any(t==n for t,_ in direct_services) for n in ['/dsr01/tcp/get_current_tcp','/dsr01/tool/get_current_tool']},reason='No getter with proven live stability; graph presence is not success evidence.'))
    save('tcp_offset_search.json',dict(status='NOT_FOUND',active_name_verified=False,frames=frames,robot_descriptions=descriptions,reason='TF and URDF cannot establish currently active controller TCP; historical values not reused.'))
    save('cartesian_pose_sources.json',dict(actual_tcp='NOT_FOUND',fk_flange_only=True,tool_offset_verified=False,classification='TCP_FLANGE_ONLY',reason='Doosan link_6 FK is available, not an active TCP transform.'))
    save('robot_state_sources.json',dict(current_values={k:'UNKNOWN' for k in ['mode','servo','protective_stop','emergency_stop','authority','motion_state']},messages=states,endpoints=endpoints,connected='PROCESS_PRESENT_NOT_CONTROLLER_STATE_PROOF',manual_confirmations=['operator_present','physical_estop_accessible','workspace_physically_clear'],provenance='UNKNOWN'))
    report=f'# Automatic hardware state audit\n\nJointState: {j["status"]}, {j["count"]} messages, {j["receive_rate"]:.3f} Hz, max source gap {j["max_source_gap"]}, receive gap {j["max_receive_gap"]}.\n\nCamera: {c["status"]}, {c["count"]} frames. This passive check does not validate inference selected-frame age.\n\nTCP: TCP_FLANGE_ONLY; active name/tool/offset/current controller Cartesian pose UNKNOWN.\n\nRobot mode/servo/protection/emergency/authority remain UNKNOWN: no fresh complete state signal verified. Event-topic silence does not prove safety.\n\nManual items: MANUAL_CONFIRMATION_REQUIRED (operator, workspace clear, E-stop access).\n\nMinimum-motion: BLOCKED. Physical commands: 0; getter requests: 0; publisher/client/action capability: none.\n\nSee raw_observation.json for current endpoints, direct discovery, messages and driver rosout collected during this window. Historical source/config information is not current hardware evidence.\n'
    (out/'AUTOMATIC_HARDWARE_STATE_REPORT.md').write_text(report)
    node.destroy_node();rclpy.shutdown()
    print(str(out));print(report)

if __name__=='__main__':main()
