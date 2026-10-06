"""Passive TF/robot_description inventory. No getter/client or robot command."""
import json,time,re,xml.etree.ElementTree as ET
from pathlib import Path
import rclpy
from tf2_msgs.msg import TFMessage
from std_msgs.msg import String
from rclpy.qos import QoSProfile,ReliabilityPolicy,DurabilityPolicy
ROOT=Path(__file__).resolve().parent
def main():
    rclpy.init();node=rclpy.create_node('phase12_passive_tcp_tf_audit');frames={};descriptions=[]
    def tf(msg,topic):
        for t in msg.transforms:
            frames[(topic,t.header.frame_id,t.child_frame_id)]=dict(topic=topic,parent=t.header.frame_id,child=t.child_frame_id,
                source_timestamp=dict(sec=t.header.stamp.sec,nanosec=t.header.stamp.nanosec),receive_monotonic=time.monotonic(),
                translation_m=[t.transform.translation.x,t.transform.translation.y,t.transform.translation.z],
                quaternion_xyzw=[t.transform.rotation.x,t.transform.rotation.y,t.transform.rotation.z,t.transform.rotation.w])
    def description(m):
        try:
            root=ET.fromstring(m.data);descriptions.append(dict(robot=root.attrib.get('name'),links=[x.attrib['name'] for x in root.findall('link')],fixed_joints=[dict(j.attrib,parent=j.find('parent').attrib,child=j.find('child').attrib,origin=j.find('origin').attrib if j.find('origin') is not None else None) for j in root.findall('joint') if j.attrib.get('type')=='fixed']))
        except Exception as exc:descriptions.append(dict(error=str(exc)))
    node.create_subscription(TFMessage,'/tf',lambda m:tf(m,'/tf'),10)
    node.create_subscription(TFMessage,'/tf_static',lambda m:tf(m,'/tf_static'),QoSProfile(depth=100,reliability=ReliabilityPolicy.RELIABLE,durability=DurabilityPolicy.TRANSIENT_LOCAL))
    node.create_subscription(String,'/dsr01/robot_description',description,QoSProfile(depth=1,reliability=ReliabilityPolicy.RELIABLE,durability=DurabilityPolicy.TRANSIENT_LOCAL))
    start=time.monotonic()
    while time.monotonic()-start<20:rclpy.spin_once(node,timeout_sec=.02)
    endpoints={}
    for topic in ('/tf','/tf_static','/dsr01/robot_description','/doosan/current_pose'):
        endpoints[topic]=[dict(node_name=e.node_name,node_namespace=e.node_namespace,topic_type=e.topic_type) for e in node.get_publishers_info_by_topic(topic)]
    candidates=[r for r in frames.values() if re.search(r'flange|tool|tcp|gripper',r['child'],re.I)]
    report=dict(status='CURRENT_TCP_UNKNOWN',duration_s=time.monotonic()-start,observed_frames=list(frames.values()),
        tool_frame_candidates=candidates,publisher_endpoints=endpoints,robot_descriptions=descriptions,
        topic_types=node.get_topic_names_and_types(),service_types=node.get_service_names_and_types(),
        active_controller_tcp_verified=False,zero_offset_verified=False,
        caveat='TF/URDF describes a model/display tree; even tool-named frames do not establish current controller active TCP.',getters_called=0,physical_commands=0)
    with (ROOT/'tcp_source_audit.json').open('x') as f:json.dump(report,f,indent=2)
    print(json.dumps(dict(status=report['status'],frame_count=len(frames),tool_frame_candidates=candidates,publisher_endpoints=endpoints,descriptions=descriptions),indent=2))
    node.destroy_node();rclpy.shutdown()
if __name__=='__main__':main()
