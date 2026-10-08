#!/usr/bin/env python3
"""Receive one JointState and compute command-free URDF link_6 FK."""
import argparse, json, os, sys, time
from pathlib import Path

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

sys.path.insert(0, str(Path(__file__).resolve().parent))
from jointstate_flange_fk import A0509FlangeFK, reorder_joint_state


class Once(Node):
    def __init__(self, topic):
        super().__init__('phase12_jointstate_fk_once_subscriber')
        self.msg=None; self.receive_monotonic_ns=None; self.receive_ros_ns=None
        self.create_subscription(JointState,topic,self.cb,10)
    def cb(self,msg):
        if self.msg is None:
            self.msg=msg; self.receive_monotonic_ns=time.monotonic_ns(); self.receive_ros_ns=self.get_clock().now().nanoseconds


def main():
    ap=argparse.ArgumentParser(); ap.add_argument('--topic',default='/dsr01/joint_states')
    ap.add_argument('--timeout',type=float,default=5.0); ap.add_argument('--urdf',required=True); ap.add_argument('--output',required=True)
    a=ap.parse_args(); __import__("os").environ["ROS_LOCALHOST_ONLY"] = "1"; rclpy.init(); n=Once(a.topic); deadline=time.monotonic()+a.timeout
    while n.msg is None and time.monotonic()<deadline: rclpy.spin_once(n,timeout_sec=.1)
    if n.msg is None: raise SystemExit('ERROR: JointState timeout; no output written')
    m=n.msg; fk=A0509FlangeFK(a.urdf).compute(reorder_joint_state(m.name,m.position))
    out={'classification':'SUBSCRIBER_ONLY_COMPUTED_FLANGE_OBSERVATION','topic':a.topic,
         'joint_source_timestamp_ns':m.header.stamp.sec*1_000_000_000+m.header.stamp.nanosec,
         'receive_ros_timestamp_ns':n.receive_ros_ns,'receive_monotonic_timestamp_ns':n.receive_monotonic_ns,
         'observed_joint_order':list(m.name),'fk':fk,
         'publisher_created':False,'service_client_created':False,'action_client_created':False,
         'command_issued':False,'executed_action':None,'robot_delivered_command':None}
    n.destroy_node(); rclpy.shutdown()
    flags=os.O_WRONLY|os.O_CREAT|os.O_EXCL; fd=os.open(a.output,flags,0o644)
    with os.fdopen(fd,'w',encoding='utf-8') as f: json.dump(out,f,indent=2); f.flush(); os.fsync(f.fileno())
    print(json.dumps(out,indent=2))


if __name__=='__main__': main()
