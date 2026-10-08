#!/usr/bin/env python3
"""Passive 30 s JointState and ROS graph stability monitor; creates no clients."""
from __future__ import annotations
import json,math,statistics,time
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState

SERVICES=('/dsr01/aux_control/get_current_posx','/dsr01/system/get_robot_state','/dsr01/system/get_robot_mode')
NODES=('/dsr01/dsr_controller2','/dsr01/joint_state_broadcaster')
CANONICAL=tuple(f'joint_{i}' for i in range(1,7))
class Monitor(Node):
 def __init__(self,duration):
  super().__init__('phase12_passive_state_stability_monitor');self.duration=duration;self.samples=[];self.polls=[];self.last_poll=0.0
  self.create_subscription(JointState,'/dsr01/joint_states',self.on_joint,qos_profile_sensor_data)
 def on_joint(self,msg):
  recv=time.monotonic_ns();stamp=msg.header.stamp.sec*1_000_000_000+msg.header.stamp.nanosec
  names=list(msg.name);lookup={n:i for i,n in enumerate(names)};missing=[n for n in CANONICAL if n not in lookup]
  pos=[] if missing else [float(msg.position[lookup[n]]) for n in CANONICAL]
  vel=[] if missing or len(msg.velocity)!=len(names) else [float(msg.velocity[lookup[n]]) for n in CANONICAL]
  effort_ok=len(msg.effort)==len(names) and all(math.isfinite(float(x)) for x in msg.effort)
  self.samples.append({'source_stamp_ns':stamp,'receive_monotonic_ns':recv,'names':names,'missing':missing,
   'position_finite':len(pos)==6 and all(math.isfinite(x) for x in pos),'velocity_finite':len(vel)==6 and all(math.isfinite(x) for x in vel),
   'effort_status':'available' if effort_ok else 'unsupported_or_not_available'})
 def poll(self):
  services={n for n,_ in self.get_service_names_and_types()};nodes={'/'+ns.strip('/')+'/'+name if ns!='/' else '/'+name for name,ns in self.get_node_names_and_namespaces()}
  pubs=len(self.get_publishers_info_by_topic('/dsr01/joint_states'))
  self.polls.append({'monotonic_ns':time.monotonic_ns(),'services':{s:s in services for s in SERVICES},'nodes':{n:n in nodes for n in NODES},'joint_publisher_count':pubs})
 def run(self):
  start=time.monotonic();self.poll()
  while time.monotonic()-start<self.duration:
   rclpy.spin_once(self,timeout_sec=.05)
   if time.monotonic()-self.last_poll>=1:self.poll();self.last_poll=time.monotonic()
  stamps=[s['source_stamp_ns'] for s in self.samples];intervals=[(b-a)/1e9 for a,b in zip(stamps,stamps[1:]) if b>a]
  nonmono=sum(b<a for a,b in zip(stamps,stamps[1:]));dup=sum(b==a for a,b in zip(stamps,stamps[1:]))
  all_up=bool(self.polls) and all(all(p['services'].values()) and all(p['nodes'].values()) and p['joint_publisher_count']>0 for p in self.polls)
  valid=bool(self.samples) and not nonmono and not dup and all(not s['missing'] and s['position_finite'] and s['velocity_finite'] for s in self.samples)
  elapsed=time.monotonic()-start; span=(stamps[-1]-stamps[0])/1e9 if len(stamps)>1 else 0.0
  return {'classification':'PASSIVE_30_SECOND_STABILITY','duration_seconds':elapsed,'sample_count':len(self.samples),
   'average_rate_hz':(len(self.samples)-1)/span if span>0 else 0,'message_timestamp_span_seconds':span,'duration_coverage_ratio':span/elapsed if elapsed else 0,
   'interval_min_s':min(intervals) if intervals else None,'interval_max_s':max(intervals) if intervals else None,
   'interval_std_s':statistics.pstdev(intervals) if intervals else None,'first_timestamp_ns':stamps[0] if stamps else None,
   'last_timestamp_ns':stamps[-1] if stamps else None,'nonmonotonic_count':nonmono,'duplicate_count':dup,
   'nonfinite_position_count':sum(not s['position_finite'] for s in self.samples),'nonfinite_velocity_count':sum(not s['velocity_finite'] for s in self.samples),
   'missing_joint_count':sum(bool(s['missing']) for s in self.samples),'observed_joint_orders':sorted({tuple(s['names']) for s in self.samples}),
   'effort_unsupported_count':sum(s['effort_status']!='available' for s in self.samples),'poll_count':len(self.polls),
   'service_down_polls':{x:sum(not p['services'][x] for p in self.polls) for x in SERVICES},
   'node_down_polls':{x:sum(not p['nodes'][x] for p in self.polls) for x in NODES},
   'publisher_zero_polls':sum(p['joint_publisher_count']==0 for p in self.polls),'data_valid':valid,'graph_all_up':all_up,
   'pass':valid and all_up and span/elapsed>=.95 and 90<=((len(self.samples)-1)/span if span>0 else 0)<=110,
   'command_issued':False}
def main():
 __import__("os").environ["ROS_LOCALHOST_ONLY"] = "1"; rclpy.init();m=Monitor(30.0)
 try:print(json.dumps(m.run(),separators=(',',':')))
 finally:m.destroy_node();rclpy.shutdown()
if __name__=='__main__':main()
