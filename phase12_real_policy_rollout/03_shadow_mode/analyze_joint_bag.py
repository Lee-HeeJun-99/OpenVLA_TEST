#!/usr/bin/env python3
"""Offline rosbag2 JointState integrity analysis."""
import argparse,json,math,statistics
import rosbag2_py
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import JointState
CANONICAL=tuple(f'joint_{i}' for i in range(1,7))
def gaps(values,threshold):return sum((b-a)/1e9>=threshold for a,b in zip(values,values[1:]))
def main():
 p=argparse.ArgumentParser();p.add_argument('bag');a=p.parse_args()
 r=rosbag2_py.SequentialReader();r.open(rosbag2_py.StorageOptions(uri=a.bag,storage_id='sqlite3'),rosbag2_py.ConverterOptions('cdr','cdr'))
 source=[];receive=[];orders=set();badp=badv=missing=effort_unavailable=0
 while r.has_next():
  topic,data,t=r.read_next()
  if topic!='/dsr01/joint_states':continue
  m=deserialize_message(data,JointState);names=list(m.name);orders.add(tuple(names));source.append(m.header.stamp.sec*1_000_000_000+m.header.stamp.nanosec);receive.append(int(t))
  idx={n:i for i,n in enumerate(names)};miss=any(n not in idx for n in CANONICAL);missing+=miss
  if not miss:
   badp+=not(len(m.position)==len(names) and all(math.isfinite(float(m.position[idx[n]])) for n in CANONICAL))
   badv+=not(len(m.velocity)==len(names) and all(math.isfinite(float(m.velocity[idx[n]])) for n in CANONICAL))
  effort_unavailable+=not(len(m.effort)==len(names) and all(math.isfinite(float(x)) for x in m.effort))
 def stats(v):
  d=[(b-a)/1e9 for a,b in zip(v,v[1:])]
  return {'span_seconds':(v[-1]-v[0])/1e9 if len(v)>1 else 0,'rate_hz':(len(v)-1)/((v[-1]-v[0])/1e9) if len(v)>1 and v[-1]>v[0] else 0,'max_gap_seconds':max(d) if d else None,
   'gap_ge_50ms':gaps(v,.05),'gap_ge_100ms':gaps(v,.1),'gap_ge_500ms':gaps(v,.5),'gap_ge_1s':gaps(v,1.0),'duplicate':sum(b==a for a,b in zip(v,v[1:])),'nonmonotonic':sum(b<a for a,b in zip(v,v[1:]))}
 print(json.dumps({'message_count':len(source),'source':stats(source),'receive_bag_clock':stats(receive),'first_source_ns':source[0] if source else None,'last_source_ns':source[-1] if source else None,'first_receive_ns':receive[0] if receive else None,'last_receive_ns':receive[-1] if receive else None,'position_invalid':badp,'velocity_invalid':badv,'missing_joint':missing,'effort_unsupported':effort_unavailable,'joint_orders':sorted(orders)},separators=(',',':')))
if __name__=='__main__':main()
