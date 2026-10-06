#!/usr/bin/env python3
"""Subscriber-only ZED raw/compressed continuity probe; no publisher or client."""
import argparse
import hashlib
import json
import statistics
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from sensor_msgs.msg import Image, CompressedImage


def summarize(values):
    gaps=[(b-a)/1e9 for a,b in zip(values,values[1:])]
    span=(values[-1]-values[0])/1e9 if len(values)>1 else 0.0
    return {
        'count':len(values),'span_s':span,
        'rate_hz':((len(values)-1)/span if span>0 else None),
        'median_gap_s':(statistics.median(gaps) if gaps else None),
        'max_gap_s':(max(gaps) if gaps else None),
        'gap_ge_50ms':sum(x>=.05 for x in gaps),
        'gap_ge_100ms':sum(x>=.1 for x in gaps),
        'gap_ge_500ms':sum(x>=.5 for x in gaps),
        'duplicate_or_nonmonotonic':sum(x<=0 for x in gaps),
    }


class Probe(Node):
    def __init__(self, raw_topic, compressed_topic, hash_frames):
        super().__init__('phase12_zed_subscriber_only_probe')
        qos=QoSProfile(depth=10,reliability=ReliabilityPolicy.RELIABLE,
                       history=HistoryPolicy.KEEP_LAST,durability=DurabilityPolicy.VOLATILE)
        self.rows={'raw':[],'compressed':[]}; self.hashes={'raw':[],'compressed':[]}
        self.hash_frames=hash_frames
        self.shapes=set(); self.formats=set()
        self.create_subscription(Image,raw_topic,self.raw_cb,qos)
        self.create_subscription(CompressedImage,compressed_topic,self.compressed_cb,qos)
    @staticmethod
    def stamp(msg): return msg.header.stamp.sec*1_000_000_000+msg.header.stamp.nanosec
    def raw_cb(self,msg):
        self.rows['raw'].append((self.stamp(msg),time.monotonic_ns()))
        if self.hash_frames:
            self.hashes['raw'].append(hashlib.sha256(memoryview(msg.data)).hexdigest())
        self.shapes.add((msg.width,msg.height,msg.encoding,msg.step))
    def compressed_cb(self,msg):
        self.rows['compressed'].append((self.stamp(msg),time.monotonic_ns()))
        if self.hash_frames:
            self.hashes['compressed'].append(hashlib.sha256(memoryview(msg.data)).hexdigest())
        self.formats.add(msg.format)


def main():
    ap=argparse.ArgumentParser(); ap.add_argument('--duration',type=float,default=30)
    ap.add_argument('--raw-topic',default='/zed/zed_node/rgb/color/rect/image')
    ap.add_argument('--compressed-topic',default='/zed/zed_node/rgb/color/rect/image/compressed')
    ap.add_argument('--no-hash',action='store_true',help='Disable frame hashing for a low-overhead timing probe')
    ap.add_argument('--output',required=True); args=ap.parse_args()
    rclpy.init(); node=Probe(args.raw_topic,args.compressed_topic,not args.no_hash); end=time.monotonic()+args.duration
    while rclpy.ok() and time.monotonic()<end: rclpy.spin_once(node,timeout_sec=.1)
    out={'classification':'ZED_SUBSCRIBER_ONLY_CONTINUITY_PROBE','duration_s':args.duration,
         'raw_topic':args.raw_topic,'compressed_topic':args.compressed_topic,
         'publisher_created':False,'service_client_created':False,'command_issued':False,
         'frame_hashing_enabled':not args.no_hash,
         'shapes':[list(x) for x in sorted(node.shapes)],'compressed_formats':sorted(node.formats)}
    for key in ('raw','compressed'):
        source=[x[0] for x in node.rows[key]]; receive=[x[1] for x in node.rows[key]]
        out[key]={'source':summarize(source),'receive_monotonic':summarize(receive),
                  'unique_hashes':(len(set(node.hashes[key])) if not args.no_hash else None),
                  'duplicate_hashes':(len(node.hashes[key])-len(set(node.hashes[key])) if not args.no_hash else None)}
    node.destroy_node(); rclpy.shutdown()
    with open(args.output,'x',encoding='utf-8') as f: json.dump(out,f,indent=2)
    print(json.dumps(out,indent=2))


if __name__=='__main__': main()
