#!/usr/bin/env python3
"""Subscriber-only live prediction recorder; intentionally has no command API."""
from __future__ import annotations

import argparse, base64, hashlib, io, json, math, sys, time
from pathlib import Path
from urllib.request import Request, urlopen

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, JointState
from PIL import Image as PILImage

sys.path.insert(0, str(Path(__file__).resolve().parent))
from integrated_logger import FsyncJsonlLogger

CANONICAL = tuple(f"joint_{i}" for i in range(1, 7))

class PassiveInputs(Node):
    def __init__(self, camera_topic: str, joint_topic: str):
        super().__init__("phase12_prediction_only_shadow")
        self.image = None; self.image_rx = None
        self.joint = None; self.joint_rx = None
        self.create_subscription(Image, camera_topic, self._image, qos_profile_sensor_data)
        self.create_subscription(JointState, joint_topic, self._joint, qos_profile_sensor_data)
    def _image(self, msg): self.image, self.image_rx = msg, time.monotonic()
    def _joint(self, msg): self.joint, self.joint_rx = msg, time.monotonic()

def stamp(msg): return msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9
def image_to_jpeg(msg):
    if msg.encoding.lower() not in {"bgra8", "rgba8", "bgr8", "rgb8"}:
        raise ValueError(f"unsupported_encoding:{msg.encoding}")
    channels = 4 if "a8" in msg.encoding.lower() else 3
    raw = bytes(msg.data)
    mode = "RGBA" if channels == 4 else "RGB"
    im = PILImage.frombytes(mode, (msg.width, msg.height), raw, "raw", msg.encoding.upper().replace("8", ""), msg.step)
    im = im.convert("RGB"); out = io.BytesIO(); im.save(out, format="JPEG", quality=95)
    return out.getvalue()
def joints(msg):
    index = {n:i for i,n in enumerate(msg.name)}
    if any(n not in index for n in CANONICAL): raise ValueError("missing_joint")
    pos=[float(msg.position[index[n]]) for n in CANONICAL]
    vel=[float(msg.velocity[index[n]]) for n in CANONICAL]
    if not all(math.isfinite(v) for v in pos+vel): raise ValueError("invalid_joint")
    return {"canonical_names":CANONICAL,"position":pos,"velocity":vel,"source_timestamp":stamp(msg),"effort":None,"effort_status":"unsupported"}
def predict(url, jpeg, instruction, timeout):
    payload={"instruction":instruction,"image_jpeg_base64":base64.b64encode(jpeg).decode("ascii")}
    req=Request(url.rstrip("/")+"/predict",data=json.dumps(payload).encode(),headers={"Content-Type":"application/json"},method="POST")
    start=time.monotonic()
    with urlopen(req,timeout=timeout) as response: result=json.loads(response.read())
    return result,time.monotonic()-start

def main():
    p=argparse.ArgumentParser();p.add_argument("--model",choices=("openvla","oft"),required=True)
    p.add_argument("--server-url",required=True);p.add_argument("--output",type=Path,required=True)
    p.add_argument("--duration",type=float,default=15);p.add_argument("--period",type=float,default=1)
    p.add_argument("--timeout",type=float,default=5);p.add_argument("--instruction",default="Pick up the orange cube.")
    p.add_argument("--camera-topic",default="/zed/zed_node/rgb/color/rect/image")
    p.add_argument("--joint-topic",default="/dsr01/joint_states");a=p.parse_args()
    __import__("os").environ["ROS_LOCALHOST_ONLY"] = "1"; rclpy.init(); node=PassiveInputs(a.camera_topic,a.joint_topic); end=time.monotonic()+a.duration; nxt=0.; seen=set(); count=0
    try:
      with FsyncJsonlLogger(a.output) as logger:
       while rclpy.ok() and time.monotonic()<end:
        rclpy.spin_once(node,timeout_sec=.05); now=time.monotonic()
        if now<nxt or node.image is None or node.joint is None: continue
        nxt=now+a.period; jpeg=image_to_jpeg(node.image); digest=hashlib.sha256(bytes(node.image.data)).hexdigest()
        if digest in seen: continue
        seen.add(digest); errors=[]; response=None; latency=None
        try:
            response,latency=predict(a.server_url,jpeg,a.instruction,a.timeout)
            acts=[response.get("action")] if a.model=="openvla" else response.get("actions")
            expected=1 if a.model=="openvla" else 5
            if not isinstance(acts,list) or len(acts)!=expected or any(not isinstance(x,list) or len(x)!=7 or not all(math.isfinite(float(v)) for v in x) for x in acts): raise ValueError("invalid_action_contract")
        except Exception as exc: errors.append(f"{type(exc).__name__}:{exc}"); acts=[]
        logger.append({"classification":"STATIONARY_LIVE_SHADOW_SMOKE_TEST","model":a.model,"instruction":a.instruction,
          "image_sha256":digest,"camera_source_timestamp":stamp(node.image),"camera_receive_monotonic_timestamp":node.image_rx,
          "joint_state":joints(node.joint),"joint_receive_monotonic_timestamp":node.joint_rx,
          "raw_model_response":response,"canonical_predictions":acts,"inference_latency_seconds":latency,
          "prediction_errors":errors,"valid":not errors,"exclusion_reason":";".join(errors) or None,
          "executed_action":None,"robot_delivered_command":None,"command_issued":False,"hold_required":bool(errors),
          "hold_reason":"prediction_error" if errors else None,"publisher_created":False,"service_client_created":False,"action_client_created":False})
        count+=1
    finally: node.destroy_node();rclpy.shutdown()
    print(json.dumps({"status":"COMPLETE","model":a.model,"records":count,"output":str(a.output),"command_issued":False}))
if __name__=="__main__":main()
