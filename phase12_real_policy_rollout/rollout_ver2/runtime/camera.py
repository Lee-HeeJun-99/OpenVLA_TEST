from __future__ import annotations
import time
import numpy as np


class CameraFeedback:
    """Read-only Image subscriber. It never creates a publisher."""
    def __init__(self, stale_s=0.1, clock=time.monotonic):
        self.stale_s=float(stale_s); self.clock=clock
        self.rgb=None; self.received_monotonic=None; self.header_stamp=None

    def callback(self, msg):
        h,w=int(msg.height),int(msg.width)
        channels={"rgb8":3,"bgr8":3,"rgba8":4,"bgra8":4}.get(msg.encoding)
        if channels is None:
            raise ValueError("unsupported camera encoding: "+str(msg.encoding))
        rows=np.frombuffer(msg.data,dtype=np.uint8).reshape(h,int(msg.step))
        image=rows[:,:w*channels].reshape(h,w,channels)
        if msg.encoding in ("bgr8","bgra8"):
            order=[2,1,0]+([3] if channels==4 else [])
            image=image[...,order]
        self.rgb=np.ascontiguousarray(image[...,:3])
        self.received_monotonic=self.clock()
        self.header_stamp=getattr(getattr(msg,"header",None),"stamp",None)

    def latest(self, now=None):
        now=self.clock() if now is None else float(now)
        if self.rgb is None or now-self.received_monotonic > self.stale_s:
            raise RuntimeError("CAMERA_STALE")
        return self.rgb.copy(),self.received_monotonic

    @classmethod
    def from_ros_node(cls,node,topic,stale_s=0.1,clock=time.monotonic):
        from sensor_msgs.msg import Image
        from rclpy.qos import qos_profile_sensor_data
        obj=cls(stale_s,clock)
        obj.subscription=node.create_subscription(Image,topic,obj.callback,qos_profile_sensor_data)
        return obj
