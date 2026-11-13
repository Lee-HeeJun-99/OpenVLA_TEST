from __future__ import annotations
import threading
import time
import numpy as np


class CameraFeedback:
    """Read-only Image subscriber; callback only retains the newest raw message."""
    def __init__(self, gap_log_threshold_s=0.1, clock=time.monotonic):
        self.gap_log_threshold_s=float(gap_log_threshold_s); self.clock=clock
        self._lock=threading.Lock();self._first_frame=threading.Event()
        self.raw_message=None;self.received_monotonic=None
        self.previous_received_monotonic=None;self.header_stamp=None;self.header_stamp_ns=None
        self.receive_count=0;self.receive_intervals_s=[]

    @property
    def rgb(self):
        """Compatibility readiness indicator; decoding is intentionally deferred."""
        return None if self.raw_message is None else True

    def callback(self, msg):
        received=self.clock()
        with self._lock:
            self.previous_received_monotonic=self.received_monotonic
            self.raw_message=msg;self.received_monotonic=received
            self.header_stamp=getattr(getattr(msg,"header",None),"stamp",None)
            self.header_stamp_ns=self._stamp_ns(self.header_stamp)
            if self.previous_received_monotonic is not None:
                self.receive_intervals_s.append(received-self.previous_received_monotonic)
            self.receive_count+=1
        self._first_frame.set()

    def wait_for_first_frame(self,timeout_s=5.0):
        if not self._first_frame.wait(float(timeout_s)):
            raise RuntimeError("CAMERA_NO_FIRST_FRAME:timeout_s="+str(float(timeout_s)))
        return self.received_monotonic

    @staticmethod
    def _stamp_ns(stamp):
        if stamp is None:return None
        return int(stamp.sec)*1_000_000_000+int(stamp.nanosec)

    @staticmethod
    def _decode_rgb(msg):
        h,w=int(msg.height),int(msg.width)
        channels={"rgb8":3,"bgr8":3,"rgba8":4,"bgra8":4}.get(msg.encoding)
        if channels is None:
            raise ValueError("unsupported camera encoding: "+str(msg.encoding))
        rows=np.frombuffer(msg.data,dtype=np.uint8).reshape(h,int(msg.step))
        image=rows[:,:w*channels].reshape(h,w,channels)
        if msg.encoding in ("bgr8","bgra8"):
            order=[2,1,0]+([3] if channels==4 else [])
            image=image[...,order]
        return np.ascontiguousarray(image[...,:3])

    def diagnostics(self,now=None):
        now=self.clock() if now is None else float(now)
        with self._lock:
            received=self.received_monotonic;previous=self.previous_received_monotonic
            count=self.receive_count;header_ns=self.header_stamp_ns
            intervals=list(self.receive_intervals_s)
        return {"last_received_monotonic":received,
                "previous_receive_interval_s":None if previous is None or received is None else received-previous,
                "last_header_stamp_ns":header_ns,
                "age_s":None if received is None else now-received,"receive_count":count,
                "gap_over_0_1s_count":sum(value>self.gap_log_threshold_s for value in intervals),
                "gap_over_1s_count":sum(value>1.0 for value in intervals),
                "max_receive_interval_s":max(intervals) if intervals else None}

    def latest(self, now=None):
        now=self.clock() if now is None else float(now)
        with self._lock:
            msg=self.raw_message;received=self.received_monotonic
            previous=self.previous_received_monotonic;count=self.receive_count
        if msg is None:raise RuntimeError("CAMERA_NO_FIRST_FRAME:receive_count=0")
        return self._decode_rgb(msg),received

    def frame_after(self,settle_host_monotonic,settle_ros_ns):
        """Return a new post-settle frame, or None. Camera gaps are log-only."""
        with self._lock:
            msg=self.raw_message;received=self.received_monotonic
            header_ns=self.header_stamp_ns;count=self.receive_count
        if (msg is None or received <= float(settle_host_monotonic) or header_ns is None or
                header_ns <= int(settle_ros_ns)):
            return None
        return {"rgb":self._decode_rgb(msg),"host_received_monotonic":received,
                "header_stamp_ns":header_ns,"receive_count":count,
                "wait_s":self.clock()-float(settle_host_monotonic)}

    def poll_frame_after(self,settle_host_monotonic,settle_ros_ns,timeout_s,now=None):
        frame=self.frame_after(settle_host_monotonic,settle_ros_ns)
        if frame is not None:return "FRAME",frame
        now=self.clock() if now is None else float(now)
        if now-float(settle_host_monotonic)>=float(timeout_s):
            return "CAMERA_NO_FRAME_TIMEOUT",None
        return "WAITING",None

    @classmethod
    def from_ros_node(cls,node,topic,gap_log_threshold_s=0.1,clock=time.monotonic,callback_group=None):
        from sensor_msgs.msg import Image
        from rclpy.qos import DurabilityPolicy,HistoryPolicy,QoSProfile,ReliabilityPolicy
        obj=cls(gap_log_threshold_s,clock)
        qos=QoSProfile(history=HistoryPolicy.KEEP_LAST,depth=1,
                       reliability=ReliabilityPolicy.RELIABLE,durability=DurabilityPolicy.VOLATILE)
        obj.subscription=node.create_subscription(Image,topic,obj.callback,qos,callback_group=callback_group)
        return obj
