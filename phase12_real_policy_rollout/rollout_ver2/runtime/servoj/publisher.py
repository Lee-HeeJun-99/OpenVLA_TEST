from __future__ import annotations
import numpy as np
from .tracking import validate_servoj_time


class ServojPublisher:
    """Legacy-compatible, disabled-by-default ServojStream transport."""
    def __init__(self,publisher,message_factory,config,gate,*,enabled=False,execution_mode="policy"):
        self.publisher=publisher;self.message_factory=message_factory;self.config=config;self.gate=gate
        self.enabled=bool(enabled);self.inhibited=False;self.events=[]
        if execution_mode not in ("stop_test_servoj","synthetic","policy"):
            raise ValueError("invalid Servoj execution mode")
        self.execution_mode=execution_mode
    @classmethod
    def from_ros_node(cls,node,config,gate,*,enabled=False,execution_mode="policy"):
        from dsr_msgs2.msg import ServojStream
        from rclpy.qos import QoSProfile
        return cls(node.create_publisher(ServojStream,config["servoj_topic"],QoSProfile(depth=1)),
          ServojStream,config,gate,enabled=enabled,execution_mode=execution_mode)
    def inhibit(self):self.inhibited=True
    def publish(self,joint_rad,now):
        if not self.enabled or self.inhibited:raise PermissionError("SERVOJ_DISABLED_OR_INHIBITED")
        self.gate.require(self.execution_mode)
        for key in ("servoj_time_unverified","servoj_velocity_unverified","servoj_acceleration_unverified"):
            if self.config.get(key) is None:raise PermissionError(f"UNVERIFIED_CONFIG:{key}")
        q=np.asarray(joint_rad,dtype=float)
        if q.shape!=(6,) or not np.all(np.isfinite(q)):raise ValueError("six finite joint radians required")
        msg=self.message_factory();msg.pos=np.rad2deg(q).tolist()
        msg.vel=self._six(self.config["servoj_velocity_unverified"],"vel")
        msg.acc=self._six(self.config["servoj_acceleration_unverified"],"acc")
        msg.time=validate_servoj_time(self.config["servoj_time_unverified"],
          self.config.get("servoj_time_min_verified_s",.02),self.config.get("servoj_time_max_verified_s",.10));msg.mode=0
        self.publisher.publish(msg);self.events.append({"time":float(now),"pos_deg":msg.pos,"mode":0})
    @staticmethod
    def _six(value,label):
        values=np.asarray(value,dtype=float)
        if values.ndim==0:values=np.repeat(values,6)
        if values.shape!=(6,) or not np.all(np.isfinite(values)):
            raise ValueError(f"{label} must be one finite value or six finite values")
        return values.tolist()
