"""Time-exploration-only ServoJ publisher."""
from __future__ import annotations
import numpy as np
from .tracking import validate_servoj_time

class TimeExplorePublisher:
    def __init__(self,publisher,message_factory,config):self.publisher=publisher;self.factory=message_factory;self.config=config;self.inhibited=False
    @classmethod
    def from_ros_node(cls,node,config):
        from dsr_msgs2.msg import ServojStream
        from rclpy.qos import QoSProfile,ReliabilityPolicy,DurabilityPolicy
        qos=QoSProfile(depth=1,reliability=ReliabilityPolicy.RELIABLE,durability=DurabilityPolicy.VOLATILE)
        return cls(node.create_publisher(ServojStream,config["servoj_topic"],qos),ServojStream,config)
    def inhibit(self):self.inhibited=True
    def publish_with_time(self,joint_rad,time_param,now):
        if self.inhibited:raise PermissionError("SERVOJ_TIME_EXPLORE_PERMANENTLY_INHIBITED")
        q=np.asarray(joint_rad,float)
        if q.shape!=(6,) or not np.all(np.isfinite(q)):raise ValueError("six finite J1..J6 radians required")
        settings=self.config["servoj_time_explore"]
        validate_servoj_time(time_param,self.config.get("servoj_time_min_verified_s",.02),
                             self.config.get("servoj_time_max_verified_s",.10))
        msg=self.factory();msg.pos=np.rad2deg(q).tolist();msg.vel=[float(settings["velocity_deg_s"])]*6
        msg.acc=[float(settings["acceleration_deg_s2"])]*6
        msg.time=float(time_param);msg.mode=0;self.publisher.publish(msg)
