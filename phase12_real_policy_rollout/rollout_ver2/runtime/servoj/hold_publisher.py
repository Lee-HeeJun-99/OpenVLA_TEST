"""Hold-only ServoJ publisher; construction itself sends nothing."""
from __future__ import annotations
import numpy as np

class HoldServojPublisher:
    def __init__(self,publisher,message_factory,config):
        self.publisher=publisher;self.message_factory=message_factory;self.config=config;self.inhibited=False
    @classmethod
    def from_ros_node(cls,node,config):
        from dsr_msgs2.msg import ServojStream
        from rclpy.qos import QoSProfile,ReliabilityPolicy,DurabilityPolicy
        qos=QoSProfile(depth=1,reliability=ReliabilityPolicy.RELIABLE,durability=DurabilityPolicy.VOLATILE)
        return cls(node.create_publisher(ServojStream,config["servoj_topic"],qos),ServojStream,config)
    def inhibit(self):self.inhibited=True
    def publish(self,joint_rad,now):
        if self.inhibited:raise PermissionError("SERVOJ_HOLD_PERMANENTLY_INHIBITED")
        q=np.asarray(joint_rad,float)
        if q.shape!=(6,) or not np.all(np.isfinite(q)):raise ValueError("six finite J1..J6 radians required")
        msg=self.message_factory();msg.pos=np.rad2deg(q).tolist()
        msg.vel=[float(self.config["servoj_hold_velocity_deg_s"])]*6
        msg.acc=[float(self.config["servoj_hold_acceleration_deg_s2"])]*6
        msg.time=float(self.config["servoj_hold_time_s"]);msg.mode=0;self.publisher.publish(msg)
