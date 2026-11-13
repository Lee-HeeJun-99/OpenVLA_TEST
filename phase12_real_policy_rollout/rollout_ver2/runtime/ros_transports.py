from __future__ import annotations
import time


class RosServiceCaller:
    def __init__(self,node,service_type,name,timeout_s=2.0):
        self.service_type=service_type;self.client=node.create_client(service_type,name);self.timeout_s=float(timeout_s)
    def call(self,**values):
        if not self.client.wait_for_service(timeout_sec=self.timeout_s):
            raise TimeoutError("service_unavailable")
        request=self.service_type.Request()
        for key,value in values.items():setattr(request,key,value)
        future=self.client.call_async(request);deadline=time.monotonic()+self.timeout_s
        while not future.done() and time.monotonic()<deadline:time.sleep(.002)
        if not future.done():raise TimeoutError("service_ack_timeout")
        response=future.result();fields=getattr(response,"get_fields_and_field_types",lambda:{})()
        result={name:getattr(response,name) for name in fields}
        result["success"]=bool(response.success)
        return result


class MoveStopTransport:
    def __init__(self,node,name="/dsr01/motion/move_stop"):
        from dsr_msgs2.srv import MoveStop
        self.caller=RosServiceCaller(node,MoveStop,name)
    def move_stop(self,stop_mode):return self.caller.call(stop_mode=int(stop_mode))


class MoveJointTransport:
    """Thin transport only; authorization and the fixed test profile stay above it."""
    def __init__(self,node,name="/dsr01/motion/move_joint"):
        from dsr_msgs2.srv import MoveJoint
        self.caller=RosServiceCaller(node,MoveJoint,name)
        self.inhibited=False
    def inhibit(self):
        self.inhibited=True
    def move_joint(self,*,pos,vel,acc,time=0.0,radius=0.0,mode=0,
                   blend_type=0,sync_type=1):
        if self.inhibited: raise PermissionError("MOVE_JOINT_PERMANENTLY_INHIBITED")
        return self.caller.call(pos=[float(x) for x in pos],vel=float(vel),
            acc=float(acc),time=float(time),radius=float(radius),mode=int(mode),
            blend_type=int(blend_type),sync_type=int(sync_type))


class ToolDigitalOutputTransport:
    def __init__(self,node,name="/dsr01/io/set_tool_digital_output"):
        from dsr_msgs2.srv import SetToolDigitalOutput,GetToolDigitalOutput
        self.caller=RosServiceCaller(node,SetToolDigitalOutput,name)
        self.reader=RosServiceCaller(node,GetToolDigitalOutput,
                                     "/dsr01/io/get_tool_digital_output")
    def set_tool_digital_output(self,index,value):
        return self.caller.call(index=int(index),value=int(value))
    def get_tool_digital_output(self,index):
        return self.reader.call(index=int(index))


class ToolDigitalOutputReader:
    """Read-only gripper preflight transport; it has no setter client."""
    def __init__(self,node,name="/dsr01/io/get_tool_digital_output"):
        from dsr_msgs2.srv import GetToolDigitalOutput
        self.reader=RosServiceCaller(node,GetToolDigitalOutput,name)
    def get_tool_digital_output(self,index):return self.reader.call(index=int(index))
