from __future__ import annotations
from dataclasses import dataclass
import json
from pathlib import Path
import threading
import time
import numpy as np
from scipy.spatial.transform import Rotation
from runtime import robot_state as rs

JOINT_NAMES=tuple(f"joint_{index}" for index in range(1,7))

def _stamp_seconds(header):
    stamp=getattr(header,"stamp",None)
    return None if stamp is None else float(stamp.sec)+float(stamp.nanosec)*1e-9

def _rows(value):return [list(getattr(row,"data",row)) for row in value]

@dataclass(frozen=True)
class FeedbackSnapshot:
    received_monotonic:float
    header_stamp_s:float|None
    joint_rad:np.ndarray
    joint_velocity_rad_s:np.ndarray
    tool0_real_pose:object
    robot_state:int|None
    robot_mode:int|None
    robot_system:int|None
    robot_state_received_monotonic:float|None
    robot_state_response_latency_s:float|None
    joint_state_received_monotonic:float|None
    @property
    def brake_state(self):return ()
    def log_record(self):
        return {"frame":"tool0_real","received_monotonic":self.received_monotonic,
          "joint_state_received_monotonic":self.joint_state_received_monotonic,
          "joint_state_header_stamp_s":self.header_stamp_s,
          "joint_position_rad":self.joint_rad.tolist(),
          "joint_velocity_rad_s":self.joint_velocity_rad_s.tolist(),
          "tool0_real_position_m":self.tool0_real_pose.position_m.tolist(),
          "tool0_real_rotation_matrix":self.tool0_real_pose.rotation.tolist(),
          "robot_state":self.robot_state,"robot_mode":self.robot_mode,
          "robot_system":self.robot_system,
          "robot_state_received_monotonic":self.robot_state_received_monotonic,
          "robot_state_response_latency_s":self.robot_state_response_latency_s}

class RobotFeedback:
    """Read-only JointState feedback and get_* polling; creates no command objects."""
    def __init__(self,kinematics,tool_transform,stale_s=.1,clock=time.monotonic):
        self.kin=kinematics;self.tool=tool_transform;self.stale_s=float(stale_s);self.clock=clock
        self.lock=threading.RLock();self.condition=threading.Condition(self.lock)
        self._joint=None;self._joint_previous_time=None;self._joint_messages=0;self._joint_fresh=False
        self._state=None;self._mode=None;self._system=None;self._faults=[]
        self.subscriptions=[];self.timers=[];self.clients={};self._inflight=set()
        self.poll_events=[];self.flange_comparisons=[];self.posj_comparisons=[]
        self.joint_events=[]
        self._flange_cycle=0;self._flange_pending={}

    @classmethod
    def from_ros_node(cls,node,kinematics,tool_transform,config,clock=time.monotonic):
        from sensor_msgs.msg import JointState
        from dsr_msgs2.msg import RobotError,RobotDisconnection
        from dsr_msgs2.srv import (GetRobotState,GetRobotMode,GetRobotSystem,
          GetCurrentPosj,GetCurrentToolFlangePosx,GetCurrentRotm)
        from rclpy.qos import QoSProfile,ReliabilityPolicy,DurabilityPolicy,HistoryPolicy
        obj=cls(kinematics,tool_transform,config["feedback_stale_s_candidate"],clock)
        qos=QoSProfile(depth=10,reliability=ReliabilityPolicy.RELIABLE,
          durability=DurabilityPolicy.TRANSIENT_LOCAL,history=HistoryPolicy.KEEP_LAST)
        obj.subscriptions.append(node.create_subscription(
          JointState,config["joint_states_topic"],obj.joint_state_callback,qos))
        event_qos=QoSProfile(depth=10,reliability=ReliabilityPolicy.RELIABLE)
        obj.subscriptions.append(node.create_subscription(
          RobotError,config["error_topic"],obj.error_callback,event_qos))
        obj.subscriptions.append(node.create_subscription(
          RobotDisconnection,config["robot_disconnection_topic"],obj.disconnection_callback,event_qos))
        specs=(("state",GetRobotState,config["get_robot_state_service"]),
          ("mode",GetRobotMode,config["get_robot_mode_service"]),
          ("system",GetRobotSystem,config["get_robot_system_service"]),
          ("posj",GetCurrentPosj,config["get_current_posj_service"]),
          ("flange",GetCurrentToolFlangePosx,config["get_flange_posx_service"]),
          ("rotm",GetCurrentRotm,config["get_current_rotm_service"]))
        obj.clients={key:node.create_client(kind,name) for key,kind,name in specs}
        obj.timers.append(node.create_timer(1/float(config["robot_state_poll_hz_candidate"]),obj.poll_robot_state))
        obj.timers.append(node.create_timer(1.0,obj.poll_preflight_identity))
        obj.timers.append(node.create_timer(1/float(config["flange_crosscheck_poll_hz_candidate"]),obj.poll_flange_crosscheck))
        return obj

    def _fault(self,code,detail=None):
        with self.lock:self._faults.append({"time":self.clock(),"code":code,"detail":detail})

    def joint_state_callback(self,msg):
        now=self.clock();names=list(msg.name)
        if len(names)!=len(set(names)):
            self._fault("JOINT_STATE_DUPLICATE_NAME",names);return
        missing=[name for name in JOINT_NAMES if name not in names]
        if missing:self._fault("JOINT_STATE_MISSING_NAME",missing);return
        mapping={name:index for index,name in enumerate(names)}
        try:
            q=np.asarray([msg.position[mapping[name]] for name in JOINT_NAMES],float)
            v=np.asarray([msg.velocity[mapping[name]] for name in JOINT_NAMES],float)
        except (IndexError,TypeError) as error:
            self._fault("JOINT_STATE_ARRAY_LENGTH",str(error));return
        if not np.all(np.isfinite(q)) or not np.all(np.isfinite(v)):
            self._fault("JOINT_STATE_NONFINITE");return
        header_stamp=_stamp_seconds(getattr(msg,"header",None))
        with self.lock:
            self._joint_messages+=1
            if self._joint_messages>=2:self._joint_fresh=True
            self._joint_previous_time=None if self._joint is None else self._joint[0]
            self._joint=(now,header_stamp,q,v,tuple(names))
            self.joint_events.append({"host_monotonic":now,"header_stamp_s":header_stamp,
              "received_names":names,"normalized_joint_rad":q.tolist(),
              "normalized_velocity_rad_s":v.tolist(),"freshness_eligible":self._joint_fresh})
            self.condition.notify_all()

    def error_callback(self,msg):
        self._fault("ROBOT_ERROR_TOPIC",{key:getattr(msg,key,None) for key in
          ("level","group","code","msg1","msg2","msg3")})
    def disconnection_callback(self,msg):
        self._fault("ROBOT_DISCONNECTION_TOPIC",{key:getattr(msg,key,None) for key in
          ("level","group","code","msg1","msg2","msg3")})

    def _call(self,key,request,on_success):
        with self.lock:
            if key in self._inflight:return
            self._inflight.add(key)
        client=self.clients.get(key)
        if client is None:
            with self.lock:self._inflight.discard(key)
            return
        requested=self.clock()
        if not client.service_is_ready():
            with self.lock:
                self._inflight.discard(key);self.poll_events.append(
                  {"service":key,"requested":requested,"success":False,"error":"SERVICE_NOT_READY"})
            return
        future=client.call_async(request)
        def done(result_future):
            completed=self.clock()
            with self.lock:self._inflight.discard(key)
            event={"service":key,"requested":requested,"completed":completed,"latency_s":completed-requested}
            try:
                response=result_future.result()
                if response is None or not bool(getattr(response,"success",False)):
                    raise RuntimeError("SERVICE_RESPONSE_UNSUCCESSFUL")
                on_success(response,requested,completed);event["success"]=True
            except Exception as error:event.update(success=False,error=str(error))
            with self.lock:self.poll_events.append(event)
        future.add_done_callback(done)

    def poll_robot_state(self):
        client=self.clients.get("state")
        if client:self._call("state",client.srv_type.Request(),self._state_response)
    def _state_response(self,response,requested,completed):
        with self.lock:
            self._state=(completed,int(response.robot_state),completed-requested)
            self.condition.notify_all()

    def wait_for_samples_after(self,start_monotonic,timeout_s=2.0):
        deadline=self.clock()+float(timeout_s)
        with self.condition:
            while True:
                joint_ok=self._joint is not None and self._joint[0]>start_monotonic
                state_ok=self._state is not None and self._state[0]>start_monotonic
                if joint_ok and state_ok:return
                remaining=deadline-self.clock()
                if remaining<=0:
                    missing=[]
                    if not joint_ok:missing.append("joint_states")
                    if not state_ok:missing.append("robot_state")
                    raise RuntimeError("FEEDBACK_NO_FIRST_SAMPLE:"+",".join(missing))
                self.condition.wait(min(remaining,.05))

    def diagnostics(self,now=None):
        now=self.clock() if now is None else float(now)
        with self.lock:
            joint=self._joint;previous=self._joint_previous_time;count=self._joint_messages
            state=self._state;events=list(self.poll_events)
        state_events=[row for row in events if row.get("service")=="state"]
        state_ok=[row for row in state_events if row.get("success")]
        last=None if joint is None else joint[0]
        return {"joint_state_age_s":None if last is None else now-last,
          "joint_state_last_received_monotonic":last,
          "joint_state_previous_receive_interval_s":None if previous is None or last is None else last-previous,
          "joint_state_receive_count":count,
          "robot_state_age_s":None if state is None else now-state[0],
          "robot_state_poll_attempts":len(state_events),
          "robot_state_poll_successes":len(state_ok)}

    def poll_preflight_identity(self):
        for key,field in (("mode","robot_mode"),("system","robot_system")):
            client=self.clients.get(key)
            if not client:continue
            def accept(response,requested,completed,key=key,field=field):
                with self.lock:setattr(self,"_"+key,(completed,int(getattr(response,field))))
            self._call(key,client.srv_type.Request(),accept)

    def poll_flange_crosscheck(self):
        with self.lock:
            if self._joint is None:return
            q=self._joint[2].copy()
        cycle=self._flange_cycle;self._flange_cycle+=1
        self._flange_pending[cycle]={"q_rad":q,"requested":self.clock()}
        for key in ("posj","flange","rotm"):
            client=self.clients.get(key)
            if not client:continue
            request=client.srv_type.Request()
            if key in ("flange","rotm"):request.ref=0
            def accept(response,requested,completed,key=key,cycle=cycle):
                self._accept_crosscheck(cycle,key,response,completed)
            self._call(key,request,accept)

    def _accept_crosscheck(self,cycle,key,response,completed):
        pending=self._flange_pending.get(cycle)
        if pending is None:return
        pending[key]=response;pending[key+"_completed"]=completed
        if not all(name in pending for name in ("posj","flange","rotm")):return
        q=pending["q_rad"];controller_q=np.deg2rad(np.asarray(pending["posj"].pos,float))
        self.posj_comparisons.append({"time":completed,"joint_state_rad":q.tolist(),
          "controller_posj_rad":controller_q.tolist(),
          "difference_deg":np.rad2deg(q-controller_q).tolist()})
        fk=self.kin.fk(q);reported_p=np.asarray(pending["flange"].pos[:3],float)/1000
        reported_r=np.asarray(_rows(pending["rotm"].rot_matrix),float)
        angle=Rotation.from_matrix(fk.rotation@reported_r.T).magnitude()
        times=[pending[name+"_completed"] for name in ("posj","flange","rotm")]
        self.flange_comparisons.append({"time":completed,"joint_rad":q.tolist(),
          "fk_link6_position_m":fk.position_m.tolist(),
          "reported_flange_position_m":reported_p.tolist(),
          "position_error_mm":float(np.linalg.norm(fk.position_m-reported_p)*1000),
          "rotation_error_deg":float(np.rad2deg(angle)),"service_span_s":float(max(times)-min(times))})
        del self._flange_pending[cycle]

    def snapshot(self,now=None):
        now=self.clock() if now is None else float(now)
        with self.lock:
            if self._joint is None:raise RuntimeError("JOINT_FEEDBACK_MISSING")
            jt,header,q,v,_=self._joint;state=self._state;mode=self._mode;system=self._system
        st,sv,lat=state if state is not None else (None,None,None)
        return FeedbackSnapshot(min(jt,st) if st is not None else jt,header,q.copy(),v.copy(),
          self.tool.link_to_tool(self.kin.fk(q)),sv,mode[1] if mode else None,
          system[1] if system else None,st,lat,jt)

    def stale_reasons(self,now=None):
        now=self.clock() if now is None else float(now);reasons=[]
        with self.lock:joint=self._joint;fresh=self._joint_fresh;state=self._state;faults=list(self._faults)
        if joint is None or not fresh or now-joint[0]>self.stale_s:reasons.append("JOINT_STATE_STALE")
        state_stale_s=max(self.stale_s,.25)
        if state is None or now-state[0]>state_stale_s:reasons.append("ROBOT_STATE_STALE")
        reasons.extend(event["code"] for event in faults)
        return reasons

    def preflight_reasons(self,now=None):
        now=self.clock() if now is None else float(now);reasons=self.stale_reasons(now)
        try:snapshot=self.snapshot(now)
        except RuntimeError as error:return reasons+[str(error)],None
        with self.lock:mode=self._mode;system=self._system
        if mode is None or now-mode[0]>2.0:reasons.append("ROBOT_MODE_STALE")
        if system is None or now-system[0]>2.0:reasons.append("ROBOT_SYSTEM_STALE")
        if snapshot.robot_mode!=1:reasons.append(f"ROBOT_MODE_NOT_AUTONOMOUS:{snapshot.robot_mode}")
        if snapshot.robot_system!=0:reasons.append(f"ROBOT_SYSTEM_NOT_REAL:{snapshot.robot_system}")
        if snapshot.robot_state!=rs.PREFLIGHT_STATE:reasons.append(f"ROBOT_STATE_NOT_STANDBY:{snapshot.robot_state}")
        return reasons,snapshot

class FeedbackJsonlLogger:
    def __init__(self,path):self.path=Path(path);self.path.parent.mkdir(parents=True,exist_ok=True)
    def write(self,snapshot):
        row=snapshot.log_record()
        with self.path.open("a") as stream:stream.write(json.dumps(row,separators=(",",":"))+"\n")
        return row


class FeedbackSpinThread:
    """Owns the feedback executor so main-thread blocking cannot stop callbacks."""
    def __init__(self,executor,node):
        self.executor=executor;self.node=node;self.stop_event=threading.Event();self.thread=None
    def start(self):
        self.executor.add_node(self.node)
        self.thread=threading.Thread(target=self._run,name="v2-feedback-spin",daemon=True)
        self.thread.start();return self
    def _run(self):
        while not self.stop_event.is_set():self.executor.spin_once(timeout_sec=.02)
    def stop(self):
        self.stop_event.set()
        if self.thread is not None:self.thread.join(timeout=1)
        self.executor.remove_node(self.node)
