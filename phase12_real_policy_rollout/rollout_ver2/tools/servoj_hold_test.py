#!/usr/bin/env /usr/bin/python3
"""One-shot physical ServoJ hold validation. No target motion is generated."""
from __future__ import annotations
import argparse,json,multiprocessing as mp,os,sys,threading,time
from pathlib import Path
import numpy as np
ROOT=Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:sys.path.insert(0,str(ROOT))
from controller.config import load_config
from controller.geometry import ToolTransform
from kinematics import PyKDLKinematics
from runtime.config import load_runtime_config
from runtime.feedback import FeedbackSpinThread,RobotFeedback
from runtime.servoj import HoldRunner,HoldServojPublisher,write_hold_outputs
from runtime.system_monitor_launcher import SystemMonitorProcess
DEFAULT_SESSION=ROOT/"reports/live/20261113_091859/servoj_hold_test"

class AlarmMonitor:
    def __init__(self,node,name):
        from dsr_msgs2.srv import GetLastAlarm
        from runtime.ros_transports import RosServiceCaller
        self.caller=RosServiceCaller(node,GetLastAlarm,name);self.lock=threading.Lock()
        self.last=None;self.events=[];self.ever_active=False;self.stop_event=threading.Event();self.thread=None
    def start(self):
        self.thread=threading.Thread(target=self._run,name="servoj-hold-alarm",daemon=True);self.thread.start();return self
    def _run(self):
        while not self.stop_event.is_set():
            try:
                result=self.caller.call();alarm=result.get("log_alarm")
                row={"time":time.monotonic(),"success":result.get("success",False),
                  "level":int(getattr(alarm,"level",0)),"group":int(getattr(alarm,"group",0)),
                  "index":int(getattr(alarm,"index",0)),"param":list(getattr(alarm,"param",[]))}
                row["active"]=any(row[k]!=0 for k in ("level","group","index"))
            except Exception as error:row={"time":time.monotonic(),"success":False,"active":True,"error":str(error)}
            with self.lock:self.last=row;self.events.append(row);self.ever_active|=bool(row["active"])
            self.stop_event.wait(.1)
    def wait_first(self,timeout=2.):
        deadline=time.monotonic()+timeout
        while time.monotonic()<deadline:
            with self.lock:
                if self.last is not None:return dict(self.last)
            time.sleep(.01)
        return None
    def active(self):
        with self.lock:return self.last is None or bool(self.last.get("active")) or not self.last.get("success",False)
    def record(self):
        with self.lock:return None if self.last is None else dict(self.last)
    def close(self):
        self.stop_event.set()
        if self.thread:self.thread.join(timeout=2)

class PipeAbort:
    def __init__(self,connection):self.connection=connection;self.value=False
    def __call__(self):
        if self.value:return True
        while self.connection.poll():
            if self.connection.recv().get("type")=="ABORT":self.value=True
        return self.value

def _preflight(node,feedback,alarm,cc,rc):
    reasons=[];started=time.monotonic();feedback.wait_for_samples_after(started,rc["feedback_first_sample_timeout_s_candidate"])
    deadline=time.monotonic()+2
    while time.monotonic()<deadline:
        current,_=feedback.preflight_reasons()
        if not any("MODE_STALE" in x or "SYSTEM_STALE" in x for x in current):break
        time.sleep(.02)
    current,snapshot=feedback.preflight_reasons();reasons.extend(current)
    if snapshot is None:error=np.full(6,np.nan)
    else:
        error=np.rad2deg(snapshot.joint_rad-cc.real_home_joint_rad)
        reasons.extend(f"START_NOT_REAL_HOME:J{i+1}" for i,x in enumerate(error) if abs(x)>2.)
    alarm_row=alarm.wait_first(2.)
    if alarm_row is None:reasons.append("ALARM_NO_FIRST_SAMPLE")
    elif not alarm_row.get("success") or alarm_row.get("active"):reasons.append("ALARM_ACTIVE_OR_STALE")
    publishers=node.count_publishers(rc["servoj_topic"]);subscribers=node.count_subscribers(rc["servoj_topic"])
    if publishers!=0:reasons.append(f"SERVOJ_OTHER_PUBLISHERS:{publishers}")
    if subscribers<1:reasons.append(f"SERVOJ_NO_DRIVER_SUBSCRIBER:{subscribers}")
    return reasons,snapshot,{"real_home_error_deg":error.tolist(),"servoj_publishers_before_create":publishers,
      "servoj_subscribers":subscribers,"alarm":alarm_row}

def worker(connection,output):
    import rclpy
    from runtime.ros_transports import MoveStopTransport
    rc=load_runtime_config();cc=load_config();out=Path(output);out.mkdir(parents=True,exist_ok=True)
    result={"pass":False,"evidence_pass":False,"sent_count":0,"gate_updates_applied":False,
      "stop_confirmation":None,"post_stop_travel_deg":None}
    node=spin=alarm=monitor=publisher=None
    rclpy.init();node=rclpy.create_node("v2_servoj_hold_control_safety")
    feedback=RobotFeedback.from_ros_node(node,PyKDLKinematics(),ToolTransform.tool0_real(),rc)
    executor=rclpy.executors.MultiThreadedExecutor(num_threads=2);spin=FeedbackSpinThread(executor,node).start()
    alarm=AlarmMonitor(node,rc["get_last_alarm_service"]).start()
    try:
        reasons,snapshot,preflight=_preflight(node,feedback,alarm,cc,rc);result["preflight"]=preflight
        if reasons:result["reason"]="PREFLIGHT:"+",".join(reasons);connection.send({"type":"DONE","summary":result});return
        connection.send({"type":"PREFLIGHT_OK","preflight":preflight});message=connection.recv()
        if message.get("value")!="I_CONFIRM_V2_MODE:servoj_hold":
            result["reason"]="FRESH_HUMAN_CONFIRMATION_REQUIRED";connection.send({"type":"DONE","summary":result});return
        reasons,snapshot,confirmed_preflight=_preflight(node,feedback,alarm,cc,rc)
        result["confirmed_preflight"]=confirmed_preflight
        if reasons:
            result["reason"]="CONFIRMED_PREFLIGHT:"+",".join(reasons);connection.send({"type":"DONE","summary":result});return
        # Command objects exist only after all preflight checks and confirmation.
        publisher=HoldServojPublisher.from_ros_node(node,rc);stop=MoveStopTransport(node,rc["stop_service"])
        monitor=SystemMonitorProcess(ROOT,out,os.getpid(),None,"http://127.0.0.1:8766",
          None,time.monotonic())
        runner=HoldRunner(publisher,feedback,stop,alarm,rc,abort_requested=PipeAbort(connection))
        result=runner.run(snapshot.joint_rad.copy());result["preflight"]=preflight
        result["confirmed_preflight"]=confirmed_preflight
        result["robot_state_values"]=sorted(set(r["robot_state"] for r in runner.rows))
        result["robot_state_changes"]=[r for i,r in enumerate(runner.rows)
          if i==0 or r["robot_state"]!=runner.rows[i-1]["robot_state"]]
        result["system_monitor"]=monitor.stop();write_hold_outputs(out,runner.rows,result)
        connection.send({"type":"DONE","summary":result})
    except Exception as error:
        result["reason"]=f"EXCEPTION:{type(error).__name__}:{error}"
        if publisher is not None:
            publisher.inhibit()
            requested=time.monotonic()
            try:
                ack=MoveStopTransport(node,rc["stop_service"]).move_stop(stop_mode=1)
                result["exception_move_stop"]={"requested_monotonic":requested,
                  "ack_completed_monotonic":time.monotonic(),"ack":ack}
            except Exception as stop_error:result["exception_stop_error"]=str(stop_error)
        connection.send({"type":"DONE","summary":result})
    finally:
        if monitor is not None:result["system_monitor"]=monitor.stop()
        if not (out/"summary.json").exists():write_hold_outputs(out,[],result)
        if alarm is not None:alarm.close()
        if spin is not None:spin.stop()
        node.destroy_node();rclpy.shutdown();connection.close()

def main():
    ap=argparse.ArgumentParser();ap.add_argument("--enable-physical",action="store_true");ap.add_argument("--output",type=Path)
    args=ap.parse_args()
    if not args.enable_physical:raise PermissionError("PHYSICAL_EXECUTION_DISABLED")
    output=args.output or DEFAULT_SESSION/time.strftime("%Y%m%d_%H%M%S")
    ctx=mp.get_context("spawn");parent,child=ctx.Pipe();process=ctx.Process(target=worker,args=(child,str(output)))
    process.start();child.close()
    try:
        while True:
            message=parent.recv()
            if message["type"]=="PREFLIGHT_OK":
                print(json.dumps(message["preflight"],indent=2));value=input("Type I_CONFIRM_V2_MODE:servoj_hold exactly: ")
                parent.send({"type":"CONFIRM","value":value})
                if value=="I_CONFIRM_V2_MODE:servoj_hold":
                    print("Hold active. Press Enter or type q then Enter to stop immediately.")
                    def wait_abort():
                        try:
                            if input().strip().lower() in ("","q"):parent.send({"type":"ABORT"})
                        except (EOFError,BrokenPipeError,OSError):pass
                    threading.Thread(target=wait_abort,daemon=True).start()
            elif message["type"]=="DONE":print(json.dumps(message["summary"],indent=2));break
    finally:
        parent.close();process.join(timeout=5)
        if process.is_alive():process.terminate();process.join()
if __name__=="__main__":main()
