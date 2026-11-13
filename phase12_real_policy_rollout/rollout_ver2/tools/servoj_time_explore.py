#!/usr/bin/env /usr/bin/python3
"""One-shot physical ServoJ time exploration; four fail-fast trials."""
from __future__ import annotations
import argparse,json,multiprocessing as mp,os,sys,threading,time
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:sys.path.insert(0,str(ROOT))
from controller.config import load_config
from controller.geometry import ToolTransform
from kinematics import PyKDLKinematics
from runtime.config import load_runtime_config
from runtime.feedback import FeedbackSpinThread,RobotFeedback
from runtime.gate import ExecutionGate,GateEntry
from runtime.servoj import TimeExplorePublisher,TimeExploreRunner,write_time_explore_outputs
from runtime.system_monitor_launcher import SystemMonitorProcess
from tools.servoj_hold_test import AlarmMonitor,PipeAbort,_preflight
DEFAULT_SESSION=ROOT/"reports/live/20261113_091859/servoj_time_explore"

def load_gate():
    raw=json.loads((ROOT/"runtime/gates.json").read_text())
    return ExecutionGate({k:GateEntry(v["passed"],v.get("evidence"),v.get("evidence_type")) for k,v in raw.items()})

def worker(connection,output):
    import rclpy
    from runtime.ros_transports import MoveStopTransport
    rc=load_runtime_config();cc=load_config();out=Path(output);out.mkdir(parents=True,exist_ok=True)
    result={"pass":False,"sent_count":0,"gate_updates_applied":False,
      "stop_confirmation":None,"post_stop_travel_deg":None,"trials":[]};node=spin=alarm=monitor=publisher=None
    rclpy.init();node=rclpy.create_node("v2_servoj_time_explore_control_safety")
    feedback=RobotFeedback.from_ros_node(node,PyKDLKinematics(),ToolTransform.tool0_real(),rc)
    executor=rclpy.executors.MultiThreadedExecutor(num_threads=2);spin=FeedbackSpinThread(executor,node).start()
    alarm=AlarmMonitor(node,rc["get_last_alarm_service"]).start()
    try:
        gate=load_gate()
        for item in ("pendant_limits_recorded","live_topics_verified","feedback_rate_verified","arm_stationary_verified"):gate.require_item(item)
        reasons,snapshot,preflight=_preflight(node,feedback,alarm,cc,rc);result["preflight"]=preflight
        if reasons:result["reason"]="PREFLIGHT:"+",".join(reasons);connection.send({"type":"DONE","summary":result});return
        connection.send({"type":"PREFLIGHT_OK","preflight":preflight,"trials":rc["servoj_time_explore"]})
        message=connection.recv()
        if message.get("value")!="I_CONFIRM_V2_MODE:servoj_time_explore":
            result["reason"]="FRESH_HUMAN_CONFIRMATION_REQUIRED";connection.send({"type":"DONE","summary":result});return
        reasons,snapshot,confirmed=_preflight(node,feedback,alarm,cc,rc);result["confirmed_preflight"]=confirmed
        if reasons:result["reason"]="CONFIRMED_PREFLIGHT:"+",".join(reasons);connection.send({"type":"DONE","summary":result});return
        publisher=TimeExplorePublisher.from_ros_node(node,rc);stop=MoveStopTransport(node,rc["stop_service"])
        monitor=SystemMonitorProcess(ROOT,out,os.getpid(),None,"http://127.0.0.1:8766",None,time.monotonic())
        runner=TimeExploreRunner(publisher,feedback,stop,alarm,rc,abort_requested=PipeAbort(connection))
        result=runner.run(snapshot.joint_rad.copy());publisher.inhibit()
        result.update(preflight=preflight,confirmed_preflight=confirmed,system_monitor=monitor.stop())
        write_time_explore_outputs(out,runner.rows,result);connection.send({"type":"DONE","summary":result})
    except Exception as error:
        result["reason"]=f"EXCEPTION:{type(error).__name__}:{error}"
        if publisher is not None:
            publisher.inhibit()
            requested=time.monotonic()
            try:result["exception_move_stop"]={"requested_monotonic":requested,
              "ack":MoveStopTransport(node,rc["stop_service"]).move_stop(stop_mode=1),
              "ack_completed_monotonic":time.monotonic()}
            except Exception as stop_error:result["exception_move_stop"]={"ack":{"success":False,"error":str(stop_error)}}
        try:connection.send({"type":"DONE","summary":result})
        except (BrokenPipeError,EOFError):pass
    finally:
        if monitor is not None:result["system_monitor"]=monitor.stop()
        if not (out/"summary.json").exists():write_time_explore_outputs(out,[],result)
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
                print(json.dumps(message,indent=2));value=input("Type I_CONFIRM_V2_MODE:servoj_time_explore exactly: ")
                parent.send({"type":"CONFIRM","value":value})
                if value=="I_CONFIRM_V2_MODE:servoj_time_explore":
                    print("Time exploration active. Press Enter or type q then Enter to inhibit and MoveStop.")
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
