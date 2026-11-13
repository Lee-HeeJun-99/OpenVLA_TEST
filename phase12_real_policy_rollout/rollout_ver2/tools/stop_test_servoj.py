#!/usr/bin/env /usr/bin/python3
"""Physical ServoJ streaming-stop validation with verified MoveJoint return."""
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
from runtime.gate import ExecutionGate,GateEntry
from runtime.servoj import StopServoJRunner,TimeExplorePublisher
from runtime.system_monitor_launcher import SystemMonitorProcess
from tools.servoj_hold_test import AlarmMonitor,PipeAbort,_preflight
DEFAULT_SESSION=ROOT/"reports/live/20261113_091859/stop_test_servoj"

def load_gate():
    raw=json.loads((ROOT/"runtime/gates.json").read_text())
    return ExecutionGate({k:GateEntry(v["passed"],v.get("evidence"),v.get("evidence_type")) for k,v in raw.items()})

def return_with_movej(move,stop,feedback,alarm,start_q,rc):
    cfg=rc["stop_test_servoj"];ack=move.move_joint(pos=np.rad2deg(start_q).tolist(),
      vel=cfg["return_velocity_deg_s"],acc=cfg["return_acceleration_deg_s2"],mode=0,blend_type=0,sync_type=1)
    begin=time.monotonic();zero_since=None;rows=[];confirmed=None;reason=None
    while time.monotonic()-begin<float(cfg["return_timeout_s"]):
        now=time.monotonic();snap=feedback.snapshot(now);error=np.max(np.abs(np.rad2deg(snap.joint_rad-start_q)))
        speed=np.max(np.abs(snap.joint_velocity_rad_s));rows.append({"time":now,"joint_rad":snap.joint_rad.tolist(),
          "velocity_rad_s":snap.joint_velocity_rad_s.tolist(),"max_error_deg":float(error),"robot_state":snap.robot_state})
        faults=feedback.stale_reasons(now)
        if alarm.active():faults.append("ROBOT_ALARM_ACTIVE")
        if snap.robot_state not in (1,2):faults.append(f"ROBOT_STATE_NOT_RUNNING:{snap.robot_state}")
        if faults:reason=",".join(faults);break
        if speed<=.001 and error<=float(cfg["return_position_tolerance_deg"]):
            zero_since=now if zero_since is None else zero_since
            if now-zero_since>=.3:confirmed=now;break
        else:zero_since=None
        time.sleep(.01)
    if confirmed is None:
        move.inhibit()
        try:stop_ack=stop.move_stop(stop_mode=1)
        except Exception as error:stop_ack={"success":False,"error":str(error)}
        return {"pass":False,"reason":reason or "RETURN_TIMEOUT","move_joint_ack":ack,"emergency_stop_ack":stop_ack,"samples":rows}
    return {"pass":True,"move_joint_ack":ack,"confirmed_monotonic":confirmed,"elapsed_s":confirmed-begin,"samples":rows}

def worker(connection,output):
    import rclpy
    from runtime.ros_transports import MoveJointTransport,MoveStopTransport
    rc=load_runtime_config();cc=load_config();out=Path(output);out.mkdir(parents=True,exist_ok=True)
    result={"pass":False,"sent_count":0,"gate_updates_applied":False,
      "stop_confirmation":None,"post_stop_travel_deg":None};node=spin=alarm=monitor=publisher=None
    rclpy.init();node=rclpy.create_node("v2_stop_test_servoj_control_safety")
    feedback=RobotFeedback.from_ros_node(node,PyKDLKinematics(),ToolTransform.tool0_real(),rc)
    executor=rclpy.executors.MultiThreadedExecutor(num_threads=2);spin=FeedbackSpinThread(executor,node).start()
    alarm=AlarmMonitor(node,rc["get_last_alarm_service"]).start()
    try:
        load_gate().require("stop_test_servoj")
        reasons,snapshot,preflight=_preflight(node,feedback,alarm,cc,rc);result["preflight"]=preflight
        if reasons:result["reason"]="PREFLIGHT:"+",".join(reasons);connection.send({"type":"DONE","summary":result});return
        connection.send({"type":"PREFLIGHT_OK","preflight":preflight,"settings":rc["stop_test_servoj"]})
        message=connection.recv()
        if message.get("value")!="I_CONFIRM_V2_MODE:stop_test_servoj":
            result["reason"]="FRESH_HUMAN_CONFIRMATION_REQUIRED";connection.send({"type":"DONE","summary":result});return
        reasons,snapshot,confirmed=_preflight(node,feedback,alarm,cc,rc)
        if reasons:result["reason"]="CONFIRMED_PREFLIGHT:"+",".join(reasons);connection.send({"type":"DONE","summary":result});return
        publisher=TimeExplorePublisher.from_ros_node(node,rc);stop=MoveStopTransport(node,rc["stop_service"])
        move=MoveJointTransport(node,rc["move_joint_service"])
        monitor=SystemMonitorProcess(ROOT,out,os.getpid(),None,"http://127.0.0.1:8766",None,time.monotonic())
        runner=StopServoJRunner(publisher,feedback,stop,alarm,rc,abort_requested=PipeAbort(connection))
        result=runner.run(snapshot.joint_rad.copy())
        if result["pass"]:result["return_to_start"]=return_with_movej(move,stop,feedback,alarm,snapshot.joint_rad.copy(),rc)
        else:result["return_to_start"]={"pass":False,"reason":"SKIPPED_STOP_TEST_FAILED"}
        result["pass"]=bool(result["pass"] and result["return_to_start"]["pass"])
        if not result["pass"]:result["candidate_gate_updates"]=[]
        result.update(preflight=preflight,confirmed_preflight=confirmed,system_monitor=monitor.stop())
        with (out/"samples.jsonl").open("w") as stream:
            for row in runner.rows:stream.write(json.dumps(row,separators=(",",":"))+"\n")
        (out/"summary.json").write_text(json.dumps(result,indent=2)+"\n");connection.send({"type":"DONE","summary":result})
    except Exception as error:
        result["reason"]=f"EXCEPTION:{type(error).__name__}:{error}"
        if publisher is not None:
            publisher.inhibit()
            try:result["exception_move_stop"]={"ack":MoveStopTransport(node,rc["stop_service"]).move_stop(stop_mode=1)}
            except Exception as stop_error:result["exception_move_stop"]={"ack":{"success":False,"error":str(stop_error)}}
        try:connection.send({"type":"DONE","summary":result})
        except Exception:pass
    finally:
        if monitor is not None:result["system_monitor"]=monitor.stop()
        if not (out/"summary.json").exists():(out/"summary.json").write_text(json.dumps(result,indent=2)+"\n")
        if alarm is not None:alarm.close()
        if spin is not None:spin.stop()
        node.destroy_node();rclpy.shutdown();connection.close()

def main():
    ap=argparse.ArgumentParser();ap.add_argument("--enable-physical",action="store_true");ap.add_argument("--output",type=Path)
    args=ap.parse_args()
    if not args.enable_physical:raise PermissionError("PHYSICAL_EXECUTION_DISABLED")
    output=args.output or DEFAULT_SESSION/time.strftime("%Y%m%d_%H%M%S")
    ctx=mp.get_context("spawn");parent,child=ctx.Pipe();process=ctx.Process(target=worker,args=(child,str(output)));process.start();child.close()
    try:
        while True:
            message=parent.recv()
            if message["type"]=="PREFLIGHT_OK":
                print(json.dumps(message,indent=2));value=input("Type I_CONFIRM_V2_MODE:stop_test_servoj exactly: ")
                parent.send({"type":"CONFIRM","value":value})
                if value=="I_CONFIRM_V2_MODE:stop_test_servoj":
                    print("Stop test active. Press Enter or q then Enter for immediate inhibit + MoveStop.")
                    def abort():
                        try:
                            if input().strip().lower() in ("","q"):parent.send({"type":"ABORT"})
                        except Exception:pass
                    threading.Thread(target=abort,daemon=True).start()
            elif message["type"]=="DONE":print(json.dumps(message["summary"],indent=2));break
    finally:
        parent.close();process.join(timeout=5)
        if process.is_alive():process.terminate();process.join()
if __name__=="__main__":main()
