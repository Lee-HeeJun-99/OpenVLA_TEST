#!/usr/bin/python3
"""Operator-driven gripper test. No arm command publisher/client is created."""
import json,multiprocessing as mp,sys,time
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

SESSION=ROOT/"reports/live/20261113_091859/gripper_test"

def require_operator_polarity_confirmation(yes):
    if not bool(yes):raise RuntimeError("POLARITY_MISMATCH")
    return True

def load_gate():
    raw=json.loads((ROOT/"runtime/gates.json").read_text())
    return ExecutionGate({k:GateEntry(v["passed"],v.get("evidence"),v.get("evidence_type")) for k,v in raw.items()})

def append_jsonl(path,row):
    with path.open("a") as stream:stream.write(json.dumps(row,separators=(",",":"))+"\n")

def worker(connection,output):
    import rclpy
    from runtime.ros_transports import ToolDigitalOutputReader,ToolDigitalOutputTransport
    out=Path(output);out.mkdir(parents=True,exist_ok=True);events=out/"events.jsonl"
    rc=load_runtime_config();cc=load_config();gate=load_gate();transport=None;result={"pass":False}
    rclpy.init();node=rclpy.create_node("v2_gripper_test_control_safety")
    feedback=RobotFeedback.from_ros_node(node,PyKDLKinematics(),ToolTransform.tool0_real(),rc)
    executor=rclpy.executors.SingleThreadedExecutor();started=time.monotonic()
    spin=FeedbackSpinThread(executor,node).start()
    def event(kind,**values):
        row={"time_monotonic":time.monotonic(),"event":kind,**values};append_jsonl(events,row);return row
    def read_pair():
        return {i:int(transport.get_tool_digital_output(i)["value"]) for i in (1,2)}
    def both_off():
        rows=[]
        for i in (1,2):rows.append({"index":i,"ack":transport.set_tool_digital_output(i,0)})
        event("BOTH_OFF",writes=rows,readback=read_pair());return rows
    def safe(baseline):
        reasons=feedback.stale_reasons();snap=feedback.snapshot()
        if snap.robot_state not in (1,2):reasons.append("ROBOT_STATE_UNSAFE:"+str(snap.robot_state))
        if np.max(np.abs(snap.joint_velocity_rad_s))>=.01:reasons.append("ARM_MOVED_VELOCITY")
        if np.max(np.abs(np.rad2deg(snap.joint_rad-baseline)))>=.05:reasons.append("ARM_MOVED_POSITION")
        return reasons
    def wait_reply(kind,baseline):
        while True:
            reasons=safe(baseline)
            if reasons:raise RuntimeError("SAFETY_MONITOR:"+",".join(reasons))
            if connection.poll(.01):
                message=connection.recv()
                if message.get("type")!=kind:raise RuntimeError("PROTOCOL_ERROR")
                return message
    try:
        for item in ("pendant_limits_recorded","live_topics_verified","gripper_wiring_recorded"):
            gate.require_item(item)
        feedback.wait_for_samples_after(started,2.0)
        deadline=time.monotonic()+2
        while time.monotonic()<deadline and (feedback._mode is None or feedback._system is None):time.sleep(.01)
        reasons,snapshot=feedback.preflight_reasons()
        home_error=np.rad2deg(snapshot.joint_rad-cc.real_home_joint_rad)
        reasons += [f"START_NOT_REAL_HOME:J{i+1}" for i,x in enumerate(home_error) if abs(x)>2]
        if reasons:raise RuntimeError("PREFLIGHT:"+",".join(reasons))
        stationary_start=time.monotonic();qs=[];max_speed=0.0
        while time.monotonic()-stationary_start<2.0:
            snap=feedback.snapshot();qs.append(snap.joint_rad.copy())
            max_speed=max(max_speed,float(np.max(np.abs(snap.joint_velocity_rad_s))))
            if feedback.stale_reasons():raise RuntimeError("STATIONARY_FEEDBACK_FAULT")
            time.sleep(.01)
        position_range=float(np.max(np.ptp(np.rad2deg(np.asarray(qs)),axis=0)))
        stationary={"evidence_type":"measurement","duration_s":time.monotonic()-stationary_start,
          "max_abs_velocity_rad_s":max_speed,"max_joint_position_range_deg":position_range,
          "velocity_candidate_rad_s":.01,"position_candidate_deg":.05,
          "pass":max_speed<.01 and position_range<.05}
        evidence=out/"arm_stationary.json";evidence.write_text(json.dumps(stationary,indent=2)+"\n")
        if not stationary["pass"]:raise RuntimeError("ARM_NOT_STATIONARY")
        baseline=feedback.snapshot().joint_rad.copy()
        reader=ToolDigitalOutputReader(node,rc["get_tool_digital_output_service"])
        initial={i:int(reader.get_tool_digital_output(i)["value"]) for i in (1,2)}
        event("INITIAL_READBACK",values=initial)
        if initial!={1:0,2:0}:raise RuntimeError("INITIAL_OUTPUT_NOT_BOTH_OFF")
        connection.send({"type":"PREFLIGHT_OK","stationary":stationary,"initial_readback":initial})
        if wait_reply("CONFIRM",baseline).get("value")!="I_CONFIRM_V2_MODE:gripper_test":
            raise RuntimeError("FRESH_HUMAN_CONFIRMATION_REQUIRED")
        # The only state-changing transport is created after every preflight and confirmation.
        transport=ToolDigitalOutputTransport(node,rc["tool_digital_output_service"])
        initial=read_pair();event("POST_CONFIRM_READBACK",values=initial)
        if initial!={1:0,2:0}:raise RuntimeError("OUTPUT_CHANGED_AFTER_PREFLIGHT")
        transport.set_tool_digital_output(2,0);event("WRITE",index=2,value=0)
        transport.set_tool_digital_output(1,1);open_started=time.monotonic()
        event("WRITE",index=1,value=1,readback=read_pair())
        connection.send({"type":"ASK_POLARITY"})
        polarity=wait_reply("POLARITY",baseline)
        try:require_operator_polarity_confirmation(polarity.get("yes"))
        except RuntimeError:
            both_off();raise
        connection.send({"type":"ASK_OPEN"});opened=wait_reply("OPEN",baseline)
        if not opened.get("yes"):raise RuntimeError("OPEN_NOT_OBSERVED")
        event("OPEN_COMPLETE",operator_time=opened["operator_time"],
              observed_duration_s=opened["operator_time"]-open_started,readback=read_pair())
        transport.set_tool_digital_output(1,0);event("WRITE",index=1,value=0)
        transport.set_tool_digital_output(2,1);close_started=time.monotonic()
        event("WRITE",index=2,value=1,readback=read_pair())
        connection.send({"type":"ASK_CLOSE"});closed=wait_reply("CLOSE",baseline)
        if not closed.get("yes"):raise RuntimeError("CLOSE_NOT_OBSERVED")
        event("CLOSE_COMPLETE",operator_time=closed["operator_time"],
              observed_duration_s=closed["operator_time"]-close_started,readback=read_pair())
        both_off();connection.send({"type":"ASK_FINAL"});final=wait_reply("FINAL",baseline)
        event("FINAL_BOTH_OFF_OBSERVATION",observation=final.get("observation"))
        result={"pass":True,"candidate_gate_updates":["arm_stationary_verified","gripper_io_verified"],
          "gate_updates_applied":False,"final_observation":final.get("observation")}
    except Exception as error:
        result.update(reason=str(error));event("TEST_END",**result)
    finally:
        if transport is not None:
            try:both_off()
            except Exception as error:event("FINAL_BOTH_OFF_FAILED",error=str(error))
        (out/"summary.json").write_text(json.dumps(result,indent=2)+"\n")
        try:connection.send({"type":"DONE","summary":result})
        except (BrokenPipeError,EOFError):pass
        spin.stop();node.destroy_node();rclpy.shutdown();connection.close()

def main():
    ctx=mp.get_context("spawn");parent,child=ctx.Pipe();process=ctx.Process(target=worker,args=(child,str(SESSION)))
    process.start();child.close()
    try:
        while True:
            message=parent.recv();kind=message["type"]
            if kind=="PREFLIGHT_OK":
                value=input("Type I_CONFIRM_V2_MODE:gripper_test exactly: ");parent.send({"type":"CONFIRM","value":value})
            elif kind=="ASK_POLARITY":
                yes=input("펜던트에 DO1이 On으로 표시됩니까? (y/n) ").strip().lower()=="y"
                parent.send({"type":"POLARITY","yes":yes})
            elif kind=="ASK_OPEN":
                yes=input("그리퍼가 열렸습니까? (y/n) ").strip().lower()=="y"
                if yes:input("완전히 열린 시점에 Enter: ")
                parent.send({"type":"OPEN","yes":yes,"operator_time":time.monotonic()})
            elif kind=="ASK_CLOSE":
                yes=input("그리퍼가 닫혔습니까? (y/n) ").strip().lower()=="y"
                if yes:input("완전히 닫힌 시점에 Enter: ")
                parent.send({"type":"CLOSE","yes":yes,"operator_time":time.monotonic()})
            elif kind=="ASK_FINAL":
                observation=input("DO1/DO2 Off 뒤 닫힌 상태가 유지됩니까, 풀립니까? [유지/풀림] ").strip()
                parent.send({"type":"FINAL","observation":observation})
            elif kind=="DONE":print(json.dumps(message["summary"],indent=2));break
    finally:
        parent.close();process.join(timeout=3)
        if process.is_alive():process.terminate();process.join()

if __name__=="__main__":main()
