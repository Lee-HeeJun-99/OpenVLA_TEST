#!/usr/bin/python3
"""Single-shot J6 MoveJoint/MoveStop validation. No retry is implemented."""
import json,math,multiprocessing as mp,sys,time
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
from runtime.stop import StopCoordinator

SESSION=ROOT/"reports/live/20261113_091859/stop_test_movej"

def expected_profile(distance_deg=2.0,velocity_deg_s=5.0,acceleration_deg_s2=10.0):
    triangular=distance_deg < velocity_deg_s**2/acceleration_deg_s2
    peak=math.sqrt(distance_deg*acceleration_deg_s2) if triangular else velocity_deg_s
    accel_time=peak/acceleration_deg_s2
    cruise_time=0.0 if triangular else (distance_deg-peak**2/acceleration_deg_s2)/peak
    total=2*accel_time+cruise_time
    return {"shape":"triangular" if triangular else "trapezoidal","peak_velocity_deg_s":peak,
      "acceleration_time_s":accel_time,"cruise_time_s":cruise_time,"total_time_s":total,
      "stop_trigger_velocity_deg_s":.5*peak,"invalid_deadline_s":.7*total,
      "points":[[0.0,0.0],[accel_time,peak],[accel_time+cruise_time,peak],[total,0.0]]}

def stop_trigger_status(j6_speed_deg_s,elapsed_s,profile):
    if abs(float(j6_speed_deg_s))>=profile["stop_trigger_velocity_deg_s"]:return "THRESHOLD"
    if float(elapsed_s)>=profile["invalid_deadline_s"]:return "INVALID_TIMEOUT"
    return "WAITING"

def load_gate():
    raw=json.loads((ROOT/"runtime/gates.json").read_text())
    return ExecutionGate({k:GateEntry(v["passed"],v.get("evidence"),v.get("evidence_type")) for k,v in raw.items()})

class CountedMoveJoint:
    def __init__(self,transport):self.transport=transport;self.calls=[];self.inhibited=False
    def move_joint(self,**kwargs):
        if self.inhibited:raise PermissionError("MOVE_JOINT_PERMANENTLY_INHIBITED")
        self.calls.append({"time":time.monotonic(),"request":kwargs});return self.transport.move_joint(**kwargs)
    def inhibit(self):self.inhibited=True;self.transport.inhibit()

def worker(connection,output):
    import rclpy
    from runtime.ros_transports import MoveJointTransport,MoveStopTransport
    out=Path(output);out.mkdir(parents=True,exist_ok=True);rc=load_runtime_config();cc=load_config()
    result={"pass":False};move=None;stop_transport=None;spin=None
    rclpy.init();node=rclpy.create_node("v2_stop_test_movej_control_safety")
    feedback=RobotFeedback.from_ros_node(node,PyKDLKinematics(),ToolTransform.tool0_real(),rc)
    executor=rclpy.executors.SingleThreadedExecutor();started=time.monotonic();spin=FeedbackSpinThread(executor,node).start()
    try:
        gate=load_gate()
        for item in ("pendant_limits_recorded","live_topics_verified","feedback_rate_verified"):gate.require_item(item)
        feedback.wait_for_samples_after(started,2.0)
        deadline=time.monotonic()+2
        while time.monotonic()<deadline and (feedback._mode is None or feedback._system is None):time.sleep(.01)
        reasons,snapshot=feedback.preflight_reasons();home_error=np.rad2deg(snapshot.joint_rad-cc.real_home_joint_rad)
        reasons += [f"START_NOT_REAL_HOME:J{i+1}" for i,x in enumerate(home_error) if abs(x)>2]
        if reasons:raise RuntimeError("PREFLIGHT:"+",".join(reasons))
        stationary_start=time.monotonic();qs=[];max_speed=0.0
        while time.monotonic()-stationary_start<2:
            snap=feedback.snapshot();qs.append(snap.joint_rad.copy())
            max_speed=max(max_speed,float(np.max(np.abs(snap.joint_velocity_rad_s))));time.sleep(.01)
        position_range=float(np.max(np.ptp(np.rad2deg(np.asarray(qs)),axis=0)))
        stationary={"duration_s":time.monotonic()-stationary_start,"max_abs_velocity_rad_s":max_speed,
          "max_joint_position_range_deg":position_range,"pass":max_speed<.01 and position_range<.05}
        (out/"arm_stationary.json").write_text(json.dumps(stationary,indent=2)+"\n")
        if not stationary["pass"]:raise RuntimeError("ARM_NOT_STATIONARY")
        connection.send({"type":"PREFLIGHT_OK","stationary":stationary})
        confirmation=connection.recv()
        if confirmation.get("value")!="I_CONFIRM_V2_MODE:stop_test_movej":
            raise RuntimeError("FRESH_HUMAN_CONFIRMATION_REQUIRED")

        cfg=rc["stop_test_movej"];profile=expected_profile(2.0,float(cfg["velocity_deg_s_candidate"]),
          float(cfg["acceleration_deg_s2_unverified"]))
        (out/"expected_velocity_profile.json").write_text(json.dumps(profile,indent=2)+"\n")
        move=CountedMoveJoint(MoveJointTransport(node,rc["move_joint_service"]))
        stop_transport=MoveStopTransport(node,rc["stop_service"])
        motion_start=time.monotonic();ack=move.move_joint(pos=cfg["target_joint_deg"],
          vel=cfg["velocity_deg_s_candidate"],acc=cfg["acceleration_deg_s2_unverified"],
          mode=cfg["mode"],blend_type=cfg["blend_type"],sync_type=cfg["sync_type"])
        trigger=None;invalid=False;motion_samples=[]
        while trigger is None:
            now=time.monotonic();snap=feedback.snapshot(now);j6=float(abs(np.rad2deg(snap.joint_velocity_rad_s[5])))
            motion_samples.append({"time_monotonic":now,"elapsed_s":now-motion_start,
              "joint_rad":snap.joint_rad.tolist(),"joint_velocity_rad_s":snap.joint_velocity_rad_s.tolist(),
              "robot_state":snap.robot_state})
            faults=feedback.stale_reasons(now)
            if faults:raise RuntimeError("SAFETY_MONITOR:"+",".join(faults))
            status=stop_trigger_status(j6,now-motion_start,profile)
            if status=="THRESHOLD":trigger=motion_samples[-1];break
            if status=="INVALID_TIMEOUT":
                trigger=motion_samples[-1];invalid=True;break
            time.sleep(.005)
        coordinator=StopCoordinator(move,feedback,stop_transport,rc)
        stop_result=coordinator.stop("STOP_TEST_MOVEJ_TRIGGER" if not invalid else "STOP_TEST_MOVEJ_TRIGGER_NOT_REACHED")
        move_calls_after_stop=max(0,len(move.calls)-1)
        with feedback.lock:
            joints=list(feedback.joint_events);polls=list(feedback.poll_events)
            states=list(feedback.state_events);flange=list(feedback.flange_comparisons)
        with (out/"joint_states.jsonl").open("w") as stream:
            for row in joints:stream.write(json.dumps(row,separators=(",",":"))+"\n")
        with (out/"robot_state.jsonl").open("w") as stream:
            for row in states:stream.write(json.dumps(row,separators=(",",":"))+"\n")
        (out/"robot_state_poll.json").write_text(json.dumps(polls,indent=2)+"\n")
        (out/"flange_crosscheck.json").write_text(json.dumps(flange,indent=2)+"\n")
        nonzero=abs(float(trigger["joint_velocity_rad_s"][5]))>0
        passed=(not invalid and nonzero and stop_result.get("state")=="STOP_CONFIRMED" and
                move_calls_after_stop==0)
        result={"pass":passed,"invalid_trigger_timeout":invalid,"profile":profile,
          "move_joint_ack":ack,"trigger_sample":trigger,"stop_result":stop_result,
          "move_joint_total_calls":len(move.calls),"move_joint_calls_after_stop":move_calls_after_stop,
          "candidate_gate_updates":["stop_movej_verified"] if passed else [],"gate_updates_applied":False}
    except Exception as error:
        result.update(reason=str(error))
        if move is not None:
            if move.calls and stop_transport is not None:
                try:result["exception_stop_result"]=StopCoordinator(
                  move,feedback,stop_transport,rc).stop("STOP_TEST_MOVEJ_EXCEPTION")
                except Exception as stop_error:result["exception_stop_error"]=str(stop_error)
            move.inhibit()
    finally:
        (out/"summary.json").write_text(json.dumps(result,indent=2)+"\n")
        try:connection.send({"type":"DONE","summary":result})
        except (BrokenPipeError,EOFError):pass
        if spin is not None:spin.stop()
        node.destroy_node();rclpy.shutdown();connection.close()

def main():
    ctx=mp.get_context("spawn");parent,child=ctx.Pipe();process=ctx.Process(target=worker,args=(child,str(SESSION)))
    process.start();child.close()
    try:
        while True:
            message=parent.recv()
            if message["type"]=="PREFLIGHT_OK":
                print("REVIEW CANDIDATES: J6 +2 deg, velocity 5 deg/s, acceleration 10 deg/s^2, MoveStop mode 1 (DR_QSTOP)")
                value=input("Type I_CONFIRM_V2_MODE:stop_test_movej exactly: ")
                parent.send({"type":"CONFIRM","value":value})
            elif message["type"]=="DONE":print(json.dumps(message["summary"],indent=2));break
    finally:
        parent.close();process.join(timeout=3)
        if process.is_alive():process.terminate();process.join()

if __name__=="__main__":main()
