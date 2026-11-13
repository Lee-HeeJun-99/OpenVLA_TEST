#!/usr/bin/python3
"""Mode-A MoveJoint policy rollout with process-isolated observation.

This executable is the only policy_movej layer allowed to construct MoveJoint
and MoveStop transports.  It never constructs a gripper output transport and
never returns home automatically.
"""
import argparse,json,multiprocessing as mp,os,sys,threading,time
from datetime import datetime
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation

ROOT=Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:sys.path.insert(0,str(ROOT))
from controller.action import ActionInterpreter
from controller.config import load_config
from controller.geometry import ToolTransform
from controller.precheck import (EndpointStreak,GripperStateTracker,
  joint_envelope_status,joint_margin_reasons)
from kinematics import PyKDLKinematics
from runtime.config import load_runtime_config
from runtime.feedback import FeedbackSpinThread,RobotFeedback
from runtime.gate import ExecutionGate,GateEntry
from runtime.observation_process import run_observation_process
from runtime.policy_movej_core import (AbortSignal,MoveJPolicyPlanner,
  PolicyMoveJPrecheckError,SettlementTracker,validate_sigma_override)
from runtime.stop import StopCoordinator
from runtime.system_monitor_launcher import SystemMonitorProcess
from runtime import robot_state as rs
from tools.dryrun_live import validate_instruction

STATS=ROOT.parents[2]/"models/vanilla_s1_balanced_step8130/dataset_statistics.json"
DEFAULT_SESSION=ROOT/"reports/live/20261113_091859"

def load_gate():
    raw=json.loads((ROOT/"runtime/gates.json").read_text())
    return ExecutionGate({key:GateEntry(value["passed"],value.get("evidence"),
      value.get("evidence_type")) for key,value in raw.items()})

class CountedMoveJoint:
    def __init__(self,transport):self.transport=transport;self.calls=[];self.inhibited=False
    def move_joint(self,**kwargs):
        if self.inhibited:raise PermissionError("MOVE_JOINT_PERMANENTLY_INHIBITED")
        self.calls.append({"time_monotonic":time.monotonic(),"request":kwargs})
        return self.transport.move_joint(**kwargs)
    def inhibit(self):self.inhibited=True;self.transport.inhibit()

def pose_delta(start,end):
    return {"position_m":(end.position_m-start.position_m).tolist(),
      "rotation_vector_rad":Rotation.from_matrix(end.rotation@start.rotation.T).as_rotvec().tolist()}

def start_abort_reader(signal):
    def read():
        line=sys.stdin.readline()
        if line!="" and line.strip().lower() in ("","q"):signal.request()
    thread=threading.Thread(target=read,name="policy-movej-operator-abort",daemon=True)
    thread.start();return thread

def require_table_watch_confirmation(value):
    if value!="TABLE_WATCH_BY_OPERATOR":raise PermissionError("TABLE_WATCH_CONFIRMATION_REQUIRED")
    return value

def rejected_candidate_record(step,action,error):
    return {"step":int(step),"action_raw":action.raw.tolist(),
            "action_translation_norm_mm":float(np.linalg.norm(action.delta_position_m)*1000),
            "action_rotation_norm_deg":float(np.rad2deg(np.linalg.norm(action.rotation_vector_rad))),
            "exception_message":str(error),**error.rejected_candidate}

def main():
    parser=argparse.ArgumentParser()
    parser.add_argument("--server",default="http://127.0.0.1:8766")
    parser.add_argument("--server-pid",type=int)
    parser.add_argument("--camera-topic",required=True)
    parser.add_argument("--instruction",default="Pick up the blue cube.")
    parser.add_argument("--scene-cube-color",default="blue")
    parser.add_argument("--max-steps",type=int,default=None)
    parser.add_argument("--max-episode-time",type=float,default=None)
    parser.add_argument("--sigma-min",type=float,default=None)
    parser.add_argument("--sigma-min-reason")
    parser.add_argument("--max-descent-mm",type=float,default=None)
    parser.add_argument("--session",type=Path,default=DEFAULT_SESSION)
    args=parser.parse_args()
    validate_instruction(args.instruction,False)
    rc=load_runtime_config();cc=load_config();pc=rc["policy_movej"]
    max_steps=int(pc["max_steps_default"] if args.max_steps is None else args.max_steps)
    max_episode_time=float(pc["max_episode_time_s_candidate"] if args.max_episode_time is None
                           else args.max_episode_time)
    if max_steps<1:raise ValueError("max-steps must be positive")
    if not np.isfinite(max_episode_time) or max_episode_time<=0:
        raise ValueError("max-episode-time must be finite and positive")
    if args.max_descent_mm is not None and (not np.isfinite(args.max_descent_mm) or args.max_descent_mm<0):
        raise ValueError("max-descent-mm must be finite and non-negative")
    sigma_audit=validate_sigma_override(cc.jacobian_min_singular_value,
                                        args.sigma_min,args.sigma_min_reason)
    gate=load_gate();gate.require("policy_movej")

    output=args.session/"policy_movej"/datetime.now().strftime("%Y%m%d_%H%M%S")
    output.mkdir(parents=True,exist_ok=False)
    run_start_monotonic=time.monotonic()
    ctx=mp.get_context("spawn");control_pipe,observation_pipe=ctx.Pipe(duplex=True)
    observer=ctx.Process(target=run_observation_process,args=(observation_pipe,args.camera_topic,
      args.server,args.instruction,str(output),str(STATS),.1,
      rc["camera_frame_wait_timeout_s_candidate"]),name="v2-policy-observation")
    observer.start();observation_pipe.close()
    system_monitor=SystemMonitorProcess(ROOT,output,os.getpid(),observer.pid,args.server,
      args.camera_topic,run_start_monotonic,args.server_pid)
    import rclpy
    from runtime.ros_transports import MoveJointTransport,MoveStopTransport
    rclpy.init();node=rclpy.create_node("v2_policy_movej_control_safety")
    kin=PyKDLKinematics();tool=ToolTransform.tool0_real()
    feedback=RobotFeedback.from_ros_node(node,kin,tool,rc)
    executor=rclpy.executors.SingleThreadedExecutor();warmup=time.monotonic()
    spin=FeedbackSpinThread(executor,node).start();move=None;stop_transport=None
    abort_signal=AbortSignal();summary={"pass":False,"steps":[],"automatic_home_calls":0,
      "gripper_command_calls":0,"instruction":args.instruction,"scene_cube_color":args.scene_cube_color,
      "max_steps":max_steps,"max_episode_time_s":max_episode_time,
      "action_magnitude_caps":"disabled_by_user",
      "sigma_min":sigma_audit,"max_descent_mm":args.max_descent_mm,
      "table_collision_control":"테이블 높이 자동 검사 없음, 작업자 감시",
      "optional_descent_guard":{"enabled":args.max_descent_mm is not None,
                                "max_descent_mm":args.max_descent_mm}}
    step_stream=(output/"steps.jsonl").open("a")
    episode_start=None;rollout_start_pose=None;real_home_pose=None;last_pose=None
    minimum_height_mm=0.0
    endpoint_events=[];first_envelope_exit=None;end_reason="NOT_STARTED"
    try:
        feedback.wait_for_samples_after(warmup,rc["feedback_first_sample_timeout_s_candidate"])
        deadline=time.monotonic()+rc["observation_process_ready_timeout_s_candidate"]
        while time.monotonic()<deadline:
            if not observer.is_alive():raise RuntimeError("OBSERVATION_PROCESS_DIED")
            if control_pipe.poll(.02) and control_pipe.recv().get("type")=="READY":break
        else:raise RuntimeError("OBSERVATION_PROCESS_NO_READY")
        identity_deadline=time.monotonic()+2
        while time.monotonic()<identity_deadline and (feedback._mode is None or feedback._system is None):time.sleep(.01)
        reasons,snapshot=feedback.preflight_reasons()
        reasons += [f"START_NOT_REAL_HOME:J{i+1}" for i,error in enumerate(
          np.rad2deg(snapshot.joint_rad-cc.real_home_joint_rad)) if abs(error)>2]
        if reasons:raise RuntimeError("PREFLIGHT:"+",".join(reasons))
        confirmation=input("Type I_CONFIRM_V2_MODE:policy_movej exactly: ")
        if confirmation!="I_CONFIRM_V2_MODE:policy_movej":
            raise PermissionError("FRESH_HUMAN_CONFIRMATION_REQUIRED")
        table_confirmation=input("Type TABLE_WATCH_BY_OPERATOR exactly: ")
        summary["table_watch_confirmation"]=require_table_watch_confirmation(table_confirmation)

        # Command clients are deliberately constructed only after all gates,
        # confirmation and read-only preflight checks have passed.
        move=CountedMoveJoint(MoveJointTransport(node,rc["move_joint_service"]))
        stop_transport=MoveStopTransport(node,rc["stop_service"])
        print("Press Enter, or type q then Enter, for immediate MoveStop.")
        start_abort_reader(abort_signal)
        interpreter=ActionInterpreter(STATS)
        endpoint=EndpointStreak();gripper=GripperStateTracker()
        command_q=snapshot.joint_rad.copy();command_tool=tool.link_to_tool(kin.fk(command_q))
        rollout_start_pose=snapshot.tool0_real_pose;last_pose=rollout_start_pose
        real_home_pose=tool.link_to_tool(kin.fk(cc.real_home_joint_rad))
        planner=MoveJPolicyPlanner(kin,tool,cc,rc,sigma_min=sigma_audit["value"],
          max_descent_mm=args.max_descent_mm,home_tool_z_m=real_home_pose.position_m[2])
        episode_start=time.monotonic()
        def stop_now(reason):
            result=StopCoordinator(move,feedback,stop_transport,rc).stop(reason)
            if reason=="OPERATOR_ABORT":
                summary["operator_abort_requested_at_monotonic"]=abort_signal.requested_at
                summary["operator_abort_move_stop_requested_at_monotonic"]=result["requested_at"]
                summary["operator_abort_response_s"]=abort_signal.response_s(result["requested_at"])
            return result
        end_reason="MAX_STEPS_COMPLETE"
        for step_index in range(max_steps):
            step_started=time.monotonic()
            if time.monotonic()-episode_start>=max_episode_time:
                stop_now("EPISODE_TIME_LIMIT");end_reason="EPISODE_TIME_LIMIT";break
            if abort_signal.is_set():
                stop_now("OPERATOR_ABORT");end_reason="OPERATOR_ABORT";break
            settle_host=time.monotonic();settle_ros_ns=node.get_clock().now().nanoseconds
            control_pipe.send({"type":"OBSERVE","t_settle_host_monotonic":settle_host,
                               "t_settle_ros_ns":settle_ros_ns})
            while True:
                if time.monotonic()-episode_start>=max_episode_time:
                    stop_now("EPISODE_TIME_LIMIT");end_reason="EPISODE_TIME_LIMIT";break
                if abort_signal.is_set():
                    stop_now("OPERATOR_ABORT");end_reason="OPERATOR_ABORT";break
                faults=feedback.stale_reasons()
                waiting_snapshot=feedback.snapshot()
                if waiting_snapshot.robot_state not in rs.ALLOWED_RUNNING_STATES:
                    faults.append(f"ROBOT_STATE_NOT_RUNNING:{waiting_snapshot.robot_state}")
                faults.extend(joint_margin_reasons(waiting_snapshot.joint_rad,cc))
                waiting_sigma=float(np.linalg.svd(kin.jacobian(waiting_snapshot.joint_rad),compute_uv=False)[-1])
                if waiting_sigma<sigma_audit["value"]:faults.append("SINGULARITY_APPROACH")
                if faults:
                    stop_now("SAFETY:"+",".join(faults));end_reason="SAFETY_STOP";break
                if not observer.is_alive():
                    stop_now("OBSERVATION_PROCESS_DIED");end_reason="OBSERVATION_PROCESS_DIED";break
                if control_pipe.poll(.01):message=control_pipe.recv();break
            if end_reason!="MAX_STEPS_COMPLETE":break
            if message.get("type")=="CAMERA_NO_FRAME_TIMEOUT":end_reason="CAMERA_NO_FRAME_TIMEOUT";break
            if message.get("type")!="RESULT" or not message.get("ok"):
                end_reason="INFERENCE_FAILED";break
            action=interpreter.parse(message["action_raw"]);streak=endpoint.update(action)
            endpoint_events.append({"step":step_index,"min":action.endpoint_min[:6].tolist(),
              "max":action.endpoint_max[:6].tolist(),"streak":streak.tolist()})
            before=feedback.snapshot();actual_start=before.tool0_real_pose
            ik_started=time.monotonic()
            try:plan=planner.plan(action,command_tool,command_q,streak)
            except PolicyMoveJPrecheckError as error:
                end_reason="PRECHECK_REJECTED";summary["precheck_reasons"]=list(error.reasons)
                summary["rejected_candidate"]=rejected_candidate_record(step_index,action,error)
                summary["rejected_candidate"]["ik_precheck_s"]=time.monotonic()-ik_started
                break
            ik_precheck_s=time.monotonic()-ik_started
            request={"pos":np.rad2deg(plan.target_joint_rad).tolist(),"vel":pc["velocity_deg_s_candidate"],
              "acc":pc["acceleration_deg_s2_confirmed"],"sync_type":1}
            motion_start=time.monotonic();ack=move.move_joint(**request);target_reached_at=None
            tracker=SettlementTracker(pc["settle_velocity_rad_s_candidate"],
              pc["settle_hold_s_candidate"],pc["target_error_deg_candidate"])
            motion_fault=None
            while True:
                now=time.monotonic();snap=feedback.snapshot(now)
                faults=feedback.stale_reasons(now)
                if snap.robot_state not in rs.ALLOWED_RUNNING_STATES: faults.append(f"ROBOT_STATE_NOT_RUNNING:{snap.robot_state}")
                faults.extend(joint_margin_reasons(snap.joint_rad,cc))
                sigma=float(np.linalg.svd(kin.jacobian(snap.joint_rad),compute_uv=False)[-1])
                if sigma<sigma_audit["value"]:faults.append("SINGULARITY_APPROACH")
                if abort_signal.is_set():motion_fault="OPERATOR_ABORT";break
                if time.monotonic()-episode_start>=max_episode_time:motion_fault="EPISODE_TIME_LIMIT";break
                if faults:motion_fault=",".join(faults);break
                if (target_reached_at is None and np.all(np.abs(
                    snap.joint_rad-plan.target_joint_rad)<=np.deg2rad(pc["target_error_deg_candidate"]))):
                    target_reached_at=now
                if tracker.update(snap.joint_rad,snap.joint_velocity_rad_s,plan.target_joint_rad,now):break
                if now-motion_start>pc["motion_timeout_s_candidate"]:motion_fault="MOTION_TIMEOUT";break
                time.sleep(.005)
            if motion_fault:
                stop_now(motion_fault)
                end_reason=motion_fault if motion_fault in ("OPERATOR_ABORT","EPISODE_TIME_LIMIT") else "SAFETY_STOP";break
            step_ended=time.monotonic();target_reached_at=target_reached_at or step_ended
            arrived=feedback.snapshot();record={"step":step_index,"action_raw":action.raw.tolist(),
              "interpreted_delta_position_m":action.delta_position_m.tolist(),
              "interpreted_rotation_vector_rad":action.rotation_vector_rad.tolist(),
              "action_translation_norm_mm":float(np.linalg.norm(action.delta_position_m)*1000),
              "action_rotation_norm_deg":float(np.rad2deg(np.linalg.norm(action.rotation_vector_rad))),
              "ik_precheck_s":ik_precheck_s,"move_s":target_reached_at-motion_start,
              "settle_s":step_ended-target_reached_at,"step_total_s":step_ended-step_started,
              "t_step_start":step_started,"t_step_end":step_ended,
              "endpoint_arm_min":action.endpoint_min[:6].tolist(),"endpoint_arm_max":action.endpoint_max[:6].tolist(),
              "endpoint_streak":streak.tolist(),"gripper_target":action.gripper_target,
              "gripper_binary":action.gripper_binary,"gripper_changed":gripper.update(action.gripper_binary),
              "gripper_execution":"record_only","target_joint_deg_j1_to_j6":request["pos"],
              "move_joint_ack":ack,"motion_time_s":time.monotonic()-motion_start,
              "command_delta":pose_delta(command_tool,plan.target_tool_pose),
              "actual_delta":pose_delta(actual_start,arrived.tool0_real_pose),
              "arrival_joint_error_deg":np.rad2deg(arrived.joint_rad-plan.target_joint_rad).tolist(),
              "path_sigma_min":min(plan.path_minimum_singular_values),
              "path_minimum_tool_z_m":plan.path_minimum_tool_z_m,
              "sim_joint_envelope_log_only":joint_envelope_status(arrived.joint_rad,cc),
              "frame_record":message.get("frame_record")}
            step_stream.write(json.dumps(record,separators=(",",":"))+"\n");step_stream.flush()
            summary["steps"].append(record);command_q=plan.target_joint_rad;command_tool=plan.target_tool_pose
            last_pose=arrived.tool0_real_pose
            minimum_height_mm=min(minimum_height_mm,
              (last_pose.position_m[2]-real_home_pose.position_m[2])*1000)
            if first_envelope_exit is None and record["sim_joint_envelope_log_only"]["outside"]:
                first_envelope_exit={"step":step_index,"joints":record["sim_joint_envelope_log_only"]["joints"],
                  "excess_deg":record["sim_joint_envelope_log_only"]["excess_deg"]}
        summary.update({"pass":end_reason in ("MAX_STEPS_COMPLETE","CAMERA_NO_FRAME_TIMEOUT","EPISODE_TIME_LIMIT"),
          "end_reason":end_reason,"move_joint_call_count":0 if move is None else len(move.calls),
          "observation_process_exit_detected":not observer.is_alive()})
    except Exception as error:
        summary.update(reason=str(error),end_reason="EXCEPTION")
        if move is not None and move.calls and stop_transport is not None:
            try:summary["exception_stop"]=StopCoordinator(move,feedback,stop_transport,rc).stop("POLICY_MOVEJ_EXCEPTION")
            except Exception as stop_error:summary["exception_stop_error"]=str(stop_error)
    finally:
        monitor_status=system_monitor.stop()
        if monitor_status.get("warning"):print("WARNING "+monitor_status["warning"],file=sys.stderr)
        if rollout_start_pose is not None:
            try:last_pose=feedback.snapshot().tool0_real_pose
            except Exception:pass
            cumulative=pose_delta(rollout_start_pose,last_pose)
            minimum_height_mm=min(minimum_height_mm,
              (last_pose.position_m[2]-real_home_pose.position_m[2])*1000)
        else:cumulative={"position_m":[0.0,0.0,0.0],"rotation_vector_rad":[0.0,0.0,0.0]}
        cumulative["position_mm"]=(np.asarray(cumulative["position_m"])*1000).tolist()
        cumulative["position_norm_mm"]=float(np.linalg.norm(cumulative["position_m"])*1000)
        cumulative["rotation_vector_deg"]=np.rad2deg(cumulative["rotation_vector_rad"]).tolist()
        cumulative["rotation_magnitude_deg"]=float(np.rad2deg(
          np.linalg.norm(cumulative["rotation_vector_rad"])))
        min_counts=np.sum([row["min"] for row in endpoint_events],axis=0).astype(int).tolist() if endpoint_events else [0]*6
        max_counts=np.sum([row["max"] for row in endpoint_events],axis=0).astype(int).tolist() if endpoint_events else [0]*6
        max_streak=np.max([row["streak"] for row in endpoint_events],axis=0).astype(int).tolist() if endpoint_events else [[0,0] for _ in range(6)]
        summary.update({"end_reason":summary.get("end_reason",end_reason),
          "executed_steps":len(summary["steps"]),
          "elapsed_s":None if episode_start is None else time.monotonic()-episode_start,
          "cumulative_displacement":cumulative,
          "minimum_height_relative_home_mm":minimum_height_mm,
          "sim_joint_envelope_first_exit":first_envelope_exit,
          "endpoint_arm_summary":{"dimensions":["dx","dy","dz","rx","ry","rz"],
            "interpreted_actions":len(endpoint_events),"min_counts":min_counts,
            "max_counts":max_counts,"max_streak_min_max":max_streak},
          "gripper_execution":"record_only","gripper_io_verified":False,
          "system_monitor":monitor_status})
        step_stream.close();(output/"summary.json").write_text(json.dumps(summary,indent=2)+"\n")
        print(json.dumps(summary,indent=2))
        if observer.is_alive():
            try:control_pipe.send({"type":"SHUTDOWN"})
            except (BrokenPipeError,EOFError):pass
            observer.join(timeout=2)
        if observer.is_alive():observer.terminate();observer.join(timeout=1)
        control_pipe.close();spin.stop();node.destroy_node();rclpy.shutdown()

if __name__=="__main__":main()
