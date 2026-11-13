#!/usr/bin/python3
"""Settled-observation policy using precomputed ServoJ Mode-A segments.

Process A owns feedback, safety, ServoJ and MoveStop. Process B owns camera,
JPEG and HTTP inference. There is no automatic home or gripper command.
"""
from __future__ import annotations
import argparse,json,multiprocessing as mp,os,sys,time
from datetime import datetime
from pathlib import Path
import numpy as np

ROOT=Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:sys.path.insert(0,str(ROOT))
from controller.action import ActionInterpreter
from controller.config import load_config
from controller.geometry import ToolTransform
from controller.precheck import EndpointStreak,GripperStateTracker,joint_envelope_status,joint_margin_reasons
from kinematics import PyKDLKinematics
from runtime import robot_state as rs
from runtime.config import load_runtime_config
from runtime.feedback import FeedbackSpinThread,RobotFeedback
from runtime.observation_process import run_observation_process
from runtime.policy_movej_core import (AbortSignal,MoveJPolicyPlanner,
  PolicyMoveJPrecheckError,validate_sigma_override)
from runtime.servoj import (FixedSegmentPlanner,ModeAObservationLatch,PolicyServoJRunner,
  ServoJPrecheckError,ServojPublisher)
from runtime.stop import StopCoordinator
from runtime.system_monitor_launcher import SystemMonitorProcess
from tools.dryrun_live import validate_instruction
from tools.policy_movej import (DEFAULT_SESSION,STATS,load_gate,pose_delta,rejected_candidate_record,
  require_table_watch_confirmation,start_abort_reader)
from tools.servoj_hold_test import AlarmMonitor,_preflight


def validate_segment_T(value):
    value=float(value)
    if not any(abs(value-x)<1e-12 for x in (.6,1.0)):
        raise ValueError("segment-T must be exactly 0.6 or 1.0 seconds")
    return value


def servoj_rejected_candidate(step,action,error,move_plan=None,seed_joint_rad=None):
    data={"step":int(step),"action_raw":action.raw.tolist(),
      "action_translation_norm_mm":float(np.linalg.norm(action.delta_position_m)*1000),
      "action_rotation_norm_deg":float(np.rad2deg(np.linalg.norm(action.rotation_vector_rad))),
      "exception_message":str(error)}
    if move_plan is not None:
        data.update({"target_tool_pose":{"position_m":move_plan.target_tool_pose.position_m.tolist(),
          "rotation_matrix":move_plan.target_tool_pose.rotation.tolist()},
          "ik_result_joint_rad":move_plan.target_joint_rad.tolist(),
          "ik_result_joint_deg":np.rad2deg(move_plan.target_joint_rad).tolist()})
        if seed_joint_rad is not None:
            delta=move_plan.target_joint_rad-np.asarray(seed_joint_rad,float)
            data.update(joint_delta_rad=delta.tolist(),joint_delta_deg=np.rad2deg(delta).tolist())
    data.update(getattr(error,"diagnostics",{}) or {})
    return data


def write_summary(path,summary):
    path.mkdir(parents=True,exist_ok=True)
    (path/"summary.json").write_text(json.dumps(summary,indent=2)+"\n")


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument("--server",default="http://127.0.0.1:8766");parser.add_argument("--server-pid",type=int)
    parser.add_argument("--camera-topic",required=True);parser.add_argument("--instruction",default="Pick up the blue cube.")
    parser.add_argument("--scene-cube-color",default="blue");parser.add_argument("--max-steps",type=int)
    parser.add_argument("--max-episode-time",type=float);parser.add_argument("--sigma-min",type=float)
    parser.add_argument("--sigma-min-reason");parser.add_argument("--max-descent-mm",type=float)
    parser.add_argument("--segment-T",type=float,default=1.0);parser.add_argument("--session",type=Path,default=DEFAULT_SESSION)
    args=parser.parse_args();segment_T=validate_segment_T(args.segment_T);validate_instruction(args.instruction,False)
    rc=load_runtime_config();cc=load_config();pc=rc["policy_movej"]
    max_steps=int(pc["max_steps_default"] if args.max_steps is None else args.max_steps)
    max_episode_time=float(pc["max_episode_time_s_candidate"] if args.max_episode_time is None else args.max_episode_time)
    if max_steps<1:raise ValueError("max-steps must be positive")
    if not np.isfinite(max_episode_time) or max_episode_time<=0:raise ValueError("max-episode-time must be finite and positive")
    if args.max_descent_mm is not None and (not np.isfinite(args.max_descent_mm) or args.max_descent_mm<0):
        raise ValueError("max-descent-mm must be finite and non-negative")
    sigma_audit=validate_sigma_override(cc.jacobian_min_singular_value,args.sigma_min,args.sigma_min_reason)
    gate=load_gate();gate.require("policy")
    output=args.session/"policy_servoj"/datetime.now().strftime("%Y%m%d_%H%M%S");output.mkdir(parents=True)
    run_start=time.monotonic();ctx=mp.get_context("spawn");control_pipe,observation_pipe=ctx.Pipe(True)
    observer=ctx.Process(target=run_observation_process,args=(observation_pipe,args.camera_topic,args.server,
      args.instruction,str(output),str(STATS),.1,rc["camera_frame_wait_timeout_s_candidate"]),
      name="v2-policy-servoj-observation");observer.start();observation_pipe.close()
    monitor=SystemMonitorProcess(ROOT,output,os.getpid(),observer.pid,args.server,args.camera_topic,run_start,args.server_pid)
    import rclpy
    from runtime.ros_transports import MoveStopTransport
    rclpy.init();node=rclpy.create_node("v2_policy_servoj_control_safety")
    kin=PyKDLKinematics();tool=ToolTransform.tool0_real();feedback=RobotFeedback.from_ros_node(node,kin,tool,rc)
    spin=FeedbackSpinThread(rclpy.executors.SingleThreadedExecutor(),node).start()
    alarm=AlarmMonitor(node,rc["get_last_alarm_service"]).start();publisher=stop_transport=None
    abort_signal=AbortSignal();episode_start=None;rollout_start_pose=real_home_pose=last_pose=None
    minimum_height_mm=0.;first_envelope_exit=None;endpoint_events=[];end_reason="NOT_STARTED"
    steps_file=(output/"steps.jsonl").open("a")
    summary={"pass":False,"steps":[],"automatic_home_calls":0,"gripper_command_calls":0,
      "instruction":args.instruction,"scene_cube_color":args.scene_cube_color,"max_steps":max_steps,
      "max_episode_time_s":max_episode_time,"control_mode":"servoj_A","segment_T_s":segment_T,
      "servoj_time_s":float(rc["servoj_time"]),"action_magnitude_caps":"disabled_by_user",
      "sigma_min":sigma_audit,"max_descent_mm":args.max_descent_mm,
      "table_collision_control":"테이블 높이 자동 검사 없음, 작업자 감시",
      "optional_descent_guard":{"enabled":args.max_descent_mm is not None,"max_descent_mm":args.max_descent_mm}}
    try:
        warmup=time.monotonic();feedback.wait_for_samples_after(warmup,rc["feedback_first_sample_timeout_s_candidate"])
        deadline=time.monotonic()+rc["observation_process_ready_timeout_s_candidate"]
        while time.monotonic()<deadline:
            if not observer.is_alive():raise RuntimeError("OBSERVATION_PROCESS_DIED")
            if control_pipe.poll(.02) and control_pipe.recv().get("type")=="READY":break
        else:raise RuntimeError("OBSERVATION_PROCESS_NO_READY")
        reasons,snapshot,preflight=_preflight(node,feedback,alarm,cc,rc)
        if reasons:raise RuntimeError("PREFLIGHT:"+",".join(reasons))
        if input("Type I_CONFIRM_V2_MODE:policy_servoj exactly: ")!="I_CONFIRM_V2_MODE:policy_servoj":
            raise PermissionError("FRESH_HUMAN_CONFIRMATION_REQUIRED")
        summary["table_watch_confirmation"]=require_table_watch_confirmation(input("Type TABLE_WATCH_BY_OPERATOR exactly: "))
        reasons,snapshot,confirmed_preflight=_preflight(node,feedback,alarm,cc,rc)
        if reasons:raise RuntimeError("CONFIRMED_PREFLIGHT:"+",".join(reasons))
        publisher=ServojPublisher.from_ros_node(node,rc,gate,enabled=True,execution_mode="policy")
        stop_transport=MoveStopTransport(node,rc["stop_service"])
        print("Press Enter, or type q then Enter, for immediate ServoJ inhibit + MoveStop.");start_abort_reader(abort_signal)
        interpreter=ActionInterpreter(STATS);endpoint=EndpointStreak();gripper=GripperStateTracker()
        command_q=snapshot.joint_rad.copy();command_tool=tool.link_to_tool(kin.fk(command_q))
        rollout_start_pose=snapshot.tool0_real_pose;last_pose=rollout_start_pose
        real_home_pose=tool.link_to_tool(kin.fk(cc.real_home_joint_rad))
        target_planner=MoveJPolicyPlanner(kin,tool,cc,rc,sigma_min=sigma_audit["value"],
          max_descent_mm=args.max_descent_mm,home_tool_z_m=real_home_pose.position_m[2])
        segment_planner=FixedSegmentPlanner(kin,tool,cc,rate_hz=100.,sigma_min=sigma_audit["value"])
        episode_start=time.monotonic();latch=ModeAObservationLatch();end_reason="MAX_STEPS_COMPLETE"
        def stop_now(reason):
            result=StopCoordinator(publisher,feedback,stop_transport,rc).stop(reason)
            if reason=="OPERATOR_ABORT":
                summary["operator_abort_requested_at_monotonic"]=abort_signal.requested_at
                summary["operator_abort_move_stop_requested_at_monotonic"]=result["requested_at"]
                summary["operator_abort_response_s"]=abort_signal.response_s(result["requested_at"])
            return result
        for step_index in range(max_steps):
            step_started=time.monotonic()
            if step_started-episode_start>=max_episode_time:stop_now("EPISODE_TIME_LIMIT");end_reason="EPISODE_TIME_LIMIT";break
            if abort_signal.is_set():stop_now("OPERATOR_ABORT");end_reason="OPERATOR_ABORT";break
            # Establish a new settlement boundary; no prior action is repeated.
            zero_since=None
            while True:
                now=time.monotonic();snap=feedback.snapshot(now);faults=feedback.stale_reasons(now)
                if snap.robot_state not in rs.ALLOWED_RUNNING_STATES:faults.append(f"ROBOT_STATE_NOT_RUNNING:{snap.robot_state}")
                faults.extend(joint_margin_reasons(snap.joint_rad,cc))
                if alarm.active():faults.append("ROBOT_ALARM_ACTIVE")
                if faults:stop_now("SAFETY:"+",".join(faults));end_reason="SAFETY_STOP";break
                if abort_signal.is_set():stop_now("OPERATOR_ABORT");end_reason="OPERATOR_ABORT";break
                if np.all(np.abs(snap.joint_velocity_rad_s)<=pc["settle_velocity_rad_s_candidate"]):
                    zero_since=now if zero_since is None else zero_since
                    if now-zero_since>=pc["settle_hold_s_candidate"]:break
                else:zero_since=None
                time.sleep(.005)
            if end_reason!="MAX_STEPS_COMPLETE":break
            settle_host=time.monotonic();settle_ros_ns=node.get_clock().now().nanoseconds;latch.request()
            control_pipe.send({"type":"OBSERVE","t_settle_host_monotonic":settle_host,"t_settle_ros_ns":settle_ros_ns})
            message=None
            while True:
                if abort_signal.is_set():stop_now("OPERATOR_ABORT");end_reason="OPERATOR_ABORT";break
                if time.monotonic()-episode_start>=max_episode_time:stop_now("EPISODE_TIME_LIMIT");end_reason="EPISODE_TIME_LIMIT";break
                now=time.monotonic();snap=feedback.snapshot(now);faults=feedback.stale_reasons(now)
                if snap.robot_state not in rs.ALLOWED_RUNNING_STATES:faults.append(f"ROBOT_STATE_NOT_RUNNING:{snap.robot_state}")
                faults.extend(joint_margin_reasons(snap.joint_rad,cc))
                if alarm.active():faults.append("ROBOT_ALARM_ACTIVE")
                if faults:stop_now("SAFETY:"+",".join(faults));end_reason="SAFETY_STOP";break
                if not observer.is_alive():stop_now("OBSERVATION_PROCESS_DIED");end_reason="OBSERVATION_PROCESS_DIED";break
                if control_pipe.poll(.01):message=control_pipe.recv();break
            if end_reason!="MAX_STEPS_COMPLETE":break
            action_raw=latch.accept(message)
            if message.get("type")=="CAMERA_NO_FRAME_TIMEOUT":end_reason="CAMERA_NO_FRAME_TIMEOUT";break
            if action_raw is None:end_reason="INFERENCE_FAILED";break
            action=interpreter.parse(action_raw);streak=endpoint.update(action)
            endpoint_events.append({"step":step_index,"min":action.endpoint_min[:6].tolist(),
              "max":action.endpoint_max[:6].tolist(),"streak":streak.tolist()})
            actual_start=feedback.snapshot().tool0_real_pose;ik_started=time.monotonic();move_plan=None
            try:
                move_plan=target_planner.plan(action,command_tool,command_q,streak)
                segment=segment_planner.plan_to_pose(command_tool,move_plan.target_tool_pose,command_q,segment_T)
                segment_delta_deg=np.rad2deg(segment.joints_rad[-1]-command_q)
                over=np.flatnonzero(np.abs(segment_delta_deg)>
                  float(pc["max_joint_delta_deg_candidate"]))
                if len(over):
                    raise ServoJPrecheckError([f"JOINT_STEP_CANDIDATE_EXCEEDED:J{i+1}" for i in over],
                      {"ik_result_joint_rad":segment.joints_rad[-1].tolist(),
                       "ik_result_joint_deg":np.rad2deg(segment.joints_rad[-1]).tolist(),
                       "joint_delta_rad":(segment.joints_rad[-1]-command_q).tolist(),
                       "joint_delta_deg":segment_delta_deg.tolist()})
            except PolicyMoveJPrecheckError as error:
                end_reason="PRECHECK_REJECTED";summary["precheck_reasons"]=list(error.reasons)
                summary["rejected_candidate"]=rejected_candidate_record(step_index,action,error)
                summary["rejected_candidate"]["ik_precheck_s"]=time.monotonic()-ik_started;break
            except ServoJPrecheckError as error:
                end_reason="PRECHECK_REJECTED";summary["precheck_reasons"]=list(error.reasons)
                summary["rejected_candidate"]=servoj_rejected_candidate(step_index,action,error,move_plan,command_q)
                summary["rejected_candidate"]["ik_precheck_s"]=time.monotonic()-ik_started;break
            ik_precheck_s=time.monotonic()-ik_started;stream_started=time.monotonic()
            runner=PolicyServoJRunner(publisher,feedback,alarm,cc,rc)
            thread,holder=runner.start(segment,stop_callback=stop_now,operator_abort=abort_signal,
              episode_deadline=episode_start+max_episode_time)
            while thread.is_alive():thread.join(.01)
            stream=holder["result"]
            if stream.fault:
                summary["last_stop"]=stream.stop_result
                end_reason=("OPERATOR_ABORT" if "OPERATOR_ABORT" in stream.fault else
                  "EPISODE_TIME_LIMIT" if "EPISODE_TIME_LIMIT" in stream.fault else "SAFETY_STOP");break
            step_ended=time.monotonic();arrived=feedback.snapshot();trajectory_end=stream.trajectory_end_monotonic or step_ended
            settled=stream.settled_monotonic or step_ended
            record={"step":step_index,"action_raw":action.raw.tolist(),
              "interpreted_delta_position_m":action.delta_position_m.tolist(),
              "interpreted_rotation_vector_rad":action.rotation_vector_rad.tolist(),
              "action_translation_norm_mm":float(np.linalg.norm(action.delta_position_m)*1000),
              "action_rotation_norm_deg":float(np.rad2deg(np.linalg.norm(action.rotation_vector_rad))),
              "ik_precheck_s":ik_precheck_s,"move_s":trajectory_end-stream_started,
              "settle_s":settled-trajectory_end,"step_total_s":step_ended-step_started,
              "t_step_start":step_started,"t_step_end":step_ended,
              "endpoint_arm_min":action.endpoint_min[:6].tolist(),"endpoint_arm_max":action.endpoint_max[:6].tolist(),
              "endpoint_streak":streak.tolist(),"gripper_target":action.gripper_target,
              "gripper_binary":action.gripper_binary,"gripper_changed":gripper.update(action.gripper_binary),
              "gripper_execution":"record_only","target_joint_deg_j1_to_j6":np.rad2deg(segment.joints_rad[-1]).tolist(),
              "move_joint_ack":None,"motion_time_s":step_ended-stream_started,
              "command_delta":pose_delta(command_tool,move_plan.target_tool_pose),
              "actual_delta":pose_delta(actual_start,arrived.tool0_real_pose),
              "arrival_joint_error_deg":np.rad2deg(arrived.joint_rad-segment.joints_rad[-1]).tolist(),
              "path_sigma_min":float(np.min(segment.sigma_min)),"path_minimum_tool_z_m":move_plan.path_minimum_tool_z_m,
              "sim_joint_envelope_log_only":joint_envelope_status(arrived.joint_rad,cc),
              "frame_record":message.get("frame_record"),"servoj_sent_count":stream.sent_count,
              "send_jitter_ms":stream.send_jitter_ms,"missed_deadline_count":stream.missed_deadline_count,
              "max_lateness_ms":stream.max_lateness_ms,"tracking_err_deg_max":stream.tracking_err_deg_max,
              "tracking_threshold_deg":stream.tracking_threshold_deg,"segment_T_s":segment_T}
            steps_file.write(json.dumps(record,separators=(",",":"))+"\n");steps_file.flush();summary["steps"].append(record)
            command_q=segment.joints_rad[-1].copy();command_tool=move_plan.target_tool_pose;last_pose=arrived.tool0_real_pose
            minimum_height_mm=min(minimum_height_mm,(last_pose.position_m[2]-real_home_pose.position_m[2])*1000)
            if first_envelope_exit is None and record["sim_joint_envelope_log_only"]["outside"]:
                first_envelope_exit={"step":step_index,"joints":record["sim_joint_envelope_log_only"]["joints"],
                  "excess_deg":record["sim_joint_envelope_log_only"]["excess_deg"]}
        summary.update({"pass":end_reason in ("MAX_STEPS_COMPLETE","CAMERA_NO_FRAME_TIMEOUT","EPISODE_TIME_LIMIT"),
          "end_reason":end_reason,"servoj_segment_count":len(summary["steps"]),"move_joint_call_count":0,
          "observation_process_exit_detected":not observer.is_alive()})
    except Exception as error:
        summary.update(reason=str(error),end_reason="EXCEPTION")
        if publisher is not None and stop_transport is not None:
            try:summary["exception_stop"]=StopCoordinator(publisher,feedback,stop_transport,rc).stop("POLICY_SERVOJ_EXCEPTION")
            except Exception as stop_error:summary["exception_stop_error"]=str(stop_error)
    finally:
        monitor_status=monitor.stop()
        if rollout_start_pose is not None:
            try:last_pose=feedback.snapshot().tool0_real_pose
            except Exception:pass
            cumulative=pose_delta(rollout_start_pose,last_pose)
            minimum_height_mm=min(minimum_height_mm,(last_pose.position_m[2]-real_home_pose.position_m[2])*1000)
        else:cumulative={"position_m":[0.]*3,"rotation_vector_rad":[0.]*3}
        cumulative["position_mm"]=(np.asarray(cumulative["position_m"])*1000).tolist()
        cumulative["position_norm_mm"]=float(np.linalg.norm(cumulative["position_m"])*1000)
        cumulative["rotation_vector_deg"]=np.rad2deg(cumulative["rotation_vector_rad"]).tolist()
        cumulative["rotation_magnitude_deg"]=float(np.rad2deg(np.linalg.norm(cumulative["rotation_vector_rad"])))
        min_counts=np.sum([x["min"] for x in endpoint_events],axis=0).astype(int).tolist() if endpoint_events else [0]*6
        max_counts=np.sum([x["max"] for x in endpoint_events],axis=0).astype(int).tolist() if endpoint_events else [0]*6
        max_streak=np.max([x["streak"] for x in endpoint_events],axis=0).astype(int).tolist() if endpoint_events else [[0,0] for _ in range(6)]
        summary.update({"end_reason":summary.get("end_reason",end_reason),"executed_steps":len(summary["steps"]),
          "elapsed_s":None if episode_start is None else time.monotonic()-episode_start,
          "cumulative_displacement":cumulative,"minimum_height_relative_home_mm":minimum_height_mm,
          "sim_joint_envelope_first_exit":first_envelope_exit,
          "endpoint_arm_summary":{"dimensions":["dx","dy","dz","rx","ry","rz"],
            "interpreted_actions":len(endpoint_events),"min_counts":min_counts,"max_counts":max_counts,
            "max_streak_min_max":max_streak},"gripper_execution":"record_only","gripper_io_verified":False,
          "alarm_events":alarm.events,"system_monitor":monitor_status})
        steps_file.close();write_summary(output,summary);print(json.dumps(summary,indent=2))
        if observer.is_alive():
            try:control_pipe.send({"type":"SHUTDOWN"})
            except (BrokenPipeError,EOFError):pass
            observer.join(timeout=2)
        if observer.is_alive():observer.terminate();observer.join(timeout=1)
        control_pipe.close();alarm.close();spin.stop();node.destroy_node();rclpy.shutdown()


if __name__=="__main__":main()
