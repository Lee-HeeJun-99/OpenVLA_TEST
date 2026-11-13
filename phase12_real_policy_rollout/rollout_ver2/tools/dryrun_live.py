#!/usr/bin/python3
"""Process-isolated read-only live dryrun; creates no robot command objects."""
import argparse,json,multiprocessing as mp,sys,time
from pathlib import Path
import numpy as np

ROOT=Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:sys.path.insert(0,str(ROOT))
from controller.config import load_config
from controller.geometry import ToolTransform
from kinematics import PyKDLKinematics
from runtime.config import load_runtime_config
from runtime.feedback import FeedbackSpinThread,RobotFeedback
from runtime.monitor import RuntimeMonitor
from runtime.observation_process import run_observation_process

STATS=ROOT.parents[2]/"models/vanilla_s1_balanced_step8130/dataset_statistics.json"
MATCHED15_INSTRUCTIONS={"Pick up the red cube.","Pick up the yellow cube.","Pick up the blue cube."}

def validate_instruction(instruction,allow_ood=False):
    matched=instruction in MATCHED15_INSTRUCTIONS
    if not matched and not allow_ood:raise ValueError("OOD_INSTRUCTION_REQUIRES_EXPLICIT_FLAG:"+instruction)
    return matched

def main():
    p=argparse.ArgumentParser();p.add_argument("--duration",type=float,default=60)
    p.add_argument("--server",default="http://127.0.0.1:8766")
    p.add_argument("--camera-topic",required=True);p.add_argument("--instruction",default="Pick up the blue cube.")
    p.add_argument("--scene-cube-color",default="blue");p.add_argument("--allow-ood-instruction",action="store_true")
    p.add_argument("--output",type=Path,required=True);args=p.parse_args()
    if args.duration<60:raise ValueError("dryrun_live requires at least 60 seconds")
    instruction_in_matched15=validate_instruction(args.instruction,args.allow_ood_instruction)
    if input("Type I_CONFIRM_V2_MODE:dryrun_live exactly: ")!="I_CONFIRM_V2_MODE:dryrun_live":
        raise PermissionError("FRESH_HUMAN_CONFIRMATION_REQUIRED")

    output=args.output;output.mkdir(parents=True,exist_ok=True)
    rc=load_runtime_config();cc=load_config();ctx=mp.get_context("spawn")
    control_pipe,observation_pipe=ctx.Pipe(duplex=True)
    observer=ctx.Process(target=run_observation_process,args=(observation_pipe,args.camera_topic,
      args.server,args.instruction,str(output),str(STATS),.1,
      rc.get("camera_frame_wait_timeout_s_candidate",10.0)),name="v2-observation")
    observer.start();observation_pipe.close()

    import rclpy
    rclpy.init();feedback_node=rclpy.create_node("v2_control_safety_readonly")
    kin=PyKDLKinematics();feedback=RobotFeedback.from_ros_node(
      feedback_node,kin,ToolTransform.tool0_real(),rc)
    feedback_executor=rclpy.executors.SingleThreadedExecutor();warmup_start=time.monotonic()
    feedback_spin=FeedbackSpinThread(feedback_executor,feedback_node).start()
    monitor=RuntimeMonitor(feedback,kin,cc,rc)
    camera_diagnostics={};results=[];observation_records=[];stale_events=[]
    try:
        feedback.wait_for_samples_after(warmup_start,rc.get("feedback_first_sample_timeout_s_candidate",2.0))
        ready_deadline=time.monotonic()+rc.get("observation_process_ready_timeout_s_candidate",30.0)
        observer_ready=False
        while time.monotonic()<ready_deadline:
            if not observer.is_alive():raise RuntimeError("OBSERVATION_PROCESS_DIED_STOP_REQUIRED")
            if control_pipe.poll(.02):
                message=control_pipe.recv()
                if message.get("type")=="READY":observer_ready=True;break
        if not observer_ready:raise RuntimeError("OBSERVATION_PROCESS_NO_READY")
        identity_deadline=time.monotonic()+2.0
        while time.monotonic()<identity_deadline:
            if not feedback.stale_reasons() and feedback._mode and feedback._system:break
            time.sleep(.01)
        reasons,snapshot=monitor.check_start()
        if reasons:raise RuntimeError("DRYRUN_PREFLIGHT_FAILED:"+",".join(reasons))

        periods=[];feedback_ages=[];states=[];runtime_faults=[]
        camera_timeout_count=0;end_reason="DURATION_COMPLETE";pending=False
        start=time.monotonic();previous=start;next_tick=start
        settle_host=start;settle_ros_ns=feedback_node.get_clock().now().nanoseconds
        control_pipe.send({"type":"OBSERVE","t_settle_host_monotonic":settle_host,
                           "t_settle_ros_ns":settle_ros_ns});pending=True
        while time.monotonic()-start<args.duration:
            now=time.monotonic();periods.append(now-previous);previous=now
            reasons_now=feedback.stale_reasons(now);runtime_faults.extend(reasons_now)
            if reasons_now:stale_events.append({"time_monotonic":now,"reasons":reasons_now,
                                                "diagnostics":feedback.diagnostics(now)})
            live=feedback.snapshot(now);feedback_ages.append(now-live.joint_state_received_monotonic)
            states.append(live.robot_state)
            if runtime_faults:end_reason="SAFETY_MONITOR_FAULT";break
            if not observer.is_alive():end_reason="OBSERVATION_PROCESS_DIED_STOP_REQUIRED";break
            if pending and control_pipe.poll():
                message=control_pipe.recv();camera_diagnostics=message.get("camera_diagnostics",camera_diagnostics)
                if message.get("type")=="CAMERA_NO_FRAME_TIMEOUT":
                    camera_timeout_count+=1;end_reason="CAMERA_NO_FRAME_TIMEOUT";break
                if message.get("type")!="RESULT":end_reason="OBSERVATION_PROTOCOL_ERROR";break
                result={"time":time.monotonic(),"ok":bool(message.get("ok")),
                        "frame_record":message.get("frame_record"),
                        "action_raw":message.get("action_raw"),
                        "processing_timing":message.get("prediction_record",{}).get("processing_timing")}
                if not result["ok"]:result["error"]=message.get("error")
                results.append(result);observation_records.append(message.get("frame_record"));pending=False
                if not result["ok"]:end_reason="INFERENCE_FAILED";break
                settle_host=time.monotonic();settle_ros_ns=feedback_node.get_clock().now().nanoseconds
                control_pipe.send({"type":"OBSERVE","t_settle_host_monotonic":settle_host,
                                   "t_settle_ros_ns":settle_ros_ns});pending=True
            next_tick+=.01;time.sleep(max(0,next_tick-time.monotonic()))

        elapsed=time.monotonic()-start
        p99=float(np.percentile(np.asarray(periods[1:])*1000,99))
        summary={"duration_s":elapsed,"architecture":"PROCESS_A_SAFETY__PROCESS_B_OBSERVATION",
          "preflight":{"robot_mode":snapshot.robot_mode,"robot_system":snapshot.robot_system,
          "robot_state":snapshot.robot_state,
          "real_home_joint_error_deg":np.rad2deg(snapshot.joint_rad-cc.real_home_joint_rad).tolist()},
          "virtual_100hz_period_ms_p99":p99,"feedback_age_s_max":float(max(feedback_ages)),
          "robot_states":sorted(set(states)),"runtime_faults":sorted(set(runtime_faults)),
          "end_reason":end_reason,"feedback_stale_events":stale_events,
          "feedback_final_diagnostics":feedback.diagnostics(),
          "camera_no_frame_timeout_count":camera_timeout_count,"observation_records":observation_records,
          "camera_final_diagnostics":camera_diagnostics,"inference_results":results,
          "instruction":args.instruction,"scene_cube_color":args.scene_cube_color,
          "instruction_in_matched15":instruction_in_matched15,
          "allow_ood_instruction":bool(args.allow_ood_instruction),
          "command_publish_count":0,"state_changing_service_call_count":0,
          "pass":elapsed>=args.duration and p99<15 and max(feedback_ages)<=.1 and set(states)=={1} and
             not runtime_faults and camera_timeout_count==0 and end_reason=="DURATION_COMPLETE" and
             all(row["ok"] for row in results) and len(observation_records)>0}
        (output/"dryrun_live_summary.json").write_text(json.dumps(summary,indent=2)+"\n")
        print(json.dumps(summary,indent=2))
        if not summary["pass"] and end_reason!="CAMERA_NO_FRAME_TIMEOUT":raise RuntimeError("DRYRUN_LIVE_ACCEPTANCE_FAILED")
    finally:
        if observer.is_alive():
            try:control_pipe.send({"type":"SHUTDOWN"})
            except (BrokenPipeError,EOFError):pass
            observer.join(timeout=2)
        if observer.is_alive():observer.terminate();observer.join(timeout=1)
        control_pipe.close();feedback_spin.stop();feedback_node.destroy_node();rclpy.shutdown()

if __name__=="__main__":main()
