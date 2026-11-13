#!/usr/bin/python3
"""Read-only 30 s check with repeated 0.45 s main-thread blocking."""
import argparse,json,sys,threading,time
from pathlib import Path
import numpy as np

ROOT=Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:sys.path.insert(0,str(ROOT))
from controller.geometry import ToolTransform
from kinematics import PyKDLKinematics
from runtime.camera import CameraFeedback
from runtime.config import load_runtime_config
from runtime.feedback import FeedbackSpinThread,RobotFeedback

def stats(values):
    a=np.asarray(values,float)
    return {"count":int(a.size),"p50":float(np.percentile(a,50)),
            "p99":float(np.percentile(a,99)),"max":float(np.max(a))} if a.size else None

def main():
    p=argparse.ArgumentParser();p.add_argument("--duration",type=float,default=30)
    p.add_argument("--block-s",type=float,default=.45)
    p.add_argument("--camera-topic",default="/zed/zed_node/rgb/color/rect/image")
    p.add_argument("--output",type=Path,required=True);args=p.parse_args()
    import rclpy
    rclpy.init();rc=load_runtime_config()
    feedback_node=rclpy.create_node("v2_feedback_thread_check_readonly")
    camera_node=rclpy.create_node("v2_feedback_thread_check_camera_readonly")
    feedback=RobotFeedback.from_ros_node(
      feedback_node,PyKDLKinematics(),ToolTransform.tool0_real(),rc)
    feedback_executor=rclpy.executors.SingleThreadedExecutor()
    warmup_start=time.monotonic()
    feedback_spin=FeedbackSpinThread(feedback_executor,feedback_node).start()
    camera=CameraFeedback.from_ros_node(camera_node,args.camera_topic,gap_log_threshold_s=.1)
    camera_executor=rclpy.executors.SingleThreadedExecutor();camera_executor.add_node(camera_node)
    camera_stop=threading.Event()
    def spin_camera():
        while not camera_stop.is_set():camera_executor.spin_once(timeout_sec=.02)
    camera_thread=threading.Thread(target=spin_camera,daemon=True);camera_thread.start()
    ages=[];samples=[]
    try:
        feedback.wait_for_samples_after(warmup_start,2.0);camera.wait_for_first_frame(2.0)
        start=time.monotonic()
        while time.monotonic()-start<args.duration:
            time.sleep(args.block_s)
            now=time.monotonic();diag=feedback.diagnostics(now)
            ages.append(diag["joint_state_age_s"]);samples.append({"time":now,**diag})
        elapsed=time.monotonic()-start
        with feedback.lock:poll_events=list(feedback.poll_events);joint_count=feedback._joint_messages
        with camera._lock:camera_intervals=list(camera.receive_intervals_s);camera_count=camera.receive_count
        state=[row for row in poll_events if row.get("service")=="state"]
        state_ok=[row for row in state if row.get("success")]
        result={"safety":"READ_ONLY_SUBSCRIPTIONS_AND_GET_SERVICES_ONLY",
          "duration_s":elapsed,"main_block_s":args.block_s,"main_block_iterations":len(samples),
          "joint_state_receive_count":joint_count,"joint_state_age_s":stats(ages),
          "robot_state_poll_attempts":len(state),"robot_state_poll_successes":len(state_ok),
          "robot_state_poll_success_rate":len(state_ok)/len(state) if state else 0.0,
          "robot_state_latency_s":stats([row["latency_s"] for row in state_ok]),
          "camera_receive_count":camera_count,"camera_interval_s":stats(camera_intervals),
          "camera_gap_over_0_1s_count":sum(x>.1 for x in camera_intervals),
          "camera_gap_over_1s_count":sum(x>1 for x in camera_intervals),
          "final_feedback_diagnostics":feedback.diagnostics(),"samples":samples}
        args.output.parent.mkdir(parents=True,exist_ok=True)
        args.output.write_text(json.dumps(result,indent=2)+"\n")
        print(json.dumps({k:v for k,v in result.items() if k!="samples"},indent=2))
    finally:
        camera_stop.set();camera_thread.join(timeout=1);camera_executor.remove_node(camera_node)
        feedback_spin.stop();camera_node.destroy_node();feedback_node.destroy_node();rclpy.shutdown()

if __name__=="__main__":main()
