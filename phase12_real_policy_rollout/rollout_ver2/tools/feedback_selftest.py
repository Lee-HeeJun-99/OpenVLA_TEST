#!/usr/bin/python3
"""30 s read-only feedback self-test. Creates no command publisher/client."""
import argparse,json,sys,time
from pathlib import Path
import numpy as np

ROOT=Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:sys.path.insert(0,str(ROOT))
from controller.geometry import ToolTransform
from kinematics import PyKDLKinematics
from runtime.config import load_runtime_config
from runtime.feedback import RobotFeedback

def stats(values):
    a=np.asarray(values,float)
    if not len(a):return None
    return {"count":int(len(a)),"mean":float(np.mean(a)),"std":float(np.std(a)),
            "p50":float(np.percentile(a,50)),"p99":float(np.percentile(a,99)),
            "max":float(np.max(a))}

def main():
    parser=argparse.ArgumentParser();parser.add_argument("--duration",type=float,default=30)
    parser.add_argument("--output",type=Path,required=True);args=parser.parse_args()
    import rclpy
    rclpy.init();node=rclpy.create_node("v2_feedback_readonly_selftest")
    feedback=RobotFeedback.from_ros_node(node,PyKDLKinematics(),ToolTransform.tool0_real(),load_runtime_config())
    warmup_start=time.monotonic();transient_first_excluded=None
    try:
        while time.monotonic()-warmup_start<60:
            rclpy.spin_once(node,timeout_sec=.02)
            if feedback.joint_events:
                transient_first_excluded=not feedback.joint_events[0]["freshness_eligible"]
            if (feedback._joint_fresh and feedback._state is not None and
                feedback._mode is not None and feedback._system is not None and
                all(client.service_is_ready() for client in feedback.clients.values())):
                break
        else:raise RuntimeError("READ_ONLY_FEEDBACK_DISCOVERY_TIMEOUT")
        warmup_s=time.monotonic()-warmup_start
        feedback.joint_events.clear();feedback.poll_events.clear()
        feedback.posj_comparisons.clear();feedback.flange_comparisons.clear()
        start=time.monotonic()
        while time.monotonic()-start<args.duration:rclpy.spin_once(node,timeout_sec=.02)
    finally:
        elapsed=time.monotonic()-start if 'start' in locals() else 0
        node.destroy_node();rclpy.shutdown()
    joint=feedback.joint_events
    host=np.diff([row["host_monotonic"] for row in joint])
    header_values=[row["header_stamp_s"] for row in joint if row["header_stamp_s"] is not None]
    header=np.diff(header_values)
    state_events=[row for row in feedback.poll_events if row["service"]=="state"]
    state_ok=[row for row in state_events if row.get("success")]
    state_times=np.diff([row["completed"] for row in state_ok])
    state_latency=[row["latency_s"] for row in state_ok]
    flange=feedback.flange_comparisons;posj=feedback.posj_comparisons
    summary={"safety":"READ_ONLY_GET_AND_SUBSCRIBE_ONLY","warmup_s":warmup_s,
      "duration_s":elapsed,"joint_messages":len(joint),
      "initial_transient_local_sample_excluded":transient_first_excluded,
      "joint_received_name_order":joint[-1]["received_names"] if joint else None,
      "joint_host_period_s":stats(host),"joint_header_period_s":stats(header),
      "robot_state_poll_attempts":len(state_events),"robot_state_poll_success":len(state_ok),
      "robot_state_poll_failures":len(state_events)-len(state_ok),
      "robot_state_period_s":stats(state_times),"robot_state_latency_s":stats(state_latency),
      "posj_comparison_count":len(posj),
      "posj_max_abs_difference_deg":float(np.max(np.abs([r["difference_deg"] for r in posj]))) if posj else None,
      "flange_comparison_count":len(flange),
      "flange_position_error_mm":stats([r["position_error_mm"] for r in flange]),
      "flange_rotation_error_deg":stats([r["rotation_error_deg"] for r in flange]),
      "flange_service_span_s":stats([r["service_span_s"] for r in flange]),
      "preflight_reasons":feedback.preflight_reasons()[0],"faults":feedback._faults,
      "latest_snapshot":feedback.snapshot().log_record() if joint else None,
      "source":{"joint_events":joint,"poll_events":feedback.poll_events,
                "posj_comparisons":posj,"flange_comparisons":flange}}
    args.output.parent.mkdir(parents=True,exist_ok=True)
    args.output.write_text(json.dumps(summary,indent=2)+"\n")
    print(json.dumps({key:value for key,value in summary.items() if key!="source"},indent=2))

if __name__=="__main__":main()
