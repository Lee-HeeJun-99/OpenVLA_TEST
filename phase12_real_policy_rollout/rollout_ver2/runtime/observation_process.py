"""Observation-only subprocess. It has no robot feedback or command imports."""
from __future__ import annotations
import os
import threading
import time


def run_mock_blocking_observation(connection,block_s=0.5):
    """Test-only process target representing expensive observation work."""
    connection.send({"type":"READY"});time.sleep(float(block_s))
    connection.send({"type":"RESULT"});connection.close()


def run_observation_process(connection,topic,server,instruction,output_dir,stats_path,
                            gap_threshold_s=.1,frame_timeout_s=10.0):
    """Own camera/JPEG/HTTP work in a process isolated from the safety process."""
    import rclpy
    from controller.action import ActionInterpreter
    from runtime.camera import CameraFeedback
    from runtime.http_client import PredictClient
    from runtime.observation_runtime import ObservationRuntime

    parent_pid=os.getppid();parent_dead=threading.Event();stop_watchdog=threading.Event()
    def watch_parent():
        while not stop_watchdog.wait(.05):
            if os.getppid()!=parent_pid:
                parent_dead.set();return
    watchdog=threading.Thread(target=watch_parent,daemon=True);watchdog.start()
    rclpy.init();node=rclpy.create_node("v2_observation_process_readonly")
    camera=CameraFeedback.from_ros_node(node,topic,gap_log_threshold_s=gap_threshold_s)
    executor=rclpy.executors.SingleThreadedExecutor();executor.add_node(node)
    spin_stop=threading.Event()
    def spin_camera():
        while not spin_stop.is_set():executor.spin_once(timeout_sec=.02)
    camera_thread=threading.Thread(target=spin_camera,daemon=True);camera_thread.start()
    observation=ObservationRuntime(PredictClient(server),ActionInterpreter(stats_path),output_dir)
    connection.send({"type":"READY","pid":os.getpid()})
    try:
        while not parent_dead.is_set():
            if not connection.poll(.05):continue
            request=connection.recv()
            if request.get("type")=="SHUTDOWN":
                connection.send({"type":"STOPPED","camera_diagnostics":camera.diagnostics()});break
            if request.get("type")!="OBSERVE":continue
            settle_host=float(request["t_settle_host_monotonic"])
            settle_ros_ns=int(request["t_settle_ros_ns"]);deadline=settle_host+float(frame_timeout_s)
            frame=None
            while time.monotonic()<deadline and not parent_dead.is_set():
                status,frame=camera.poll_frame_after(settle_host,settle_ros_ns,frame_timeout_s)
                if status=="FRAME":break
                time.sleep(.005)
            if parent_dead.is_set():break
            if frame is None:
                connection.send({"type":"CAMERA_NO_FRAME_TIMEOUT",
                  "camera_diagnostics":camera.diagnostics()});continue
            frame_record={"t_settle_host_monotonic":settle_host,"t_settle_ros_ns":settle_ros_ns,
              "frame_host_received_monotonic":frame["host_received_monotonic"],
              "frame_header_stamp_ns":frame["header_stamp_ns"],
              "frame_wait_s":frame["host_received_monotonic"]-settle_host,
              "camera_receive_count":frame["receive_count"]}
            try:
                action=observation.predict(frame["rgb"],instruction,frame_record)
                connection.send({"type":"RESULT","ok":True,"frame_record":frame_record,
                  "action_raw":action.raw.tolist(),"prediction_record":observation.last_record,
                  "camera_diagnostics":camera.diagnostics()})
            except Exception as error:
                connection.send({"type":"RESULT","ok":False,"error":str(error),
                  "frame_record":frame_record,"camera_diagnostics":camera.diagnostics()})
    except (EOFError,BrokenPipeError):
        pass
    finally:
        spin_stop.set();camera_thread.join(timeout=1);executor.remove_node(node);node.destroy_node()
        rclpy.shutdown();stop_watchdog.set();watchdog.join(timeout=.2);connection.close()
