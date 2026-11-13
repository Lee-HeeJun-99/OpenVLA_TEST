import json,sys,tempfile,unittest
from pathlib import Path
import numpy as np
ROOT=Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:sys.path.insert(0,str(ROOT))
from runtime.server_timing import serialize_action_unchanged
from runtime.system_monitor_launcher import SystemMonitorProcess
from tools.system_monitor import (RosReceiveMetrics,cpu_frequency_mhz,
  find_listening_pid,process_metrics,safe_metric,start_ros_executor_thread)
from tools.wandb_upload import build_upload


class TelemetryTests(unittest.TestCase):
    def test_timed_server_action_bytes_match_original_serialization(self):
        action=np.array([[.1,-.2,.3,-.4,.5,-.6,1.]],dtype=np.float32)
        # Original serve_adapter.py: float64 -> squeeze leading singleton -> tolist.
        old=np.asarray(action,dtype=np.float64)
        while old.ndim>1 and old.shape[0]==1:old=old[0]
        old_payload=json.dumps({"action":old.tolist()},separators=(",", ":")).encode()
        new_payload=json.dumps({"action":serialize_action_unchanged(action)},separators=(",", ":")).encode()
        self.assertEqual(new_payload,old_payload)

    def test_wandb_payload_skips_missing_optional_files_and_maps_metrics(self):
        with tempfile.TemporaryDirectory() as d:
            root=Path(d);(root/"summary.json").write_text(json.dumps({"end_reason":"DONE",
              "instruction":"Pick up the blue cube.","control_mode":"servoj_A",
              "segment_T_s":1.0,"servoj_time_s":.05}))
            step={"step":0,"action_raw":[.001,.002,.003,.01,.02,.03,1.],
              "gripper_binary":1,"ik_precheck_s":.1,"move_s":.2,"settle_s":.3,
              "step_total_s":.8,"path_sigma_min":.09,"arrival_joint_error_deg":[.01]*6,
              "command_delta":{"position_m":[.001,0,0],"rotation_vector_rad":[0,0,.01]},
              "actual_delta":{"position_m":[.0009,0,0],"rotation_vector_rad":[0,0,.009]},
              "target_joint_deg_j1_to_j6":[1,2,3,4,5,6]}
            (root/"steps.jsonl").write_text(json.dumps(step)+"\n")
            payload=build_upload(root)
            self.assertFalse(payload["files_found"]["predictions"])
            self.assertEqual(payload["summary"]["first_close_step"],0)
            self.assertAlmostEqual(payload["step_logs"][0]["ctrl/cmd_actual_pos_mm"],.1)
            self.assertEqual(payload["config"]["control_mode"],"servoj_A")
            self.assertEqual(payload["config"]["segment_T_s"],1.0)
            self.assertEqual(payload["config"]["servoj_time_s"],.05)

    def test_process_metrics_missing_pid_is_nonfatal(self):
        self.assertIsNone(process_metrics(None));self.assertIsNone(process_metrics(999999999))

    def test_cpu_frequency_is_normalized_to_mhz(self):
        from types import SimpleNamespace
        self.assertAlmostEqual(cpu_frequency_mhz(
          SimpleNamespace(current=3.599,min=800.0,max=5200.0)),3599.0)
        self.assertAlmostEqual(cpu_frequency_mhz(
          SimpleNamespace(current=3599.0,min=800.0,max=5200.0)),3599.0)
        self.assertIsNone(cpu_frequency_mhz(None))

    def test_ros_callbacks_only_queue_and_sampler_computes_both_intervals(self):
        from types import SimpleNamespace
        now=[10.0];clock=lambda:now[0]
        metrics=RosReceiveMetrics(None,clock=clock)
        def joint(stamp_s):
            sec=int(stamp_s);nanosec=int(round((stamp_s-sec)*1e9))
            return SimpleNamespace(header=SimpleNamespace(
              stamp=SimpleNamespace(sec=sec,nanosec=nanosec)))
        metrics.on_joint(joint(100.000));now[0]=10.011;metrics.on_joint(joint(100.010))
        now[0]=10.021;metrics.on_joint(joint(100.020))
        metrics.on_camera(None);now[0]=10.041;metrics.on_camera(None)
        # Callbacks only enqueue; interval state is untouched until snapshot.
        self.assertIsNone(metrics.joint_last_recv)
        row=metrics.snapshot(10.045)
        self.assertAlmostEqual(row["header_interval_max_ms"],10.0,places=6)
        self.assertAlmostEqual(row["recv_interval_max_ms"],11.0,places=6)
        self.assertAlmostEqual(row["joint_states_interval_max_ms"],11.0,places=6)
        self.assertAlmostEqual(row["joint_states_age_max_ms"],24.0,places=6)
        self.assertEqual(row["camera_hz"],2.0)

    def test_camera_rate_uses_only_recent_one_second_receives(self):
        now=[1.0];metrics=RosReceiveMetrics(None,clock=lambda:now[0])
        metrics.on_camera(None);now[0]=1.5;metrics.on_camera(None);now[0]=2.1;metrics.on_camera(None)
        self.assertEqual(metrics.snapshot(2.1)["camera_hz"],2.0)

    def test_ros_executor_spins_on_dedicated_thread(self):
        import threading
        main_ident=threading.get_ident()
        class Executor:
            def __init__(self):self.ident=None
            def spin(self):self.ident=threading.get_ident()
        executor=Executor();thread=start_ros_executor_thread(executor);thread.join(1.0)
        self.assertFalse(thread.is_alive());self.assertIsNotNone(executor.ident)
        self.assertNotEqual(executor.ident,main_ident)

    def test_ros_executor_exception_is_captured_not_raised(self):
        class Executor:
            def spin(self):raise RuntimeError("mock spin failure")
        errors=[];thread=start_ros_executor_thread(Executor(),errors);thread.join(1.0)
        self.assertFalse(thread.is_alive());self.assertEqual(len(errors),1)
        self.assertIn("mock spin failure",errors[0])

    def test_one_metric_failure_returns_default_and_next_metric_continues(self):
        errors=[]
        failed=safe_metric(errors,"broken",lambda:(_ for _ in ()).throw(RuntimeError("boom")))
        healthy=safe_metric(errors,"healthy",lambda:42)
        self.assertIsNone(failed);self.assertEqual(healthy,42)
        self.assertEqual(errors,[{"metric":"broken","error":"RuntimeError: boom"}])

    def test_monitor_death_is_warning_and_stop_is_idempotent(self):
        class FakeProcess:
            def __init__(self,*args,**kwargs):self.code=None;self.terminated=0
            def poll(self):return self.code
            def terminate(self):self.terminated+=1;self.code=0
            def wait(self,timeout=None):return self.code
            def kill(self):self.code=-9
        made=[]
        def popen(*args,**kwargs):
            proc=FakeProcess();made.append(proc);return proc
        with tempfile.TemporaryDirectory() as d:
            monitor=SystemMonitorProcess(ROOT,Path(d),1,2,"http://127.0.0.1:8766",
              "/camera",10.0,popen=popen)
            self.assertTrue(monitor.status()["running"])
            made[0].code=7
            for index in range(8):monitor.log.write(f"trace line {index}\n")
            status=monitor.status()
            self.assertTrue(status["warning"].startswith("SYSTEM_MONITOR_EXITED:7\n"))
            self.assertEqual(status["traceback_tail"],[f"trace line {i}" for i in range(3,8)])
            self.assertEqual(monitor.stop()["returncode"],7)
            self.assertEqual(made[0].terminated,0)

    def test_monitor_omits_none_pid_and_optional_camera(self):
        commands=[]
        class FakeProcess:
            def poll(self):return 0
            def wait(self,timeout=None):return 0
        def popen(command,**kwargs):commands.append(command);return FakeProcess()
        with tempfile.TemporaryDirectory() as d:
            monitor=SystemMonitorProcess(ROOT,Path(d),123,None,"http://127.0.0.1:8766",
              None,10.0,popen=popen);monitor.stop()
        self.assertNotIn("--obs-pid",commands[0]);self.assertNotIn("None",commands[0])
        self.assertNotIn("--camera-topic",commands[0])

if __name__=="__main__":unittest.main()
