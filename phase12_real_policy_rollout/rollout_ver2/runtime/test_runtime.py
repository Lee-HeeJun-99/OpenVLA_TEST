import json
import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace
import numpy as np

from controller.config import load_config
from controller.geometry import ToolTransform
from kinematics import PyKDLKinematics
from runtime.config import load_runtime_config
from runtime.executor import V2Executor
from runtime.feedback import RobotFeedback,FeedbackJsonlLogger,RobotStateFullRateLogger
from runtime.gate import ExecutionGate,GateEntry
from runtime.gripper_io import BinaryIOGripper
from runtime.servoj import ServojPublisher
from runtime.stop import StopCoordinator
from runtime.camera import CameraFeedback


class FakeClock:
    def __init__(self): self.value=0.0
    def __call__(self): return self.value
    def sleep(self,seconds): self.value+=seconds


class PassGate:
    def __init__(self): self.modes=[]
    def require(self,mode): self.modes.append(mode)


class RuntimeTests(unittest.TestCase):
    def setUp(self): self.rc=load_runtime_config()

    def test_servoj_publish_deg_mode_zero_and_inhibit(self):
        class Pub:
            def __init__(self): self.messages=[]
            def publish(self,msg): self.messages.append(msg)
        rc=dict(self.rc,servoj_time_unverified=.01,
                servoj_velocity_unverified=1.,servoj_acceleration_unverified=2.)
        pub=Pub(); gate=PassGate()
        sender=ServojPublisher(pub,SimpleNamespace,rc,gate,enabled=True)
        sender.publish(np.deg2rad([1,2,3,4,5,6]),1.0)
        np.testing.assert_allclose(pub.messages[0].pos,[1,2,3,4,5,6],atol=1e-12)
        self.assertEqual(pub.messages[0].vel,[1.]*6)
        self.assertEqual(pub.messages[0].acc,[2.]*6)
        self.assertEqual(pub.messages[0].mode,0)
        self.assertEqual(gate.modes,["policy"])
        sender.inhibit()
        with self.assertRaises(PermissionError): sender.publish(np.zeros(6),2.)

    def test_feedback_subscription_callbacks_and_stale(self):
        clock=FakeClock(); kin=PyKDLKinematics()
        feedback=RobotFeedback(kin,ToolTransform.tool0_real(),.1,clock)
        feedback.joint_state_callback(SimpleNamespace(
            name=[f"joint_{i}" for i in range(1,7)],position=np.zeros(6),velocity=np.zeros(6)))
        feedback.robot_state_callback(SimpleNamespace(robot_state=1,actual_bk=[0]*6))
        self.assertEqual(feedback.stale_reasons(),[])
        snapshot=feedback.snapshot();self.assertEqual(snapshot.robot_state,1)
        self.assertEqual(snapshot.log_record()["frame"],"tool0_real")
        with tempfile.TemporaryDirectory() as directory:
            row=FeedbackJsonlLogger(Path(directory)/"feedback.jsonl").write(snapshot)
            self.assertIn("tool0_real_rotation_matrix",row)
        clock.value=.11
        self.assertEqual(set(feedback.stale_reasons()),{"JOINT_STATE_STALE","ROBOT_STATE_STALE"})

    def test_robot_state_full_rate_logger_keeps_posx_for_delta_analysis(self):
        clock=FakeClock();kin=PyKDLKinematics()
        with tempfile.TemporaryDirectory() as directory:
            path=Path(directory)/"robot_state_full_rate.jsonl"
            feedback=RobotFeedback(kin,ToolTransform.tool0_real(),.1,clock,
                                   RobotStateFullRateLogger(path))
            msg=SimpleNamespace(robot_state=2,actual_bk=[0]*6,
                current_posj=[1]*6,current_velj=[2]*6,target_posj=[3]*6,
                current_posx=[4]*6,current_tool_posx=[5]*6,
                rotation_matrix=[[1,0,0],[0,1,0],[0,0,1]],
                f_current_pos_world=[[6]*6,[7]*6],joint_err=[8]*6,
                task_err=[9]*6,sync_time=10.0)
            feedback.robot_state_callback(msg)
            row=json.loads(path.read_text())
            self.assertEqual(row["current_posx"],[4]*6)
            self.assertEqual(row["f_current_pos_world"][1],[7]*6)

    def test_camera_bgr_decode_and_stale(self):
        clock=FakeClock(); camera=CameraFeedback(.1,clock)
        camera.callback(SimpleNamespace(height=1,width=1,step=3,encoding="bgr8",
                                        data=bytes([1,2,3]),header=None))
        rgb,_=camera.latest(); self.assertEqual(rgb.tolist(),[[[3,2,1]]])
        clock.value=.11
        with self.assertRaisesRegex(RuntimeError,"CAMERA_STALE"): camera.latest()

    def _stop_case(self,ack,states,velocities):
        clock=FakeClock()
        class Servo:
            def __init__(self): self.inhibited=False
            def inhibit(self): self.inhibited=True
        class Feedback:
            def __init__(self): self.i=0
            def snapshot(self,now):
                i=min(self.i,len(velocities)-1);self.i+=1
                return SimpleNamespace(joint_velocity_rad_s=np.ones(6)*velocities[i],
                                       robot_state=states[min(i,len(states)-1)])
        class Stop:
            def move_stop(self,stop_mode):
                if isinstance(ack,Exception): raise ack
                return {"success":ack}
        rc=dict(self.rc,zero_velocity_hold_s_candidate=.03,
                stop_confirmation_timeout_s_candidate=.08)
        servo=Servo(); coordinator=StopCoordinator(servo,Feedback(),Stop(),rc,clock,clock.sleep)
        return coordinator.stop("test"),coordinator,servo

    def test_stop_normal(self):
        result,_,servo=self._stop_case(True,[1]*8,[.1,0,0,0,0,0])
        self.assertEqual(result["state"],"STOP_CONFIRMED");self.assertTrue(servo.inhibited)

    def test_stop_ack_failure_safe_stop_preceded(self):
        result,_,_=self._stop_case(RuntimeError("ack"),[10]*8,[0]*8)
        self.assertEqual(result["state"],"STOP_CONFIRMED")
        self.assertTrue(result["safe_state_preceded_ack_failure"])

    def test_stop_ack_failure_without_safe_state_still_velocity_confirmed(self):
        result,_,_=self._stop_case(RuntimeError("ack"),[1]*8,[0]*8)
        self.assertEqual(result["state"],"STOP_CONFIRMATION_FAILED")

    def test_stop_velocity_not_zero_permanently_inhibits(self):
        result,coordinator,_=self._stop_case(True,[1]*20,[.1]*20)
        self.assertEqual(result["state"],"STOP_CONFIRMATION_FAILED")
        self.assertTrue(coordinator.permanently_inhibited)

    def test_gate_requires_evidence_and_executor_confirmation(self):
        with tempfile.TemporaryDirectory() as directory:
            evidence=Path(directory)/"e.json";evidence.write_text("{}")
            entries={name:GateEntry(True,str(evidence)) for name in
                     ("pendant_limits_recorded","live_topics_verified",
                      "feedback_rate_verified")}
            gate=ExecutionGate(entries); gate.require("stop_test_movej")
            rc=dict(self.rc);rc["stop_test_movej"]=dict(rc["stop_test_movej"],
                acceleration_deg_s2_unverified=5.0)
            executor=V2Executor(gate,physical_enabled=True,runtime_config=rc)
            with self.assertRaises(PermissionError): executor.authorize("stop_test_movej","yes")
            self.assertTrue(executor.authorize(
                "stop_test_movej","I_CONFIRM_V2_MODE:stop_test_movej"))
            with self.assertRaises(PermissionError): ExecutionGate().require("stop_test_movej")
            with self.assertRaisesRegex(PermissionError,"V2_SETTINGS_UNVERIFIED"):
                V2Executor(gate,physical_enabled=True).authorize(
                    "stop_test_movej","I_CONFIRM_V2_MODE:stop_test_movej")

    def test_mode_specific_gate_contracts(self):
        with tempfile.TemporaryDirectory() as directory:
            evidence=Path(directory)/"e.json";evidence.write_text("{}")
            passed=lambda names: ExecutionGate({name:GateEntry(True,str(evidence))
                                                for name in names})
            gripper=("pendant_limits_recorded","live_topics_verified",
                     "gripper_wiring_recorded","arm_stationary_verified")
            passed(gripper).require("gripper_test")
            self.assertIn("gripper_wiring_recorded:not_passed",
                          ExecutionGate().audit("gripper_test"))
            movej=("pendant_limits_recorded","live_topics_verified","feedback_rate_verified")
            gate=passed(movej);gate.require("stop_test_movej")
            self.assertNotIn("servoj_time_param_verified:not_passed",
                             gate.audit("stop_test_movej"))
            servoj=movej+("stop_movej_verified","servoj_time_param_verified",
                          "tracking_thresholds_calibrated")
            passed(servoj).require("stop_test_servoj")
            self.assertIn("stop_servoj_verified:not_passed",passed(servoj).audit("synthetic"))
            self.assertIn("stop_servoj_verified:not_passed",passed(servoj).audit("policy"))
            synthetic=servoj+("stop_servoj_verified",)
            passed(synthetic).require("synthetic")
            passed(synthetic).require("policy")

    def test_stop_uses_configured_common_mode(self):
        result,_,_=self._stop_case(True,[1]*8,[.1,0,0,0,0,0])
        self.assertEqual(result["stop_mode"]["value"],1)
        self.assertEqual(result["stop_mode"]["name"],"DR_QSTOP")

    def test_gripper_only_state_changes_and_exact_pulses(self):
        clock=FakeClock()
        class IO:
            def __init__(self): self.calls=[]
            def set_tool_digital_output(self,**kw): self.calls.append(kw);return {"success":True}
        io=IO(); gripper=BinaryIOGripper(io,PassGate(),self.rc,enabled=True,
                                         clock=clock,sleep=clock.sleep)
        opened=gripper.command_model_value(.1)
        duplicate=gripper.command_model_value(.2)
        closed=gripper.command_model_value(.9)
        self.assertEqual([(x["index"],x["value"]) for x in opened["writes"]],
                         [(1,1),(1,0),(1,1),(1,0)])
        self.assertEqual(duplicate["state"],"suppressed")
        self.assertEqual([(x["index"],x["value"]) for x in closed["writes"]],[(2,1),(2,0)])
        self.assertAlmostEqual(clock.value,2.5)


if __name__=="__main__": unittest.main()
