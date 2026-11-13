import json
import multiprocessing as mp
import tempfile
import time
import unittest
from pathlib import Path
from types import SimpleNamespace
import numpy as np

from controller.config import load_config
from controller.geometry import ToolTransform
from kinematics import PyKDLKinematics
from runtime.config import load_runtime_config
from runtime.executor import V2Executor
from runtime.feedback import FeedbackSpinThread,RobotFeedback,FeedbackJsonlLogger
from runtime import robot_state as rs
from runtime.gate import ExecutionGate,GateEntry
from runtime.gripper_io import BinaryIOGripper,LevelIOGripper,create_gripper
from runtime.servoj import ServojPublisher
from runtime.stop import StopCoordinator
from runtime.camera import CameraFeedback
from runtime.observation_process import run_mock_blocking_observation
from runtime.policy_movej_core import (AbortSignal,MockableMoveJStep,MoveJPlan,
  PolicyMoveJPrecheckError,SettlementTracker,descent_limit_reason,
  validate_sigma_override)


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
        msg=SimpleNamespace(name=[f"joint_{i}" for i in range(1,7)],
            position=np.zeros(6),velocity=np.zeros(6),
            header=SimpleNamespace(stamp=SimpleNamespace(sec=12,nanosec=500000000)))
        feedback.joint_state_callback(msg)
        self.assertIn("JOINT_STATE_STALE",feedback.stale_reasons())
        feedback.joint_state_callback(msg)
        feedback._state=(clock(),1,.001);feedback._mode=(clock(),1);feedback._system=(clock(),0)
        self.assertEqual(feedback.stale_reasons(),[])
        snapshot=feedback.snapshot();self.assertEqual(snapshot.robot_state,1)
        self.assertEqual(snapshot.header_stamp_s,12.5)
        self.assertEqual(snapshot.log_record()["frame"],"tool0_real")
        with tempfile.TemporaryDirectory() as directory:
            row=FeedbackJsonlLogger(Path(directory)/"feedback.jsonl").write(snapshot)
            self.assertIn("tool0_real_rotation_matrix",row)
        clock.value=.26
        self.assertEqual(set(feedback.stale_reasons()),{"JOINT_STATE_STALE","ROBOT_STATE_STALE"})

    def test_feedback_spin_thread_updates_while_main_blocks(self):
        feedback=RobotFeedback(PyKDLKinematics(),ToolTransform.tool0_real())
        msg=SimpleNamespace(name=[f"joint_{i}" for i in range(1,7)],
            position=np.zeros(6),velocity=np.zeros(6),header=None)
        class Executor:
            def add_node(self,node):pass
            def remove_node(self,node):pass
            def spin_once(self,timeout_sec):
                feedback.joint_state_callback(msg);time.sleep(.01)
        spinner=FeedbackSpinThread(Executor(),object()).start()
        try:
            before=feedback._joint_messages
            time.sleep(.5)  # Simulated blocking HTTP/image work on the main thread.
            diag=feedback.diagnostics()
            self.assertGreater(feedback._joint_messages-before,20)
            self.assertLess(diag["joint_state_age_s"],.05)
        finally:spinner.stop()

    def test_observation_process_block_does_not_age_control_feedback(self):
        feedback=RobotFeedback(PyKDLKinematics(),ToolTransform.tool0_real())
        msg=SimpleNamespace(name=[f"joint_{i}" for i in range(1,7)],
            position=np.zeros(6),velocity=np.zeros(6),header=None)
        class Executor:
            def add_node(self,node):pass
            def remove_node(self,node):pass
            def spin_once(self,timeout_sec):feedback.joint_state_callback(msg);time.sleep(.01)
        spinner=FeedbackSpinThread(Executor(),object()).start()
        ctx=mp.get_context("spawn");control,observation=ctx.Pipe()
        process=ctx.Process(target=run_mock_blocking_observation,args=(observation,.5))
        process.start();observation.close();max_age=0.0
        try:
            self.assertEqual(control.recv()["type"],"READY")
            while process.is_alive():
                time.sleep(.02);max_age=max(max_age,feedback.diagnostics()["joint_state_age_s"])
            process.join();self.assertEqual(control.recv()["type"],"RESULT")
            self.assertLess(max_age,.05)
        finally:
            if process.is_alive():process.terminate();process.join()
            control.close();spinner.stop()

    def test_feedback_warmup_requires_samples_after_start(self):
        feedback=RobotFeedback(PyKDLKinematics(),ToolTransform.tool0_real())
        start=time.monotonic()
        with self.assertRaisesRegex(RuntimeError,"FEEDBACK_NO_FIRST_SAMPLE:joint_states,robot_state"):
            feedback.wait_for_samples_after(start,.01)

    def test_mixed_joint_names_are_normalized_and_command_order_is_j1_to_j6(self):
        clock=FakeClock();kin=PyKDLKinematics();feedback=RobotFeedback(
            kin,ToolTransform.tool0_real(),.1,clock)
        names=["joint_1","joint_2","joint_4","joint_5","joint_3","joint_6"]
        values=np.deg2rad([1,2,4,5,3,6])
        msg=SimpleNamespace(name=names,position=values,velocity=np.zeros(6),header=None)
        feedback.joint_state_callback(msg);feedback.joint_state_callback(msg)
        feedback._state=(clock(),1,.001);feedback._mode=(clock(),1);feedback._system=(clock(),0)
        np.testing.assert_allclose(np.rad2deg(feedback.snapshot().joint_rad),[1,2,3,4,5,6])
        class Pub:
            def __init__(self):self.messages=[]
            def publish(self,msg):self.messages.append(msg)
        rc=dict(self.rc,servoj_time_unverified=.01,
            servoj_velocity_unverified=1.,servoj_acceleration_unverified=2.)
        sender=ServojPublisher(Pub(),SimpleNamespace,rc,PassGate(),enabled=True)
        sender.publish(feedback.snapshot().joint_rad,clock())
        np.testing.assert_allclose(sender.publisher.messages[0].pos,[1,2,3,4,5,6])

    def test_joint_name_missing_and_duplicate_are_immediate_faults(self):
        feedback=RobotFeedback(PyKDLKinematics(),ToolTransform.tool0_real())
        feedback.joint_state_callback(SimpleNamespace(name=["joint_1"]*6,
            position=np.zeros(6),velocity=np.zeros(6),header=None))
        self.assertIn("JOINT_STATE_DUPLICATE_NAME",feedback.stale_reasons())
        feedback=RobotFeedback(PyKDLKinematics(),ToolTransform.tool0_real())
        feedback.joint_state_callback(SimpleNamespace(name=["joint_1","joint_2"],
            position=np.zeros(2),velocity=np.zeros(2),header=None))
        self.assertIn("JOINT_STATE_MISSING_NAME",feedback.stale_reasons())

    def test_error_and_disconnection_topics_are_immediate_faults(self):
        feedback=RobotFeedback(PyKDLKinematics(),ToolTransform.tool0_real())
        event=SimpleNamespace(level=3,group=1,code=99,msg1="x",msg2="",msg3="")
        feedback.error_callback(event);feedback.disconnection_callback(event)
        reasons=feedback.stale_reasons()
        self.assertIn("ROBOT_ERROR_TOPIC",reasons)
        self.assertIn("ROBOT_DISCONNECTION_TOPIC",reasons)

    def test_preflight_requires_autonomous_real_standby(self):
        feedback=RobotFeedback(PyKDLKinematics(),ToolTransform.tool0_real())
        msg=SimpleNamespace(name=[f"joint_{i}" for i in range(1,7)],
            position=np.zeros(6),velocity=np.zeros(6),header=None)
        feedback.joint_state_callback(msg);feedback.joint_state_callback(msg)
        now=feedback.clock();feedback._state=(now,2,.001);feedback._mode=(now,0);feedback._system=(now,1)
        reasons,_=feedback.preflight_reasons(now)
        self.assertIn("ROBOT_MODE_NOT_AUTONOMOUS:0",reasons)
        self.assertIn("ROBOT_SYSTEM_NOT_REAL:1",reasons)
        self.assertIn("ROBOT_STATE_NOT_STANDBY:2",reasons)

    def test_state_poll_failure_is_recorded_and_state_is_stale(self):
        class Service:
            class Request:pass
        class Client:
            srv_type=Service
            def service_is_ready(self):return False
        feedback=RobotFeedback(PyKDLKinematics(),ToolTransform.tool0_real())
        feedback.clients["state"]=Client();feedback.poll_robot_state()
        self.assertEqual(feedback.poll_events[-1]["error"],"SERVICE_NOT_READY")
        self.assertIn("ROBOT_STATE_STALE",feedback.stale_reasons())

    def test_canonical_robot_state_definition(self):
        self.assertEqual((rs.RECOVERY,rs.SAFE_STOP2,rs.SAFE_OFF2),(8,9,10))
        self.assertEqual(rs.CONTROLLER_STOP_STATES,{3,5,6,9,10})

    def test_camera_new_frame_after_settle_and_no_action_repeat(self):
        clock=FakeClock(); camera=CameraFeedback(.1,clock)
        stamp=SimpleNamespace(sec=10,nanosec=100)
        camera.callback(SimpleNamespace(height=1,width=1,step=3,encoding="bgr8",
                                        data=bytes([1,2,3]),header=SimpleNamespace(stamp=stamp)))
        self.assertEqual(camera.wait_for_first_frame(.01),0.0)
        rgb,_=camera.latest(); self.assertEqual(rgb.tolist(),[[[3,2,1]]])
        status,frame=camera.poll_frame_after(-.1,10_000_000_000,.5,now=0)
        self.assertEqual(status,"FRAME");self.assertEqual(frame["receive_count"],1)
        # The same frame cannot trigger another inference/action cycle.
        status,frame=camera.poll_frame_after(0.0,10_000_000_100,.5,now=.1)
        self.assertEqual((status,frame),("WAITING",None))

    def test_camera_wait_timeout_is_normal_end_status_and_gap_is_log_only(self):
        clock=FakeClock();camera=CameraFeedback(.1,clock)
        with self.assertRaisesRegex(RuntimeError,"CAMERA_NO_FIRST_FRAME:timeout_s=0.001"):
            camera.wait_for_first_frame(.001)
        msg=SimpleNamespace(height=1,width=1,step=4,encoding="bgra8",data=bytes([1,2,3,255]),
                            header=SimpleNamespace(stamp=SimpleNamespace(sec=1,nanosec=0)))
        camera.callback(msg);clock.value=.04;camera.callback(msg);clock.value=.15
        diag=camera.diagnostics()
        self.assertAlmostEqual(diag["previous_receive_interval_s"],.04)
        self.assertEqual(diag["receive_count"],2)
        # Age > stale_s is no longer a continuous fault.
        self.assertEqual(camera.latest()[0].tolist(),[[[3,2,1]]])
        status,frame=camera.poll_frame_after(.15,2_000_000_000,10.0,now=10.15)
        self.assertEqual((status,frame),("CAMERA_NO_FRAME_TIMEOUT",None))

    def test_instruction_allowlist_requires_explicit_ood_flag(self):
        from tools.dryrun_live import validate_instruction
        self.assertTrue(validate_instruction("Pick up the blue cube."))
        with self.assertRaisesRegex(ValueError,"OOD_INSTRUCTION_REQUIRES_EXPLICIT_FLAG"):
            validate_instruction("Pick up the orange cube.")
        self.assertFalse(validate_instruction("Pick up the orange cube.",True))

    def _stop_case(self,ack,states,velocities):
        clock=FakeClock()
        class Servo:
            def __init__(self): self.inhibited=False
            def inhibit(self): self.inhibited=True
        class Feedback:
            def __init__(self): self.i=0
            def poll_robot_state(self):self.polls=getattr(self,"polls",0)+1
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
        result,coordinator,servo=self._stop_case(True,[1]*8,[.1,0,0,0,0,0])
        self.assertEqual(result["state"],"STOP_CONFIRMED");self.assertTrue(servo.inhibited)
        self.assertGreater(coordinator.feedback.polls,0)

    def test_stop_ack_failure_safe_stop_preceded(self):
        result,_,_=self._stop_case(RuntimeError("ack"),[9]*8,[0]*8)
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
            entries={name:GateEntry(True,str(evidence),"photo") for name in
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
            self.assertTrue(V2Executor(gate,physical_enabled=True).authorize(
                "stop_test_movej","I_CONFIRM_V2_MODE:stop_test_movej"))

    def test_operator_attestation_evidence_type_and_schema(self):
        with tempfile.TemporaryDirectory() as directory:
            path=Path(directory)/"attestation.json"
            path.write_text(json.dumps({"items":[{"item":"limits","value":"default"}],
                "verification_method":"펜던트 화면 직접 확인","date":"2026-11-13",
                "operator":"tester","photo_omission_reason":"operator decision"}))
            gate=ExecutionGate({name:GateEntry(True,str(path),"operator_attestation")
                for name in ("pendant_limits_recorded","live_topics_verified",
                              "feedback_rate_verified")})
            gate.require("stop_test_movej")
            bad=ExecutionGate({"pendant_limits_recorded":GateEntry(True,str(path),"scan")})
            self.assertIn("pendant_limits_recorded:evidence_type_invalid",
                          bad.audit("stop_test_movej"))

    def test_measurement_gate_and_field_test_decisions(self):
        from tools.gripper_test import require_operator_polarity_confirmation
        from tools.stop_test_movej import expected_profile,stop_trigger_status
        with tempfile.TemporaryDirectory() as directory:
            evidence=Path(directory)/"measurement.json";evidence.write_text("{}")
            gate=ExecutionGate({"live_topics_verified":GateEntry(
                True,str(evidence),"measurement")})
            self.assertEqual(gate._entry_failures("live_topics_verified"),[])
        self.assertTrue(require_operator_polarity_confirmation(True))
        with self.assertRaisesRegex(RuntimeError,"POLARITY_MISMATCH"):
            require_operator_polarity_confirmation(False)
        profile=expected_profile()
        self.assertEqual(profile["shape"],"triangular")
        self.assertAlmostEqual(profile["peak_velocity_deg_s"],np.sqrt(20))
        self.assertEqual(stop_trigger_status(profile["peak_velocity_deg_s"]*.5,.1,profile),"THRESHOLD")
        self.assertEqual(stop_trigger_status(0,profile["invalid_deadline_s"],profile),"INVALID_TIMEOUT")

    def test_mode_specific_gate_contracts(self):
        with tempfile.TemporaryDirectory() as directory:
            evidence=Path(directory)/"e.json";evidence.write_text("{}")
            passed=lambda names: ExecutionGate({name:GateEntry(True,str(evidence),"photo")
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
            movej=movej+("stop_movej_verified",)
            passed(movej).require("policy_movej")
            self.assertNotIn("servoj_time_param_verified:not_passed",
                             passed(movej).audit("policy_movej"))

    def test_policy_movej_precheck_failure_never_calls_move_joint(self):
        class Reject:
            def plan(self,*args):raise PolicyMoveJPrecheckError(["SINGULARITY_APPROACH"])
        class Move:
            def __init__(self):self.calls=[]
            def move_joint(self,**kwargs):self.calls.append(kwargs)
        move=Move();step=MockableMoveJStep(Reject(),move,lambda reason:reason,
                                           self.rc["policy_movej"])
        with self.assertRaisesRegex(PolicyMoveJPrecheckError,"SINGULARITY"):
            step.start(None,None,None,None)
        self.assertEqual(move.calls,[])

    def test_policy_movej_settlement_requires_velocity_position_and_hold(self):
        tracker=SettlementTracker(.001,.3,.1);target=np.zeros(6)
        self.assertFalse(tracker.update(target,np.zeros(6),target,1.0))
        self.assertFalse(tracker.update(target,np.ones(6)*.002,target,1.2))
        self.assertFalse(tracker.update(target,np.zeros(6),target,2.0))
        self.assertTrue(tracker.update(target,np.zeros(6),target,2.31))
        self.assertFalse(SettlementTracker(.001,.3,.1).update(
            np.deg2rad(np.ones(6)*.11),np.zeros(6),target,3.0))

    def test_policy_movej_j1_to_j6_order_abort_and_gripper_record_only(self):
        target=np.deg2rad([1,2,3,4,5,6])
        plan=MoveJPlan(target,None,None,(.2,),.5,{"outside":False})
        class Planner:
            def plan(self,*args):return plan
        class Move:
            def __init__(self):self.calls=[]
            def move_joint(self,**kwargs):self.calls.append(kwargs)
        stopped=[];move=Move();step=MockableMoveJStep(
            Planner(),move,lambda reason:stopped.append(reason),self.rc["policy_movej"])
        step.start(None,None,None,None);step.operator_abort()
        np.testing.assert_allclose(move.calls[0]["pos"],[1,2,3,4,5,6],atol=1e-12)
        self.assertEqual(move.calls[0]["sync_type"],1);self.assertEqual(stopped,["OPERATOR_ABORT"])
        # policy_movej has no gripper transport: the state tracker is log-only.
        from controller.precheck import GripperStateTracker
        tracker=GripperStateTracker();self.assertFalse(tracker.update(0));self.assertTrue(tracker.update(1))

    def test_policy_movej_full_rollout_defaults_and_table_confirmation(self):
        from tools.policy_movej import require_table_watch_confirmation
        pc=self.rc["policy_movej"]
        self.assertEqual(pc["max_steps_default"],300)
        self.assertEqual(pc["max_episode_time_s_candidate"],600.0)
        self.assertEqual(require_table_watch_confirmation("TABLE_WATCH_BY_OPERATOR"),
                         "TABLE_WATCH_BY_OPERATOR")
        with self.assertRaisesRegex(PermissionError,"TABLE_WATCH"):
            require_table_watch_confirmation("yes")

    def test_policy_movej_sigma_override_audit(self):
        default=.0803
        self.assertEqual(validate_sigma_override(default)["value"],default)
        raised=validate_sigma_override(default,.09,None)
        self.assertFalse(raised["lowered"]);self.assertIsNone(raised["reason"])
        with self.assertRaisesRegex(ValueError,"REQUIRES_REASON"):
            validate_sigma_override(default,.07,None)
        lowered=validate_sigma_override(default,.07,"offline replay review")
        self.assertTrue(lowered["lowered"]);self.assertEqual(lowered["reason"],"offline replay review")

    def test_policy_movej_optional_descent_limit_includes_path_point(self):
        self.assertIsNone(descent_limit_reason(.7,.681,20,7))
        self.assertEqual(descent_limit_reason(.7,.679,20,7),"PATH_7:DESCENT_LIMIT")
        self.assertEqual(descent_limit_reason(.7,.680,20,8),"PATH_8:DESCENT_LIMIT")
        self.assertIsNone(descent_limit_reason(.7,.1,None,7))

    def test_policy_movej_operator_abort_response_timestamp(self):
        clock=FakeClock();signal=AbortSignal(clock)
        clock.value=10.;signal.request();clock.value=10.004
        self.assertTrue(signal.is_set());self.assertAlmostEqual(signal.response_s(clock()),.004)

    def test_policy_movej_instruction_allowlist(self):
        from tools.dryrun_live import validate_instruction
        self.assertTrue(validate_instruction("Pick up the blue cube."))
        with self.assertRaisesRegex(ValueError,"OOD_INSTRUCTION"):
            validate_instruction("Pick up the orange cube.")

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

    def test_level_gripper_interlock_readback_and_state_changes(self):
        class IO:
            def __init__(self):self.state={1:0,2:0};self.writes=[]
            def set_tool_digital_output(self,index,value):
                self.state[index]=value;self.writes.append((index,value,dict(self.state)))
                return {"success":True}
            def get_tool_digital_output(self,index):return {"success":True,"value":self.state[index]}
        io=IO();gripper=LevelIOGripper(io,PassGate(),self.rc,enabled=True)
        opened=gripper.command_model_value(.1);duplicate=gripper.command_model_value(.2)
        closed=gripper.command_model_value(.9)
        self.assertEqual([(w["index"],w["value"]) for w in opened["writes"]],[(2,0),(1,1)])
        self.assertEqual([(w["index"],w["value"]) for w in closed["writes"]],[(1,0),(2,1)])
        self.assertEqual(duplicate["state"],"suppressed")
        self.assertFalse(any(snapshot[1]==1 and snapshot[2]==1 for _,_,snapshot in io.writes))
        self.assertEqual(io.state,{1:0,2:1})

    def test_level_gripper_readback_mismatch_faults_and_turns_both_off(self):
        class IO:
            def __init__(self):self.state={1:0,2:0}
            def set_tool_digital_output(self,index,value):self.state[index]=value;return {"success":True}
            def get_tool_digital_output(self,index):return {"success":True,"value":1}
        io=IO();event=LevelIOGripper(io,PassGate(),self.rc,enabled=True).command_model_value(.1)
        self.assertEqual(event["state"],"fault")
        self.assertIn("GRIPPER_OFF_READBACK_MISMATCH",event["reason"])
        self.assertEqual(io.state,{1:0,2:0})

    def test_level_gripper_policy_is_record_only_when_disabled(self):
        class IO:
            def set_tool_digital_output(self,**kwargs):raise AssertionError("must not write")
            def get_tool_digital_output(self,**kwargs):raise AssertionError("must not read")
        event=LevelIOGripper(IO(),PassGate(),self.rc,enabled=False).command_model_value(.9)
        self.assertEqual(event["state"],"record_only")
        with self.assertRaisesRegex(PermissionError,"gripper_io_verified"):
            LevelIOGripper(IO(),ExecutionGate(),self.rc,enabled=True,
                           execution_mode="policy")

    def test_gripper_factory_defaults_to_level(self):
        self.assertIsInstance(create_gripper(object(),PassGate(),self.rc),LevelIOGripper)


if __name__=="__main__": unittest.main()
