import unittest
import json,tempfile
from pathlib import Path
from types import SimpleNamespace
import numpy as np
from controller.config import load_config
from controller.geometry import ToolTransform
from kinematics import PyKDLKinematics
from runtime.servoj import (FixedSegmentPlanner,ServoJPrecheckError,ServoStreamRunner,
  TrackingMonitor,ServoJModeACore,HoldRunner,run_hold_if_preflight,write_hold_outputs)
from runtime.servoj import TimeExploreRunner,TimeExploreFault,StopServoJRunner
from runtime.servoj import PolicyServoJRunner,ModeAObservationLatch
from runtime.servoj.tracking import tracking_threshold_deg,validate_servoj_time
from runtime.servoj.session_contract import OperatorAbort,require_confirmation


class FakeClock:
    def __init__(self):self.value=10.
    def __call__(self):return self.value
    def sleep(self,value):self.value+=max(0,float(value))


class ServoJTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.config=load_config();cls.kin=PyKDLKinematics();cls.tool=ToolTransform.tool0_real()
    def test_minimum_jerk_segment_precomputed_before_send(self):
        q=self.config.real_home_joint_rad;start=self.tool.link_to_tool(self.kin.fk(q))
        segment=FixedSegmentPlanner(self.kin,self.tool,self.config).plan_action(
          start,[.001,0,0],[0,0,0],q,1.0)
        self.assertEqual(len(segment.times_s),101);self.assertEqual(segment.joints_rad.shape,(101,6))
        np.testing.assert_allclose(segment.tool_poses[0].position_m,start.position_m,atol=1e-12)
        np.testing.assert_allclose(segment.tool_poses[-1].position_m,start.position_m+[.001,0,0],atol=1e-12)
        np.testing.assert_allclose(segment.velocity_rad_s[[0,-1]],0,atol=2e-3)
    def test_nonfinite_action_rejected_without_segment(self):
        q=self.config.real_home_joint_rad;start=self.tool.link_to_tool(self.kin.fk(q))
        with self.assertRaisesRegex(ServoJPrecheckError,"NONFINITE_ACTION"):
            FixedSegmentPlanner(self.kin,self.tool,self.config).plan_action(start,[np.nan,0,0],[0,0,0],q,1.)
    def test_tracking_monitor_faults_on_error_stale_and_safe_state(self):
        monitor=TrackingMonitor(1.0);command=np.zeros(6)
        good=SimpleNamespace(joint_rad=np.deg2rad([.5]*6),robot_state=2)
        self.assertIsNone(monitor.check(command,good))
        bad=SimpleNamespace(joint_rad=np.deg2rad([0,0,2,0,0,0]),robot_state=5)
        fault=monitor.check(command,bad,["JOINT_STATE_STALE"])
        self.assertIn("TRACKING_ERROR:J3",fault.reasons);self.assertIn("JOINT_STATE_STALE",fault.reasons)
        self.assertIn("ROBOT_STATE_NOT_RUNNING:5",fault.reasons)
    def test_stream_100hz_jitter_and_mock_stop_on_fault(self):
        clock=FakeClock()
        class Publisher:
            def __init__(self):self.sent=[];self.blocked=False
            def publish(self,q,now):self.sent.append((now,q.copy()))
            def inhibit(self):self.blocked=True
        segment=SimpleNamespace(times_s=np.array([0.,.01,.02]),joints_rad=np.zeros((3,6)))
        pub=Publisher();stops=[];runner=ServoStreamRunner(pub,clock=clock,sleep=clock.sleep)
        result=runner.run(segment,lambda q,i:"TRACKING" if i==2 else None,stops.append)
        self.assertEqual(result.sent_count,2);self.assertEqual(result.fault,"TRACKING")
        self.assertEqual(stops,["TRACKING"]);self.assertTrue(pub.blocked)
        self.assertLess(abs(result.jitter_ms["max"]),1e-9)
    def test_stream_missed_deadline_skips_without_burst(self):
        clock=FakeClock()
        class Publisher:
            def __init__(self):self.sent=[]
            def publish(self,q,now):self.sent.append((int(q[0]),now));clock.value+=.025
            def inhibit(self):pass
        segment=SimpleNamespace(times_s=np.arange(6)*.01,
          joints_rad=np.repeat(np.arange(6)[:,None],6,axis=1))
        pub=Publisher();result=ServoStreamRunner(pub,clock=clock,sleep=clock.sleep).run(segment)
        self.assertGreater(result.missed_deadline_count,0)
        self.assertLess(result.sent_count,6)
        self.assertEqual([x[0] for x in pub.sent],sorted(set(x[0] for x in pub.sent)))
    def test_stream_loop_exception_inhibits_and_calls_stop(self):
        clock=FakeClock();stops=[]
        class Publisher:
            inhibited=False
            def publish(self,q,now):raise RuntimeError("publish boom")
            def inhibit(self):self.inhibited=True
        pub=Publisher();segment=SimpleNamespace(times_s=np.array([0.,.01]),joints_rad=np.zeros((2,6)))
        result=ServoStreamRunner(pub,clock=clock,sleep=clock.sleep).run(segment,stop_callback=stops.append)
        self.assertTrue(pub.inhibited);self.assertEqual(len(stops),1);self.assertIn("publish boom",result.fault)
    def test_mode_a_precomputes_before_stream_and_settle(self):
        events=[]
        class Planner:
            def plan_action(self,*args):events.append("plan");return SimpleNamespace(joints_rad=np.zeros((2,6)))
        class Runner:
            def run(self,segment):events.append("stream");return object()
        ServoJModeACore(Planner(),Runner(),lambda q:events.append("settle")).execute_action(
          None,[0]*3,[0]*3,np.zeros(6),.4)
        self.assertEqual(events,["plan","stream","settle"])
    def test_field_confirmation_and_abort_stop_once(self):
        with self.assertRaises(PermissionError):require_confirmation("policy","wrong")
        require_confirmation("policy","I_CONFIRM_V2_MODE:policy")
        calls=[];abort=OperatorAbort(lambda:calls.append("move_stop"));abort.request();abort.request()
        self.assertEqual(calls,["move_stop"]);self.assertIsNotNone(abort.response_s)

class HoldTests(unittest.TestCase):
    def config(self):
        return {"move_stop_mode_candidate_unverified":1,"servoj_hold_max_delta_deg_abort":.05,
          "servoj_hold_max_velocity_rad_s_abort":.005,"servoj_hold_rate_hz":100.,
          "servoj_hold_stream_duration_s":.02,"servoj_hold_post_stream_observe_s":.01,
          "servoj_hold_post_stop_observe_s":.01}
    def fakes(self,q=None,abort=lambda:False):
        clock=FakeClock();q=np.zeros(6) if q is None else np.asarray(q,float)
        class Pub:
            def __init__(self):self.sent=[];self.inhibited=False
            def publish(self,target,now):self.sent.append(now)
            def inhibit(self):self.inhibited=True
        class Feed:
            def snapshot(self,now):return SimpleNamespace(joint_rad=q,joint_velocity_rad_s=np.zeros(6),robot_state=1)
            def stale_reasons(self,now):return []
        class Stop:
            def __init__(self):self.calls=[]
            def move_stop(self,stop_mode):self.calls.append(stop_mode);return {"success":True}
        class Alarm:
            ever_active=False;events=[]
            def active(self):return False
            def record(self):return {"active":False}
        pub=Pub();stop=Stop();runner=HoldRunner(pub,Feed(),stop,Alarm(),self.config(),
          clock=clock,sleep=clock.sleep,abort_requested=abort)
        return runner,pub,stop
    def test_preflight_failure_sends_zero(self):
        runner,pub,stop=self.fakes();result=run_hold_if_preflight(["ROBOT_STATE_NOT_STANDBY"],runner,np.zeros(6))
        self.assertEqual(result["sent_count"],0);self.assertEqual(pub.sent,[]);self.assertEqual(stop.calls,[])
    def test_delta_over_point_zero_five_stops_immediately(self):
        runner,pub,stop=self.fakes(np.deg2rad([.051,0,0,0,0,0]));result=runner.run(np.zeros(6))
        self.assertEqual(result["sent_count"],1);self.assertEqual(stop.calls,[1]);self.assertIn("HOLD_DELTA_EXCEEDED",result["reason"])
    def test_enter_abort_stops_before_publish(self):
        runner,pub,stop=self.fakes(abort=lambda:True);result=runner.run(np.zeros(6))
        self.assertEqual(result["sent_count"],0);self.assertEqual(stop.calls,[1]);self.assertIn("OPERATOR_ABORT",result["reason"])
        self.assertIn("requested_monotonic",result["move_stop"]);self.assertEqual(result["move_stop"]["ack"],{"success":True})
        self.assertIn("confirmed",result["stop_confirmation"])
    def test_summary_written(self):
        with tempfile.TemporaryDirectory() as td:
            write_hold_outputs(td,[{"stage":"x"}],{"pass":False,"sent_count":0})
            self.assertEqual(json.loads((Path(td)/"summary.json").read_text())["sent_count"],0)
            self.assertEqual(len((Path(td)/"hold_samples.jsonl").read_text().splitlines()),1)
    def test_hold_processing_delay_skips_deadlines_without_negative_sleep(self):
        runner,pub,stop=self.fakes();original=runner.feedback.snapshot
        def delayed(now):
            value=original(now);runner.clock.value+=.025;return value
        runner.feedback.snapshot=delayed;result=runner.run(np.zeros(6))
        self.assertGreater(result["missed_deadline_count"],0)
        self.assertGreaterEqual(result["max_lateness_ms"],10.)

class TimeExploreTests(unittest.TestCase):
    def make(self):
        class Publisher:
            inhibited=False
            def inhibit(self):self.inhibited=True
        class Feedback:
            q=np.zeros(6)
            def stale_reasons(self,now):return []
            def snapshot(self,now):return SimpleNamespace(joint_rad=self.q.copy(),joint_velocity_rad_s=np.zeros(6),robot_state=1)
        class Stop:
            def __init__(self):self.calls=[]
            def move_stop(self,stop_mode):self.calls.append(stop_mode);return {"success":True}
        class Alarm:
            def active(self):return False
            def record(self):return {"active":False}
        pub=Publisher();stop=Stop();config={"move_stop_mode_candidate_unverified":1,
          "servoj_time_explore":{"j1_to_j5_abort_delta_deg":.05,"j6_tracking_error_abort_deg":1.,
          "j6_overshoot_abort_deg":.3,"j6_velocity_abort_deg_s":10.,"time_values_s":[.1,.05,.02,.01]}}
        clock=FakeClock()
        return TimeExploreRunner(pub,Feedback(),stop,Alarm(),config,clock=clock,sleep=clock.sleep),pub,stop
    def test_safety_reasons_cover_j1_to_j5_tracking_and_overshoot(self):
        runner,_,_=self.make();start=np.zeros(6);command=np.zeros(6);final=np.deg2rad(2.)
        snap=lambda q:SimpleNamespace(joint_rad=np.deg2rad(q),joint_velocity_rad_s=np.zeros(6),robot_state=1)
        self.assertIn("J1_TO_J5_DELTA_EXCEEDED",runner.safety_reasons(start,command,final,1,snap([.051,0,0,0,0,0])))
        self.assertIn("J6_TRACKING_ERROR_EXCEEDED",runner.safety_reasons(start,command,final,1,snap([0,0,0,0,0,1.01])))
        self.assertIn("J6_OVERSHOOT_EXCEEDED",runner.safety_reasons(start,np.deg2rad([0,0,0,0,0,2]),final,1,snap([0,0,0,0,0,2.31])))
    def test_failed_trial_inhibits_stops_and_skips_remaining(self):
        runner,pub,stop=self.make();calls=[]
        def fail(index,value,start):calls.append(index);return {"trial":index,"time_s":value,"pass":False,"reason":"MOCK_LIMIT"}
        runner._run_trial=fail;result=runner.run(np.zeros(6))
        self.assertEqual(calls,[0]);self.assertTrue(pub.inhibited);self.assertEqual(stop.calls,[1])
        self.assertEqual(sum(bool(x.get("skipped")) for x in result["trials"]),3)
    def test_each_motion_safety_fault_immediately_stops(self):
        cases=[("J1_TO_J5_DELTA_EXCEEDED",[.051,0,0,0,0,0],np.zeros(6),2.),
          ("J6_TRACKING_ERROR_EXCEEDED",[0,0,0,0,0,1.01],np.zeros(6),2.),
          ("J6_OVERSHOOT_EXCEEDED",[0,0,0,0,0,2.31],np.deg2rad([0,0,0,0,0,2]),2.)]
        for expected,actual_deg,command,final_deg in cases:
            with self.subTest(expected=expected):
                runner,pub,stop=self.make();snap=SimpleNamespace(joint_rad=np.deg2rad(actual_deg),
                  joint_velocity_rad_s=np.zeros(6),robot_state=1)
                def fail(index,value,start):
                    reasons=runner.safety_reasons(start,command,np.deg2rad(final_deg),1,snap)
                    raise TimeExploreFault(",".join(reasons))
                runner._run_trial=fail;result=runner.run(np.zeros(6))
                self.assertIn(expected,result["reason"]);self.assertTrue(pub.inhibited);self.assertEqual(stop.calls,[1])

class StopServoJTests(unittest.TestCase):
    def components(self,trigger=True):
        clock=FakeClock()
        class Feedback:
            def __init__(self):self.q=np.zeros(6);self.speed=np.deg2rad(2.0 if trigger else 0.)
            def snapshot(self,now):
                velocity=np.zeros(6);velocity[5]=self.speed
                return SimpleNamespace(joint_rad=self.q.copy(),joint_velocity_rad_s=velocity,robot_state=1)
            def stale_reasons(self,now):return []
        feedback=Feedback();events=[]
        class Publisher:
            def publish_with_time(self,q,t,now):feedback.q=q.copy();events.append("publish")
            def inhibit(self):events.append("inhibit")
        class Stop:
            def move_stop(self,stop_mode):events.append("move_stop");feedback.speed=0.;return {"success":True}
        class Alarm:
            def active(self):return False
        cfg={"move_stop_mode_candidate_unverified":1,"servoj_time_min_verified_s":.02,
          "servoj_time_max_verified_s":.1,"stop_test_servoj":{"time_s":.05}}
        return StopServoJRunner(Publisher(),feedback,Stop(),Alarm(),cfg,clock=clock,sleep=clock.sleep),events
    def test_trigger_inhibits_before_movestop(self):
        runner,events=self.components(True);result=runner.run(np.zeros(6))
        self.assertEqual(result["reason"],"VELOCITY_TRIGGER");self.assertTrue(result["pass"])
        self.assertLess(events.index("inhibit"),events.index("move_stop"))
    def test_no_trigger_times_out_and_stops(self):
        runner,events=self.components(False);result=runner.run(np.zeros(6))
        self.assertEqual(result["reason"],"INVALID_TRIGGER_TIMEOUT");self.assertFalse(result["pass"])
        self.assertIn("move_stop",events)
    def test_time_floor_and_tracking_formula(self):
        with self.assertRaisesRegex(ValueError,"BELOW_VERIFIED_MIN"):validate_servoj_time(.019)
        self.assertAlmostEqual(float(tracking_threshold_deg(.05,3.75)),.475)

class PolicyServoJTests(unittest.TestCase):
    def components(self,actual_deg=None,abort=None):
        clock=FakeClock();events=[];actual=np.deg2rad(np.zeros(6) if actual_deg is None else actual_deg)
        class Publisher:
            def publish(self,q,now):events.append("publish")
            def inhibit(self):events.append("inhibit")
        class Feedback:
            def snapshot(self,now):return SimpleNamespace(joint_rad=actual.copy(),joint_velocity_rad_s=np.zeros(6),robot_state=1)
            def stale_reasons(self,now):return []
        class Alarm:
            def active(self):return False
        rc={"rate_hz":100.,"servoj_time":.05,"policy_movej":{"settle_velocity_rad_s_candidate":.001,
          "settle_hold_s_candidate":.3,"target_error_deg_candidate":.1,"motion_timeout_s_candidate":1.}}
        runner=PolicyServoJRunner(Publisher(),Feedback(),Alarm(),load_config(),rc,clock=clock,sleep=clock.sleep)
        segment=SimpleNamespace(joints_rad=np.zeros((2,6)),velocity_rad_s=np.zeros((1,6)))
        return runner,segment,events,abort
    def test_tracking_excess_inhibits_then_stops(self):
        runner,segment,events,_=self.components([1,0,0,0,0,0]);stops=[]
        result=runner.run(segment,stop_callback=lambda reason:stops.append(reason) or {"state":"STOP_CONFIRMED"})
        self.assertEqual(result.sent_count,0);self.assertIn("TRACKING_ERROR:J1",result.fault)
        self.assertEqual(events,["inhibit"]);self.assertEqual(len(stops),1)
    def test_operator_abort_inhibits_and_stops(self):
        runner,segment,events,_=self.components();signal=SimpleNamespace(is_set=lambda:True);stops=[]
        result=runner.run(segment,stop_callback=lambda reason:stops.append(reason),operator_abort=signal)
        self.assertIn("OPERATOR_ABORT",result.fault);self.assertEqual(events,["inhibit"]);self.assertEqual(len(stops),1)
    def test_final_target_is_streamed_until_settled(self):
        runner,segment,events,_=self.components()
        result=runner.run(segment)
        self.assertIsNone(result.fault);self.assertIsNotNone(result.settled_monotonic)
        self.assertGreater(result.sent_count,len(segment.joints_rad))
        self.assertEqual(events.count("publish"),result.sent_count)
    def test_precheck_rejection_sends_zero(self):
        events=[]
        class Planner:
            def plan_action(self,*args):raise ServoJPrecheckError(["MOCK_REJECT"])
        class Stream:
            def run(self,*args):events.append("publish")
        with self.assertRaises(ServoJPrecheckError):
            ServoJModeACore(Planner(),Stream(),lambda q:None).execute_action(None,[0]*3,[0]*3,np.zeros(6),1.)
        self.assertEqual(events,[])
    def test_frame_wait_has_no_action_repeat(self):
        latch=ModeAObservationLatch();latch.request()
        with self.assertRaisesRegex(RuntimeError,"ALREADY_PENDING"):latch.request()
        self.assertEqual(latch.actions,0)
        self.assertEqual(latch.accept({"type":"RESULT","ok":True,"action_raw":[0]*7}),[0]*7)
        self.assertEqual(latch.actions,1)
    def test_summary_writer_and_segment_choices(self):
        from tools.policy_servoj import write_summary,validate_segment_T
        self.assertEqual(validate_segment_T(.6),.6);self.assertEqual(validate_segment_T(1),1.)
        with self.assertRaises(ValueError):validate_segment_T(.4)
        with tempfile.TemporaryDirectory() as td:
            write_summary(Path(td),{"control_mode":"servoj_A","executed_steps":0})
            self.assertEqual(json.loads((Path(td)/"summary.json").read_text())["control_mode"],"servoj_A")

if __name__=="__main__":unittest.main()
