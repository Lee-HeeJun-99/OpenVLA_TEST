import unittest
from types import SimpleNamespace
import numpy as np
from controller.config import load_config
from controller.geometry import ToolTransform
from kinematics import PyKDLKinematics
from runtime.servoj import FixedSegmentPlanner,ServoJPrecheckError,ServoStreamRunner,TrackingMonitor,ServoJModeACore
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

if __name__=="__main__":unittest.main()
