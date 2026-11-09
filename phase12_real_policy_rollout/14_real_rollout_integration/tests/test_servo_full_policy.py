"""Fake transport only; these tests are not hardware verification."""
import threading
import time
import unittest
from unittest.mock import patch
from full_policy_live_mode import FullPolicyLiveMode
from real_sink import FakeRosTransport
from servo_experimental.full_policy import ServoFullPolicyGripper
from servo_experimental.live_guard import StreamGuard


class Log:
    def append(self, row): pass


class ServoGripperTests(unittest.TestCase):
    def setup_session(self):
        transport=FakeRosTransport();guard=StreamGuard(lambda:None)
        mode=FullPolicyLiveMode('unused')
        approval=dict(execution_mode='FULL_POLICY_ROLLOUT',confirmed_wall_time=time.time(),
            **dict.fromkeys(('operator_present','workspace_clear','estop_accessible','robot_stationary',
            'tcp_engineering_acceptance','gripper_mapping_confirmed','explicit_one_episode'),True))
        cfg=mode.approved_gripper_config(approval)
        for k in ('open_first_high_hold_s','open_second_high_hold_s','open_between_pulses_low_hold_s','close_high_hold_s'):cfg[k]=0
        holder=[]
        def fail(reason):guard.finish(reason);holder[0].abort_async(reason)
        s=ServoFullPolicyGripper(lambda:transport,cfg,Log(),guard,guard.check,fail)
        holder.append(s)
        self.assertEqual(transport.calls,[])
        return s,guard,transport

    def initialize(self,s):
        s.full_policy_gripper.initialize_open()
        s.full_policy_gripper.confirm_observed_open(observed_open=True,evidence_source='FAKE_TEST_ONLY')
        s.start()

    def wait_for(self,predicate):
        deadline=time.monotonic()+2
        while not predicate() and time.monotonic()<deadline:time.sleep(.002)
        self.assertTrue(predicate())

    def test_actual_OPEN_required_and_gripper_only_endpoint(self):
        s,g,t=self.setup_session()
        with self.assertRaises(PermissionError):s.start()
        for kind in ('pose','hold','stop'):
            with self.assertRaises(PermissionError):s.sink._send(kind,{})
        self.assertEqual(t.calls,[])

    def test_state_transitions_and_no_duplicate(self):
        s,g,t=self.setup_session();self.initialize(s)
        try:
            for i,value in enumerate((.9,.9,.5,.1)):
                s.offer(value,time.monotonic(),i)
                self.wait_for(lambda:s.pending is None and not s.busy)
            writes=[v for _,typ,v in t.calls if typ=='SetToolDigitalOutput']
            self.assertEqual([(v['index'],v['value']) for v in writes],
                [(1,0),(2,0),(1,1),(1,0),(1,1),(1,0),(2,1),(2,0),(1,1),(1,0),(1,1),(1,0)])
            self.assertEqual(g.attempts,0) # IO never masquerades as arm stream.
            self.assertFalse(s.prepare_finish())
        finally:s.prepare_finish();s.join()

    def test_fault_open_or_closed_idempotent_OFF_and_no_future_ON(self):
        for closed in (False,True):
            s,g,t=self.setup_session();self.initialize(s)
            if closed:
                s.offer(.9,time.monotonic(),0)
                self.wait_for(lambda:s.full_policy_gripper.state=='CLOSED' and not s.busy)
            before=len(t.calls);g.finish('model_failure')
            s.abort_async('model_failure');s.abort_async('again');s.join()
            self.assertEqual(s.cleanup_result['state'],'completed')
            writes=[v for _,typ,v in t.calls[before:] if typ=='SetToolDigitalOutput']
            self.assertEqual(writes,[dict(index=1,value=0),dict(index=2,value=0)])
            with self.assertRaises(RuntimeError):s.offer(.9,time.monotonic(),1)

    def test_pulse_worker_does_not_block_arm_and_inflight_abort(self):
        s,g,t=self.setup_session();self.initialize(s)
        entered=threading.Event();release=threading.Event()
        original=s.full_policy_gripper.transition
        def blocked(value):entered.set();release.wait(2);return original(value)
        with patch.object(s.full_policy_gripper,'transition',side_effect=blocked):
            s.offer(.9,time.monotonic(),0);self.assertTrue(entered.wait(1))
            for _ in range(20):g.publish(lambda:None)
            self.assertEqual(g.submitted,20)
            self.assertTrue(s.prepare_finish())
            g.finish('operator_abort');s.abort_async('operator_abort');release.set();s.join()
        self.assertEqual(s.cleanup_result['state'],'completed')

    def test_stale_intent_faults_instead_of_replaying_old_grasp(self):
        s,g,t=self.setup_session();self.initialize(s)
        s.offer(.9,time.monotonic()-2,0)
        self.wait_for(lambda:g.reason is not None);s.join()
        self.assertIn('stale_gripper_intent',g.reason)
        self.assertFalse(any(typ=='SetToolDigitalOutput' and v==dict(index=2,value=1) for _,typ,v in t.calls))
