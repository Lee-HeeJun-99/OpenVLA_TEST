import unittest
from real_sink import RealDoosanCommandSink, Authorization, FakeRosTransport
from full_policy_gripper import FullPolicyGripper


class Log:
    def append(self,row):pass


class InterlockedTests(unittest.TestCase):
    def setup_sink(self):
        cfg=dict(command_mode='interlocked_level',polarity_confirmed=True,
            abort_value=0,abort_value_confirmed=True,
            gripper_open_output_index=1,gripper_closed_output_index=2,
            gripper_open_hardware_value=1,gripper_closed_hardware_value=1,
            inactive_hardware_value=0,open_first_high_hold_s=0)
        t=FakeRosTransport()
        sink=RealDoosanCommandSink(lambda:t,Authorization(True,True,True,'MOTION_ENABLED'),gripper_config=cfg)
        return sink,t

    def values(self,t):return [(v['index'],v['value']) for _,_,v in t.calls]

    def test_startup_OFF_OFF_does_not_claim_open(self):
        s,t=self.setup_sink();r=s.initialize_gripper_outputs()
        self.assertEqual(self.values(t),[(1,0),(2,0)])
        self.assertIsNone(s.last_gripper)
        self.assertIsNone(r['measured_gripper_state'])
        with self.assertRaises(PermissionError):s.initialize_gripper_outputs()

    def test_no_command_before_initialization(self):
        s,t=self.setup_sink()
        with self.assertRaises(PermissionError):s.send_gripper(False)
        self.assertEqual(t.calls,[])

    def test_open_close_open_retains_levels_without_pulse(self):
        s,t=self.setup_sink();s.initialize_gripper_outputs()
        a=s.send_gripper(False);b=s.send_gripper(True);c=s.send_gripper(False)
        self.assertEqual(self.values(t),[(1,0),(2,0),(2,0),(1,1),(1,0),(1,1),(1,0),(2,1),(2,0),(1,1),(1,0),(1,1)])
        self.assertEqual([r['electrical_output_state'] for r in (a,b,c)],[[1,0],[0,1],[1,0]])
        # Mock config skips the hold; no trailing OFF, final OPEN is DO1=1.
        outputs={1:0,2:0}
        for channel,value in self.values(t):
            outputs[channel]=value
            self.assertNotEqual([outputs[1],outputs[2]],[1,1])

    def test_same_state_suppressed(self):
        s,t=self.setup_sink();s.initialize_gripper_outputs();s.send_gripper(False)
        count=len(t.calls)
        self.assertEqual(s.send_gripper(False)['state'],'suppressed')
        self.assertEqual(len(t.calls),count)

    def test_opposite_off_failure_never_asserts_target(self):
        s,t=self.setup_sink();s.initialize_gripper_outputs()
        t.outcomes=[{'success':False}]
        r=s.send_gripper(True)
        self.assertEqual(r['state'],'failed')
        self.assertEqual(self.values(t)[-1],(1,0))
        self.assertNotIn((2,1),self.values(t))
        self.assertIsNone(s.last_gripper)

    def test_target_on_failure_does_not_commit(self):
        s,t=self.setup_sink();s.initialize_gripper_outputs()
        t.outcomes=[{'success':True},{'success':False}]
        self.assertEqual(s.send_gripper(True)['state'],'failed')
        self.assertIsNone(s.last_gripper)

    def test_abort_between_OFF_and_ON_stops_transition(self):
        s,t=self.setup_sink();s.initialize_gripper_outputs()
        original=t.wait
        def wait(future,timeout):
            result=original(future,timeout);s.inhibit();return result
        t.wait=wait
        r=s.send_gripper(True)
        self.assertEqual(r['state'],'ABORTED_IN_FLIGHT')
        self.assertNotIn((2,1),self.values(t))
        with self.assertRaises(PermissionError):s.send_gripper(False)

    def test_failed_startup_does_not_enable_transitions(self):
        s,t=self.setup_sink();t.outcomes=[{'success':False}]
        self.assertEqual(s.initialize_gripper_outputs()['state'],'failed')
        self.assertFalse(s.gripper_outputs_initialized)
        self.assertEqual(self.values(t),[(1,0)])

    def test_abort_unknown_still_blocks_normal_transition(self):
        s,t=self.setup_sink();s.gripper_config['abort_value']=None
        with self.assertRaises(PermissionError):s.send_gripper(True)
        self.assertEqual(t.calls,[])

    def test_full_policy_initializes_OFF_OFF_before_OPEN(self):
        s,t=self.setup_sink()
        contract=dict(abort_behavior='OUTPUT_OFF',abort_behavior_confirmed=True,
            abort_evidence_source='FAKE_TEST_ONLY_NOT_HARDWARE')
        g=FullPolicyGripper(s,Log(),contract)
        g.initialize_open()
        self.assertEqual(self.values(t),[(1,0),(2,0),(2,0),(1,1),(1,0),(1,1)])
        self.assertFalse(g.initialized)
        g.confirm_observed_open(observed_open=True,evidence_source='FAKE_OPERATOR_TEST_ONLY')
        g.transition(.9)
        self.assertEqual(self.values(t)[-2:],[(1,0),(2,1)])

    def test_open_stops_at_any_failed_ACK_without_committing(self):
        expected=[(2,0),(1,1),(1,0),(1,1)]
        for fail_index in range(4):
            with self.subTest(fail_index=fail_index):
                s,t=self.setup_sink();s.initialize_gripper_outputs()
                t.outcomes=[{'success':True}]*fail_index+[{'success':False}]
                r=s.send_gripper(False)
                self.assertEqual(r['state'],'failed')
                self.assertEqual(self.values(t)[2:],expected[:fail_index+1])
                self.assertIsNone(s.last_gripper)

    def test_one_second_hold_after_first_ON_ACK_only(self):
        from unittest.mock import patch
        s,t=self.setup_sink();s.initialize_gripper_outputs()
        s.gripper_config['open_first_high_hold_s']=1.0
        snapshots=[]
        def wait(seconds):
            snapshots.append((seconds,self.values(t)[:]))
            return False
        with patch.object(s.gripper_cancel,'wait',side_effect=wait):
            s.send_gripper(False)
            s.send_gripper(True)
        self.assertEqual(snapshots,[(1.0,[(1,0),(2,0),(2,0),(1,1)])])
        self.assertEqual(self.values(t)[4:],[(1,0),(1,1),(1,0),(2,1)])

    def test_cancel_during_hold_stops_before_OFF_and_second_ON(self):
        from unittest.mock import patch
        s,t=self.setup_sink();s.initialize_gripper_outputs()
        s.gripper_config['open_first_high_hold_s']=1.0
        with patch.object(s.gripper_cancel,'wait',return_value=True):r=s.send_gripper(False)
        self.assertEqual(r['state'],'aborted')
        self.assertEqual(self.values(t),[(1,0),(2,0),(2,0),(1,1)])
        self.assertIsNone(s.last_gripper)

    def test_legacy_full_task_cannot_move_before_level_initialization(self):
        from run_real_rollout import Controller
        s,t=self.setup_sink()
        c=Controller('openvla','full_task',s,Log(),dry_run=False,initial_open=True)
        obs=dict(receive_monotonic=1,tcp_m_abc=[.4,0,.5,10,20,30],phase='alignment',
                 robot_state_ok=True,camera_ok=True,joint_state_ok=True,tcp_ok=True,
                 model_ok=True,communication_ok=True)
        c.step([0,0,0,0,0,0,0],obs,inference_time=1,chunk_id='test',chunk_index=0)
        self.assertTrue(c.aborted)
        self.assertFalse(any(n.endswith('move_line') for n,_,_ in t.calls))


if __name__=='__main__':unittest.main()
