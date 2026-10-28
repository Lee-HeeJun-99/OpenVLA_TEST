import unittest
from real_sink import RealDoosanCommandSink,Authorization,FakeRosTransport

class AbortOffTests(unittest.TestCase):
    def sink(self,outcomes=None):
        t=FakeRosTransport(outcomes)
        cfg=dict(abort_behavior='BOTH_OUTPUTS_OFF',abort_policy_operator_approved=True)
        s=RealDoosanCommandSink(lambda:t,Authorization(True,True,True,'MOTION_ENABLED'),gripper_config=cfg)
        return s,t
    def test_success_first_attempt_and_idempotent(self):
        s,t=self.sink();r=s.abort_gripper_outputs_off()
        self.assertEqual(r['state'],'completed');self.assertEqual(len(r['attempts']),1)
        self.assertEqual(len(t.calls),4)
        self.assertIs(s.abort_gripper_outputs_off(),r);self.assertEqual(len(t.calls),4)
        self.assertTrue(s.aborted)
        with self.assertRaises(RuntimeError):s.send_pose([400,0,500,0,0,0])
    def test_readback_nonzero_retries_pair_only(self):
        s,t=self.sink([{'success':True},{'success':True},{'success':True,'value':1},{'success':True,'value':0}])
        r=s.abort_gripper_outputs_off();self.assertEqual(len(r['attempts']),2)
        self.assertEqual(r['state'],'completed')
        self.assertTrue(all(v['value']==0 for _,typ,v in t.calls if typ=='SetToolDigitalOutput'))
    def test_ack_failure_retries_even_if_readback_zero(self):
        s,t=self.sink([{'success':False},{'success':True},{'success':True,'value':0},{'success':True,'value':0}])
        self.assertEqual(len(s.abort_gripper_outputs_off()['attempts']),2)
    def test_three_total_attempts_then_error_no_rollout_retry(self):
        outcomes=[{'success':True},{'success':True},{'success':True,'value':1},{'success':True,'value':1}]*3
        s,t=self.sink(outcomes);r=s.abort_gripper_outputs_off()
        self.assertEqual(r['state'],'failed');self.assertEqual(len(r['attempts']),3)
        self.assertEqual(len(t.calls),12)
        self.assertIn('3_ATTEMPTS',r['reason']);self.assertFalse(r['rollout_retry'])
        s.abort_gripper_outputs_off();self.assertEqual(len(t.calls),12)
    def test_missing_readback_value_not_success(self):
        s,t=self.sink([{'success':True}]*12)
        self.assertEqual(s.abort_gripper_outputs_off()['state'],'failed')
    def test_read_timeout_bounded(self):
        one=[{'success':True},{'success':True},TimeoutError('read'),TimeoutError('read')]
        s,t=self.sink(one*3);r=s.abort_gripper_outputs_off()
        self.assertEqual(r['state'],'failed');self.assertEqual(len(t.calls),12)
    def test_unapproved_policy_no_transport(self):
        s,t=self.sink();s.gripper_config['abort_policy_operator_approved']=False
        with self.assertRaises(PermissionError):s.abort_gripper_outputs_off()
        self.assertEqual(t.calls,[])
    def test_broken_logger_does_not_skip_controller_cleanup(self):
        from run_real_rollout import Controller
        s,t=self.sink();s.gripper_config.update(polarity_confirmed=True,command_mode='operator_timed_pulse')
        class Log:
            def append(self,row):raise OSError('logger')
        c=Controller('openvla','full_task',s,Log(),dry_run=False,execution_mode='FULL_POLICY_ROLLOUT',
            gripper_contract=dict(abort_behavior='BOTH_OUTPUTS_OFF',operator_approved=True))
        r=c.abort('logger_failure');c.abort('again')
        self.assertEqual(r['gripper_abort']['state'],'completed')
        self.assertEqual(len(t.calls),5) # one arm HOLD, two OFF writes, two reads

if __name__=='__main__':unittest.main()
