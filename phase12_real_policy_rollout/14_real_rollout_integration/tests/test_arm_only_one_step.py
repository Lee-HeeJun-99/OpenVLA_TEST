import unittest
from run_real_rollout import Controller
from real_sink import RealDoosanCommandSink,Authorization,FakeRosTransport

class Log:
    def __init__(self):self.rows=[]
    def append(self,row):self.rows.append(row)

class ArmOnlyTests(unittest.TestCase):
    def setup_controller(self,arm_only):
        transport=FakeRosTransport();sink=RealDoosanCommandSink(lambda:transport,Authorization(True,True,True,'MOTION_ENABLED'))
        controller=Controller('openvla','short_horizon',sink,Log(),dry_run=False,arm_only_integration=arm_only)
        obs=dict(receive_monotonic=1,tcp_m_abc=[.4,0,.5,10,20,30],phase='alignment',robot_state_ok=True,camera_ok=True,joint_state_ok=True,tcp_ok=True,model_ok=True,communication_ok=True)
        return controller,transport,obs

    def test_close_is_logged_but_never_gripper_dispatched(self):
        c,t,obs=self.setup_controller(True)
        raw=[0,0,.0128,0,0,0,.998]
        record=c.step(raw,obs,inference_time=1,chunk_id='one',chunk_index=0)
        self.assertEqual(len(t.calls),1);self.assertEqual(t.calls[0][0],'/dsr01/motion/move_line')
        self.assertEqual(record['canonical_action'],raw)
        self.assertEqual(record['evaluation_scope'],'SYSTEM_INTEGRATION_TEST')
        with self.assertRaises(PermissionError):c.step(raw,obs,inference_time=1,chunk_id='two',chunk_index=0)
        self.assertEqual(len(t.calls),1)

    def test_default_full_policy_gripper_gate_unchanged(self):
        c,t,obs=self.setup_controller(False)
        c.step([0,0,.0128,0,0,0,.998],obs,inference_time=1,chunk_id='one',chunk_index=0)
        self.assertTrue(c.aborted)
        self.assertFalse(any(call[0]=='/dsr01/motion/move_line' for call in t.calls))

    def test_arm_only_still_rejects_large_translation(self):
        c,t,obs=self.setup_controller(True)
        c.step([.0131,0,0,0,0,0,.998],obs,inference_time=1,chunk_id='one',chunk_index=0)
        self.assertTrue(c.aborted)
        self.assertFalse(any(call[0]=='/dsr01/motion/move_line' for call in t.calls))

    def test_not_full_task(self):
        with self.assertRaises(ValueError):Controller('openvla','full_task',None,None,arm_only_integration=True)
