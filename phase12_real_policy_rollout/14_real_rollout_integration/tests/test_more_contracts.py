import sys,unittest
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from real_sink import Authorization,RealDoosanCommandSink,FakeRosTransport
from integration_metrics import collect

class MoreContracts(unittest.TestCase):
    def test_gripper_ack_is_not_measurement(self):
        t=FakeRosTransport()
        cfg=dict(polarity_confirmed=True,abort_value=0,gripper_closed_output_index=2,
            gripper_closed_hardware_value=1,inactive_hardware_value=0,closed_pulse_count=1,closed_pulse_time_s=0)
        s=RealDoosanCommandSink(lambda:t,Authorization(True,True,True,'MOTION_ENABLED'),gripper_config=cfg)
        self.assertIsNone(s.send_gripper(True)['measured_gripper_state'])
        self.assertEqual(s.send_gripper(True)['state'],'suppressed')
        self.assertEqual(len(t.calls),2)
    def test_gripper_unconfirmed_never_creates_transport(self):
        calls=[]
        s=RealDoosanCommandSink(lambda:calls.append(True),Authorization(True,True,True,'MOTION_ENABLED'))
        with self.assertRaises(PermissionError):s.send_gripper(True)
        self.assertEqual(calls,[])
    def test_metrics_do_not_invent_success(self):
        self.assertIsNone(collect([],[])['task_success'])
    def test_transport_allowlist(self):
        with self.assertRaises(ValueError):FakeRosTransport().send('/dsr01/motion/home','Home',{})

if __name__=='__main__':unittest.main()
