import unittest
from pathlib import Path
from unittest.mock import patch
import yaml
from real_sink import RealDoosanCommandSink, Authorization, FakeRosTransport


class TimedGripperTests(unittest.TestCase):
    def setup_sink(self):
        cfg=yaml.safe_load((Path(__file__).parents[1]/'configs/gripper_hardware.yaml').read_text())
        # Synthetic authorization does not establish real hardware safety.
        cfg.update(polarity_confirmed=True,abort_value=0)
        t=FakeRosTransport()
        s=RealDoosanCommandSink(lambda:t,Authorization(True,True,True,'MOTION_ENABLED'),gripper_config=cfg)
        return s,t

    def values(self,t):return [(v['index'],v['value']) for _,_,v in t.calls]

    def test_open_requested_values_and_delays_from_production_config(self):
        s,t=self.setup_sink();s.initialize_gripper_outputs();snapshots=[]
        def wait(seconds):snapshots.append((seconds,self.values(t)[:]));return False
        with patch.object(s.gripper_cancel,'wait',side_effect=wait):r=s.send_gripper(False)
        self.assertEqual(self.values(t),[(1,0),(2,0),(1,1),(1,0),(1,1),(1,0)])
        self.assertEqual(snapshots,[(1.3,[(1,0),(2,0),(1,1)]),(.2,[(1,0),(2,0),(1,1),(1,0)]),(.5,[(1,0),(2,0),(1,1),(1,0),(1,1)])])
        self.assertEqual(r['commanded_final_output_state'],[0,0])
        self.assertIsNone(r['measured_gripper_state'])

    def test_close_only_DO2_half_second_then_OFF(self):
        s,t=self.setup_sink();s.initialize_gripper_outputs()
        with patch.object(s.gripper_cancel,'wait',return_value=False) as wait:r=s.send_gripper(True)
        self.assertEqual(self.values(t),[(1,0),(2,0),(2,1),(2,0)])
        wait.assert_called_once_with(.5)
        self.assertEqual(r['state'],'completed')

    def test_duplicate_no_new_pulses(self):
        s,t=self.setup_sink();s.initialize_gripper_outputs()
        with patch.object(s.gripper_cancel,'wait',return_value=False):s.send_gripper(False)
        count=len(t.calls)
        self.assertEqual(s.send_gripper(False)['state'],'suppressed')
        self.assertEqual(len(t.calls),count)

    def test_initialization_and_abort_gates_unchanged(self):
        s,t=self.setup_sink()
        with self.assertRaises(PermissionError):s.send_gripper(False)
        self.assertEqual(t.calls,[])
        s.gripper_config['abort_value']=None
        with self.assertRaises(PermissionError):s.send_gripper(False)
        self.assertEqual(t.calls,[])

    def test_each_failed_OPEN_write_stops_without_commit(self):
        expected=[(1,1),(1,0),(1,1),(1,0)]
        for index in range(4):
            s,t=self.setup_sink();s.initialize_gripper_outputs()
            t.outcomes=[{'success':True}]*index+[{'success':False}]
            with patch.object(s.gripper_cancel,'wait',return_value=False):r=s.send_gripper(False)
            self.assertEqual(r['state'],'failed')
            self.assertEqual(self.values(t)[2:],expected[:index+1])
            self.assertIsNone(s.last_gripper)

    def test_cancel_during_either_hold_never_continues_or_invents_abort_IO(self):
        for results,expected in [([True],[(1,1)]),([False,True],[(1,1),(1,0)]),([False,False,True],[(1,1),(1,0),(1,1)])]:
            s,t=self.setup_sink();s.initialize_gripper_outputs()
            with patch.object(s.gripper_cancel,'wait',side_effect=results):r=s.send_gripper(False)
            self.assertEqual(r['state'],'aborted')
            self.assertEqual(self.values(t)[2:],expected)
            self.assertIsNone(s.last_gripper)

    def test_calibration_allowlists_match_current_protocol(self):
        from run_gripper_level_open_once import OpenCalibrationSink,CloseCalibrationSink
        self.assertEqual(OpenCalibrationSink.approved_sequence,((1,0),(2,0),(1,1),(1,0),(1,1),(1,0)))
        self.assertEqual(CloseCalibrationSink.approved_sequence,((2,1),(2,0)))


if __name__=='__main__':unittest.main()
