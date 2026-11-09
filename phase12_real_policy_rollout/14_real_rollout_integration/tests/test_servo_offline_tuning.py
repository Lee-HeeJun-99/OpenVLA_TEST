import ast
from pathlib import Path
import unittest
from tune_servo_offline import candidates,required_times,reference_horizon,FIELDS


class OfflineTuningTests(unittest.TestCase):
    def setUp(self):self.base=dict(zip(FIELDS,[20.,20.,20.,20.]))
    def test_one_factor_changes_are_truthful_and_rotation_speed_unchanged(self):
        for p in candidates(self.base):
            self.assertEqual(p['angular_velocity_deg_s'],20)
            changed=[key for key in FIELDS if p[key]!=self.base[key]]
            self.assertEqual(changed,p['changed_fields'])
    def test_velocity_increase_does_not_fix_acceleration_dominated_case(self):
        pairs=[dict(translation_tracking_difference_mm=4.,rotation_tracking_difference_deg=2.)]
        baseline=required_times(pairs,self.base)
        higher_v=dict(self.base,linear_velocity_mm_s=40.)
        self.assertEqual(required_times(pairs,higher_v),baseline)
        higher_a=dict(self.base,linear_acceleration_mm_s2=40.,angular_acceleration_deg_s2=40.)
        self.assertLess(required_times(pairs,higher_a)[0],baseline[0])
    def test_reference_rounding_and_invalid_inputs(self):
        self.assertAlmostEqual(reference_horizon([.899443946],.061545657),1.)
        self.assertAlmostEqual(reference_horizon([.636003],.061545657),.7)
        with self.assertRaises(ValueError):reference_horizon([],0)
    def test_no_ros_or_live_transport(self):
        source=(Path(__file__).resolve().parents[1]/'tune_servo_offline.py').read_text()
        tree=ast.parse(source)
        imports=[n.module for n in ast.walk(tree) if isinstance(n,ast.ImportFrom)]
        self.assertFalse(any(n and ('rclpy' in n or 'dsr_msgs' in n) for n in imports))
        self.assertNotIn('create_publisher',source);self.assertNotIn('call_async',source)
