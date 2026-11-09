import ast
import math
from pathlib import Path
import unittest
from analyze_servo_timing import rest_to_rest_time,pose_difference,align_feedback,published_commands


class TimingAnalysisTests(unittest.TestCase):
    def test_triangular_and_trapezoidal_profiles(self):
        self.assertAlmostEqual(rest_to_rest_time(0,20,20),0)
        self.assertAlmostEqual(rest_to_rest_time(5,20,20),1)
        self.assertAlmostEqual(rest_to_rest_time(20,20,20),2)
        self.assertAlmostEqual(rest_to_rest_time(40,20,20),3)
        with self.assertRaises(ValueError):rest_to_rest_time(-1,20,20)
        with self.assertRaises(ValueError):rest_to_rest_time(1,20,0)
        with self.assertRaises(ValueError):rest_to_rest_time(float('nan'),20,20)

    def test_orientation_compared_in_so3_not_euler_components(self):
        d,a=pose_difference([400,0,500,179,-155,3],[400,0,500,-1,155,-177])
        self.assertEqual(d,0);self.assertLess(a,1e-10)
        d,a=pose_difference([403,4,500,0,0,0],[400,0,500,0,0,90])
        self.assertEqual(d,5);self.assertAlmostEqual(a,90)

    def test_no_future_pair_or_unreturned_submission(self):
        row=dict(pos=[400,0,500,0,0,0],time=.1,vel=[20,20],acc=[20,20])
        events=[row,dict(event='PUBLISH_RETURNED_NOT_VENDOR_ACK',published_monotonic=1.,logger_wall_time=10.),row]
        commands=published_commands(events);self.assertEqual(len(commands),1)
        def pred(t):return dict(step=0,camera_timing=dict(inference_start_monotonic=t),
            action_pipeline=dict(action_age_sec=0),current_controller_pose=[397,0,500,0,0,0])
        self.assertEqual(align_feedback([pred(.9)],commands),[])
        pairs=align_feedback([pred(1.05)],commands)
        self.assertEqual(len(pairs),1)
        self.assertAlmostEqual(pairs[0]['translation_tracking_difference_mm'],3)
        self.assertTrue(pairs[0]['exceeds_even_full_target_time_speed_budget'])

    def test_offline_only(self):
        source=(Path(__file__).resolve().parents[1]/'analyze_servo_timing.py').read_text()
        tree=ast.parse(source)
        imports=[n.module for n in ast.walk(tree) if isinstance(n,ast.ImportFrom)]
        self.assertFalse(any(n and ('rclpy' in n or 'dsr_msgs' in n) for n in imports))
        self.assertNotIn('create_publisher',source);self.assertNotIn('call_async',source)


if __name__=='__main__':unittest.main()
