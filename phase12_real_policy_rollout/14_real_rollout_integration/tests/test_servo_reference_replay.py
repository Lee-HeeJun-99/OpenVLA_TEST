"""Pure offline guards and reference proposals; fake callbacks only."""
import math
import unittest
import numpy as np
import yaml
from pathlib import Path
from servo_experimental.controller import ServoShadowController
from servo_experimental.reference_replay import (
    delta_target,install_reference,pose_distance,DiagnosticGuardLimits,OfflineStreamBoundary)


class ReferenceReplayTests(unittest.TestCase):
    def make(self):
        sends=[];stops=[]
        guard=OfflineStreamBoundary([400,0,500,0,0,0],
            DiagnosticGuardLimits(1e-6,1e-6,5.,2.,1.2),
            lambda p:sends.append(p),lambda:stops.append('FAKE_STOP'))
        return guard,sends,stops

    def test_delta_target_has_exact_geodesic_budget(self):
        anchor=[400,0,500,179,-180,-179]
        action=[.003,.004,0,0,0,math.radians(1),1]
        target=delta_target(anchor,action)
        t,r=pose_distance(anchor,target)
        self.assertAlmostEqual(t,5.)
        self.assertAlmostEqual(r,1.)

    def test_proposals_use_their_named_anchor(self):
        root=Path(__file__).resolve().parents[2]
        limits=yaml.safe_load((root/'01_configs/safety_limits.yaml').read_text())
        tracker=ServoShadowController([400,0,500,0,0,0],limits)
        tracker.v=np.array([1.,0,0]);tracker.w=np.array([0.,0,.01])
        actual=[403,0,500,1,0,0];action=[.001,0,0,0,0,.01,0]
        anchor,target=install_reference(tracker,action,actual,'command')
        self.assertEqual(anchor[:3],[400,0,500]);self.assertEqual(target[0],401)
        self.assertEqual(tracker.v[0],1.)
        anchor,target=install_reference(tracker,action,actual,'actual')
        self.assertEqual(anchor,actual);self.assertEqual(target[0],404)
        np.testing.assert_equal(tracker.v,[0,0,0])

    def test_translation_budget_before_publish_and_inhibit(self):
        g,s,st=self.make();g.begin_segment(1,[.001,0,0,0,0,0,0])
        g.inspect_and_publish([400.5,0,500,0,0,0],.1,[400,0,500,0,0,0],.05)
        with self.assertRaisesRegex(RuntimeError,'translation_action_budget'):
            g.inspect_and_publish([401.1,0,500,0,0,0],.2,[400.5,0,500,0,0,0],.15)
        self.assertEqual(len(s),1);self.assertEqual(len(st),1)
        with self.assertRaises(RuntimeError):g.begin_segment(2,[.001,0,0,0,0,0,0])
        with self.assertRaises(RuntimeError):g.inspect_and_publish([400,0,500,0,0,0],.3,[400,0,500,0,0,0],.3)
        self.assertEqual(len(st),1);self.assertEqual(len(s),1)

    def test_rotation_arc_detects_cancelling_oscillation(self):
        g,s,st=self.make();g.begin_segment(1,[0,0,0,0,0,math.radians(1),0])
        g.inspect_and_publish([400,0,500,.6,0,0],.1,[400,0,500,0,0,0],.05)
        with self.assertRaisesRegex(RuntimeError,'rotation_action_budget'):
            g.inspect_and_publish([400,0,500,-.2,0,0],.2,[400,0,500,.6,0,0],.15)
        self.assertLess(g.events[-1]['net_rotation_deg'],1.)
        self.assertGreater(g.events[-1]['arc_rotation_deg'],1.)
        self.assertEqual(len(s),1);self.assertEqual(len(st),1)

    def test_following_error_inhibits_and_requests_one_fake_stop(self):
        g,s,st=self.make();g.begin_segment(1,[.01,0,0,0,0,.1,0])
        g.inspect_and_publish([401,0,500,0,0,0],.1,[400,0,500,0,0,0],.05)
        with self.assertRaisesRegex(RuntimeError,'following_error_limit'):
            g.inspect_and_publish([402,0,500,0,0,0],.2,[401,0,500,3,0,0],.15)
        self.assertEqual(len(s),1);self.assertEqual(len(st),1)

    def test_feedback_compared_to_past_command_not_future(self):
        g,s,st=self.make();g.begin_segment(1,[.01,0,0,0,0,0,0])
        g.inspect_and_publish([410,0,500,0,0,0],.1,[400,0,500,0,0,0],.05)
        g.inspect_and_publish([410,0,500,0,0,0],.2,[400,0,500,0,0,0],.05)
        self.assertEqual(len(s),2);self.assertEqual(g.events[-1]['following_translation_mm'],0)

    def test_equivalent_angles_do_not_false_trigger(self):
        self.assertLess(pose_distance([400,0,500,180,0,0],[400,0,500,-180,0,0])[1],1e-8)

    def test_unknown_feedback_fails_closed(self):
        g,s,st=self.make();g.begin_segment(1,[.01,0,0,0,0,0,0])
        with self.assertRaisesRegex(RuntimeError,'stale_or_future_feedback'):
            g.inspect_and_publish([400,0,500,0,0,0],2.,[400,0,500,0,0,0],0.)
        self.assertEqual(s,[])

    def test_invalid_candidate_latches(self):
        g,s,st=self.make();g.begin_segment(1,[.01,0,0,0,0,0,0])
        with self.assertRaisesRegex(RuntimeError,'invalid_pose'):
            g.inspect_and_publish([float('nan'),0,500,0,0,0],.1,[400,0,500,0,0,0],.05)
        self.assertTrue(g.closed);self.assertEqual(s,[])

    def test_190404_original_replay_matches_published_stream(self):
        from replay_servo_references import read_inputs,replay
        result,_,_=replay('original',False,read_inputs())
        self.assertEqual(result['rows'],2143)
        self.assertLess(result['max_recorded_pose_difference_mm'],1e-7)
        self.assertLess(result['max_recorded_pose_difference_deg'],1e-7)


if __name__=='__main__':unittest.main()
