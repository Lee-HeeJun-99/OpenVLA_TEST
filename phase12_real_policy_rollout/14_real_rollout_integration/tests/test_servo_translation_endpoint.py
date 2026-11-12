"""Offline tests: translation endpoint is unscaled; tracker remains bounded."""
import math
import unittest
from pathlib import Path
import yaml
from run_real_rollout import SafetyPipeline, RuntimeState, CanonicalAction
from servo_experimental.controller import ServoShadowController


class TranslationEndpointTests(unittest.TestCase):
    def pipeline(self, enabled=True):
        return SafetyPipeline('openvla', operator_confirmed_initial_open=False,
                              arm_only=True, translation_endpoint_delta=enabled)

    def inspect(self, pipeline, raw, t=1., **state):
        return pipeline.inspect(CanonicalAction.from_vector(raw,
            timestamp_monotonic=t, sequence_id=str(t), source_model='openvla'),
            RuntimeState(t, (.305, -.011, .723), (179., -155., 3.), 'alignment', **state))

    def test_translation_only_passthrough_rotation_unchanged(self):
        raw=[.012875,0,0,.03,0,0,1]
        changed=self.inspect(self.pipeline(),raw).candidate
        original=self.inspect(self.pipeline(False),raw).candidate
        self.assertEqual(changed['limited_canonical_action'][:3],raw[:3])
        self.assertEqual(changed['limited_canonical_action'][3:],original['limited_canonical_action'][3:])
        self.assertFalse(changed['translation_delta_limiter_applied'])
        self.assertTrue(changed['rotation_delta_limiter_applied'])
        self.assertAlmostEqual(changed['target_pose_candidate_mm_zyz_deg'][0],317.875)

    def test_repeated_endpoint_does_not_use_displacement_as_velocity(self):
        pipeline=self.pipeline()
        for t in (1.,1.2,1.4):
            self.assertTrue(self.inspect(pipeline,[.012875,0,0,0,0,0,1],t).accepted)

    def test_existing_rejections_remain(self):
        for raw,state in (([.051,0,0,0,0,0,1],{}),
                          ([0,0,0,math.radians(4.1),0,0,1],{}),
                          ([0,0,.04,0,0,0,1],{'joint_limits_ok':False}),
                          ([.01,0,0,0,0,0,1],{'camera_ok':False}),
                          ([.01,0,0,0,0,0,1],{'logger_ok':False})):
            with self.subTest(raw=raw,state=state):
                self.assertFalse(self.inspect(self.pipeline(),raw,**state).accepted)
        with self.assertRaises(ValueError):
            self.inspect(self.pipeline(),[float('nan'),0,0,0,0,0,1])

    def test_tracker_still_limits_first_tick(self):
        root=Path(__file__).resolve().parents[1]
        limits=yaml.safe_load((root.parent/'01_configs/safety_limits.yaml').read_text())
        candidate=self.inspect(self.pipeline(),[.012875,0,0,0,0,0,1]).candidate
        tracker=ServoShadowController([305,-11,723,179,-155,3],limits)
        tracker.set_target(candidate['target_pose_candidate_mm_zyz_deg'],1.,1.)
        row=tracker.tick(1.)
        self.assertAlmostEqual(row['pos'][0]-305,.025)
        self.assertAlmostEqual(row['translation_velocity_mm_s'][0],1.)


if __name__=='__main__':unittest.main()
