import sys,unittest
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'02_safety'))
from action_step_limits import RAW_TRANSLATION_STEP_LIMIT_M,raw_translation_exceeds_limit
from canonical_action import CanonicalAction
from safety_pipeline import SafetyPipeline,RuntimeState


class TranslationGateTests(unittest.TestCase):
    def test_boundaries(self):
        self.assertEqual(RAW_TRANSLATION_STEP_LIMIT_M,.050)
        for value in (.0499,.049999,.050):
            self.assertFalse(raw_translation_exceeds_limit([value,0,0]))
        self.assertTrue(raw_translation_exceeds_limit([.0501,0,0]))

    def test_production_pipeline(self):
        for value in (.0499,.049999,.050,.0501):
            raw=[value,0,0,0,0,0,0]
            action=CanonicalAction.from_vector(raw,timestamp_monotonic=1,sequence_id='test',source_model='openvla')
            decision=SafetyPipeline('openvla',operator_confirmed_initial_open=True).inspect(
                action,RuntimeState(1,(.4,0,.5),(10,20,30),'alignment'))
            self.assertEqual('raw_translation_step_limit' in decision.reason,value>.050)
            self.assertEqual(list(action.as_vector()),raw)
            self.assertEqual(decision.candidate['raw_canonical_action'],raw)
            self.assertFalse(decision.command_issued)

    def test_nonfinite_rejected(self):
        for value in (float('nan'),float('inf')):
            with self.assertRaises(ValueError):raw_translation_exceeds_limit([value,0,0])

    def test_limit_is_vector_norm_not_independent_axis_cap(self):
        self.assertFalse(raw_translation_exceeds_limit([.030,.040,0]))
        self.assertTrue(raw_translation_exceeds_limit([.040,.040,0]))
        self.assertFalse(raw_translation_exceeds_limit([0,0,-.050]))
        self.assertTrue(raw_translation_exceeds_limit([0,0,-.0501]))

    def test_config_consistency_and_dynamics_unchanged(self):
        import yaml
        from action_rate_limiter import CanonicalActionRateLimiter
        root=Path(__file__).resolve().parents[1]
        safety=yaml.safe_load((root/'01_configs/safety_limits.yaml').read_text())
        disabled=yaml.safe_load((root/'01_configs/command_disabled_runtime.yaml').read_text())
        self.assertEqual(safety['translation_step_m'],RAW_TRANSLATION_STEP_LIMIT_M)
        self.assertEqual(disabled['raw_translation_step_limit_m'],RAW_TRANSLATION_STEP_LIMIT_M)
        self.assertEqual(safety['translation_velocity_mm_s'],20)
        self.assertEqual(safety['translation_acceleration_mm_s2'],20)
        self.assertEqual(safety['rotation_step_deg'],4)
        limiter=CanonicalActionRateLimiter()
        self.assertEqual(limiter.dt,.2)
        self.assertEqual(limiter.tv,.02)
        self.assertEqual(limiter.ta,.02)
        servo=yaml.safe_load((root/'14_real_rollout_integration/configs/servo_episode.yaml').read_text())
        self.assertEqual(servo['servo_transport']['acceleration_mm_s2_deg_s2'],[80.,80.])
        self.assertEqual(servo['servo_transport']['target_time_s'],.7)
