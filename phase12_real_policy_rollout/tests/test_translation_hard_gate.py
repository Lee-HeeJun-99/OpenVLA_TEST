import sys,unittest
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'02_safety'))
from action_step_limits import RAW_TRANSLATION_STEP_LIMIT_M,raw_translation_exceeds_limit
from canonical_action import CanonicalAction
from safety_pipeline import SafetyPipeline,RuntimeState


class TranslationGateTests(unittest.TestCase):
    def test_boundaries(self):
        self.assertEqual(RAW_TRANSLATION_STEP_LIMIT_M,.010)
        for value in (.0099,.009999999,.010):
            self.assertFalse(raw_translation_exceeds_limit([value,0,0]))
        self.assertTrue(raw_translation_exceeds_limit([.0101,0,0]))

    def test_production_pipeline(self):
        for value in (.0099,.009999999,.010,.0101):
            raw=[value,0,0,0,0,0,0]
            action=CanonicalAction.from_vector(raw,timestamp_monotonic=1,sequence_id='test',source_model='openvla')
            decision=SafetyPipeline('openvla',operator_confirmed_initial_open=True).inspect(
                action,RuntimeState(1,(.4,0,.5),(10,20,30),'alignment'))
            self.assertEqual('raw_translation_step_limit' in decision.reason,value>.010)
            self.assertEqual(list(action.as_vector()),raw)
            self.assertEqual(decision.candidate['raw_canonical_action'],raw)
            self.assertFalse(decision.command_issued)

    def test_nonfinite_rejected(self):
        for value in (float('nan'),float('inf')):
            with self.assertRaises(ValueError):raw_translation_exceeds_limit([value,0,0])
