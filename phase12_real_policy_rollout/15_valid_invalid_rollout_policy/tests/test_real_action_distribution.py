import sys,unittest
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT))
from collect_real_action_distribution import stats,candidates

class DistributionTests(unittest.TestCase):
    def test_statistics_include_over_limit(self):
        result=stats([5,10,12,20]);self.assertEqual(result['count'],4)
        self.assertEqual(result['max'],20);self.assertEqual(result['mean'],11.75)

    def test_minimum_count(self):
        self.assertEqual(candidates([12]*29)['status'],'INSUFFICIENT_VALID_SAMPLES')

    def test_candidates_descriptive_not_applied(self):
        result=candidates([5]*40+[12]*10)
        self.assertEqual(result['candidates'][0]['rejection_count'],10)
        self.assertEqual(result['candidates'][0]['rejection_rate'],.2)
        self.assertIsNone(result['margin'])
        self.assertEqual(result['status'],'DESCRIPTIVE_CANDIDATES_NOT_APPLIED')

    def test_no_physical_clients(self):
        source=(ROOT/'collect_real_action_distribution.py').read_text()
        for symbol in ('MoveLine','MoveJoint','MoveStop','ServoJ','ServoL','SetToolDigitalOutput','RealDoosanCommandSink'):
            self.assertNotIn(symbol,source)
