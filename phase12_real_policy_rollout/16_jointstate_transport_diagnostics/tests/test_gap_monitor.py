import sys,unittest
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from gap_monitor import summarize

class GapTests(unittest.TestCase):
    def test_no_samples_not_pass(self):
        self.assertEqual(summarize([])['interpretation'],'NO_SAMPLES_NOT_STABILITY_PASS')
    def test_receive_only_gap(self):
        rows=[dict(receive_monotonic=0,source_gap_s=None,receive_gap_s=None,valid=True),
              dict(receive_monotonic=.102,source_gap_s=.01,receive_gap_s=.102,valid=True)]
        result=summarize(rows)
        self.assertEqual(result['receive_gap_ge_100ms'],1);self.assertEqual(result['source_gap_ge_100ms'],0)
    def test_both_source_receive_gap(self):
        row=dict(receive_monotonic=3.07,source_gap_s=3.06,receive_gap_s=3.07,valid=True)
        result=summarize([row]);self.assertEqual(result['source_gap_ge_100ms'],1)
    def test_nonincreasing_invalid(self):
        rows=[dict(receive_monotonic=0,source_gap_s=0,receive_gap_s=.01,valid=False),
              dict(receive_monotonic=.01,source_gap_s=-.01,receive_gap_s=.01,valid=True)]
        result=summarize(rows);self.assertEqual(result['regressions'],1)
        self.assertEqual(result['duplicate_source_timestamps'],1);self.assertEqual(result['invalid_samples'],1)
