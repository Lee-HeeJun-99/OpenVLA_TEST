import csv,json,unittest
from pathlib import Path
ROOT=Path(__file__).resolve().parent;R=ROOT/'results'

class CorrectedReplayTest(unittest.TestCase):
 def summary(self):return json.loads((R/'summary.json').read_text())
 def test_before_after_counts(self):
  s=self.summary();self.assertEqual(s['before']['accepted'],10);self.assertEqual(s['after']['accepted'],13)
 def test_acceleration_artifact_removed(self):
  s=self.summary();self.assertEqual(s['before']['blockers']['translation_acceleration_limit'],19)
  self.assertNotIn('translation_acceleration_limit',s['after']['blockers'])
 def test_early_close_unchanged(self):
  s=self.summary();self.assertEqual(s['first_close_candidate_target_step'],2)
  self.assertEqual(s['early_close_lead_steps'],24);self.assertAlmostEqual(s['early_close_lead_s'],4.8)
 def test_correct_timing_and_phase_fields(self):
  with (R/'action_timing_corrected.csv').open() as f:rows=list(csv.DictReader(f))
  self.assertEqual(len(rows),45)
  for r in rows:
   self.assertEqual(int(r['target_step']),int(r['inference_frame'])+int(r['chunk_index']))
   self.assertAlmostEqual(float(r['target_time_s']),int(r['target_step'])*.2)
 def test_no_command(self):
  for line in (R/'episode4_oft_corrected_shadow.jsonl').read_text().splitlines():
   r=json.loads(line);self.assertFalse(r['command_issued']);self.assertIsNone(r['executed_action']);self.assertIsNone(r['robot_delivered_command'])

if __name__=='__main__':unittest.main()
