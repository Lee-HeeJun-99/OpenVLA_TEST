import csv,json,subprocess,sys,unittest
from pathlib import Path
ROOT=Path(__file__).resolve().parent;RESULTS=ROOT/'results'

class OftTimingAnalysisTest(unittest.TestCase):
 def rows(self,name):
  with (RESULTS/name).open() as f:return list(csv.DictReader(f))
 def test_complete_target_steps_and_k_indices(self):
  r=self.rows('action_timing.csv')
  self.assertEqual([int(x['frame']) for x in r],list(range(45)))
  self.assertEqual({int(x['chunk_index']) for x in r},set(range(5)))
 def test_correct_5hz_execution_mapping(self):
  for r in self.rows('action_timing.csv'):
   self.assertAlmostEqual(float(r['expected_execution_time_s']),int(r['frame'])*.2)
   self.assertEqual(int(r['frame']),int(r['inference_frame'])+int(r['chunk_index']))
 def test_first_close_and_reference_timing(self):
  s=json.loads((RESULTS/'summary.json').read_text())
  self.assertEqual(s['first_close_candidate']['expanded_target_frame'],2)
  self.assertEqual(s['reference_episode4_close_command_step'],26)
  self.assertEqual(s['first_mapped_grasp_close_frame'],27)
 def test_phase_mapping_does_not_explain_early_close(self):
  s=json.loads((RESULTS/'summary.json').read_text())
  self.assertEqual(s['phase_mapping']['premature_count_coarse_inference_phase'],22)
  self.assertEqual(s['phase_mapping']['premature_count_target_step_phase'],22)
 def test_all_command_fields_false(self):
  self.assertTrue(all(r['command_issued']=='False' for r in self.rows('action_timing.csv')))

if __name__=='__main__':unittest.main()
