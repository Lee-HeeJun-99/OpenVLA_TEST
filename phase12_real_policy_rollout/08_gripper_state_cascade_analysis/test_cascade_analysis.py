import json,sys,unittest
from pathlib import Path
ROOT=Path(__file__).resolve().parent;sys.path.insert(0,str(ROOT))
from compare_gripper_state_policy import run

class CascadeAnalysisTest(unittest.TestCase):
 def test_legacy_baseline_and_decoupled_counts(self):
  current,decoupled=run('current'),run('decoupled')
  self.assertEqual(sum(r['accepted'] for r in current),3)
  self.assertEqual(sum(r['accepted'] for r in decoupled),10)
 def test_decoupled_removes_unknown_cascade(self):
  rows=run('decoupled');blockers=[b for r in rows for b in json.loads(r['blockers'])]
  self.assertNotIn('gripper_unknown_state_blocks_command',blockers)
  self.assertNotIn('gripper_stale_or_timeout',blockers)
 def test_decoupled_preserves_premature_close_gate(self):
  rows=run('decoupled')
  self.assertEqual(sum('premature_gripper_close' in json.loads(r['blockers']) for r in rows),22)
 def test_no_command_or_measured_state_claim(self):
  for policy in ('current','decoupled'):
   rows=run(policy)
   self.assertTrue(all(not r['command_issued'] and not r['gripper_candidate_executed'] for r in rows))
   self.assertTrue(all(r['measured_gripper_state']=='MEASURED_UNKNOWN' for r in rows))

if __name__=='__main__':unittest.main()
