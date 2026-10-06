import json,sys,unittest
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT/'03_shadow_mode'))
from oft_timing_contract import action_age_sec,inference_time_sec,target_step,target_time_sec

class OftTimingContractTest(unittest.TestCase):
 def test_required_examples(self):
  self.assertEqual(target_time_sec(0,0),0.0);self.assertEqual(target_time_sec(0,4),.8)
  self.assertEqual(target_time_sec(5,0),1.0);self.assertEqual(target_time_sec(5,4),1.8)
 def test_target_step_invariant(self):
  for frame in range(0,45,5):
   for k in range(5):self.assertEqual(target_step(frame,k),frame+k)
 def test_action_age_is_chunk_offset(self):
  for frame in range(0,45,5):
   for k in range(5):self.assertAlmostEqual(action_age_sec(frame,k),k*.2)
 def test_negative_indices_rejected(self):
  with self.assertRaises(ValueError):target_step(-1,0)
  with self.assertRaises(ValueError):target_step(0,-1)

if __name__=='__main__':unittest.main()
