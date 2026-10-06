import math,sys,unittest
from pathlib import Path
R=Path(__file__).resolve().parents[1];sys.path.insert(0,str(R/'02_safety'))
from canonical_action import CanonicalAction
class Tests(unittest.TestCase):
 def test_roundtrip(self):
  a=CanonicalAction.from_vector([.001,0,0,0,.01,0,.7],timestamp_monotonic=1,sequence_id='a',source_model='openvla')
  self.assertEqual(7,len(a.as_vector()));self.assertEqual('openvla',a.as_dict()['source_model'])
 def test_invalid(self):
  for v in ([0]*6,[math.nan]+[0]*6,[0]*6+[1.1]):
   with self.assertRaises(ValueError):CanonicalAction.from_vector(v,timestamp_monotonic=1,sequence_id='a',source_model='openvla')
if __name__=='__main__':unittest.main()
