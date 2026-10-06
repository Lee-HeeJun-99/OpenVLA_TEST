import sys,unittest
from pathlib import Path
R=Path(__file__).resolve().parents[1];sys.path.insert(0,str(R/'02_safety'))
from action_contract import translation_m_to_mm
class Tests(unittest.TestCase):
 def test_only_unit_conversion(self):
  self.assertEqual((1000.,-500.,250.),translation_m_to_mm([1,-.5,.25]).delta_mm)
  with self.assertRaises(RuntimeError):translation_m_to_mm([1,0,0],empirical_gain=2.8)
if __name__=='__main__':unittest.main()
