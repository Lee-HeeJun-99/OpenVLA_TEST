import sys,unittest
from pathlib import Path
R=Path(__file__).resolve().parents[1];sys.path.insert(0,str(R/'02_safety'))
from command_sinks import NullCommandSink,DoosanCommandSinkSkeleton
class Tests(unittest.TestCase):
 def test_null_records_without_delivery(self):
  r=NullCommandSink().submit({'x':1},{'accepted':True});self.assertTrue(r['would_send_command']);self.assertFalse(r['command_issued']);self.assertIsNone(r['executed_action'])
 def test_real_skeleton_cannot_construct(self):
  with self.assertRaises(RuntimeError):DoosanCommandSinkSkeleton()
  with self.assertRaises(RuntimeError):DoosanCommandSinkSkeleton(motion_enabled=True,explicit_motion_approval=True)
if __name__=='__main__':unittest.main()
