import sys,unittest
from pathlib import Path
R=Path(__file__).resolve().parents[1];sys.path.insert(0,str(R/'02_safety'))
from command_sinks import MockDoosanCommandSink
class Tests(unittest.TestCase):
 def test_accept_reject_are_memory_only(self):
  s=MockDoosanCommandSink();a=s.submit({}, {'accepted':True});r=s.submit({}, {'accepted':False})
  self.assertTrue(a['mock_accepted']);self.assertFalse(r['mock_accepted']);self.assertFalse(a['command_issued'])
if __name__=='__main__':unittest.main()
