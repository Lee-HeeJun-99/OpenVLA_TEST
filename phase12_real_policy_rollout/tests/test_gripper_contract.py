import sys,unittest
from pathlib import Path
R=Path(__file__).resolve().parents[1];sys.path.insert(0,str(R/'02_safety'))
from open_loop_gripper_supervisor import OpenLoopGripperSupervisor
class Tests(unittest.TestCase):
 def test_canonical_polarity_unknown_measured(self):
  g=OpenLoopGripperSupervisor();g.confirm_initial_open(True)
  self.assertEqual('COMMAND_OPEN',g.resolve(.2,phase='alignment').command_knowledge_after)
  d=g.resolve(.8,phase='grasp_close');self.assertEqual('COMMAND_CLOSED',d.command_knowledge_after);self.assertIsNone(d.measured_gripper_state)
if __name__=='__main__':unittest.main()
