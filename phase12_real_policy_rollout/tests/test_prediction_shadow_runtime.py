import sys,unittest
from pathlib import Path
R=Path(__file__).resolve().parents[1];sys.path[:0]=[str(R/'02_safety'),str(R/'03_shadow_mode')]
from canonical_action import CanonicalAction
from mock_closed_loop import MockClosedLoop,MockRobotState
class Tests(unittest.TestCase):
 def test_mock_path_updates_only_mock_state(self):
  a=CanonicalAction.from_vector([.001,0,0,0,0,0,0],timestamp_monotonic=.2,sequence_id='a',source_model='mock')
  rows=MockClosedLoop().run(MockRobotState([.4,0,.5],[10,20,30]),[a]);self.assertAlmostEqual(.4008,rows[0]['mock_state']['position_m'][0]);self.assertFalse(rows[0]['command_issued'])
if __name__=='__main__':unittest.main()
