import sys,unittest
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'02_safety'))
from open_loop_gripper_supervisor import (CommandKnowledge,GripperRuntimeContext,
                                           OpenLoopGripperSupervisor)

def context(*,accepted=False,communication=True,channel=True,known=True,executed=False,fresh=True):
 return GripperRuntimeContext(fresh,accepted,communication,channel,known,executed)

class GripperStateDecouplingTest(unittest.TestCase):
 def open_supervisor(self):
  g=OpenLoopGripperSupervisor();g.confirm_initial_open(True);return g
 def assert_reject_preserves_open(self,closedness=.8,phase='grasp_close'):
  g=self.open_supervisor();d=g.resolve_with_context(closedness,phase=phase,context=context(accepted=False))
  self.assertEqual(d.command_knowledge_before,'COMMAND_OPEN');self.assertEqual(d.command_knowledge_after,'COMMAND_OPEN')
  self.assertFalse(d.candidate_executed);self.assertFalse(d.state_invalidated)
  self.assertIsNone(d.measured_gripper_state)
 def test_gripper_state_preserved_on_translation_reject(self):self.assert_reject_preserves_open()
 def test_gripper_state_preserved_on_rotation_reject(self):self.assert_reject_preserves_open()
 def test_gripper_state_preserved_on_acceleration_reject(self):self.assert_reject_preserves_open()
 def test_gripper_state_preserved_on_workspace_reject(self):self.assert_reject_preserves_open()
 def test_gripper_state_preserved_on_premature_close(self):
  g=self.open_supervisor();d=g.resolve_with_context(.8,phase='alignment',context=context(accepted=True))
  self.assertEqual(d.reason,'close_forbidden_outside_grasp_close');self.assertEqual(d.command_knowledge_after,'COMMAND_OPEN')
  self.assertFalse(d.state_invalidated);self.assertFalse(d.candidate_executed)
 def test_gripper_state_invalidated_on_true_channel_failure(self):
  g=self.open_supervisor();d=g.resolve_with_context(.2,phase='alignment',context=context(channel=False))
  self.assertEqual(d.command_knowledge_after,'UNKNOWN');self.assertTrue(d.state_invalidated)
  self.assertEqual(d.state_invalidation_reason,'command_channel_failure')
 def test_unknown_initial_state_stays_unknown(self):
  g=OpenLoopGripperSupervisor();d=g.resolve_with_context(.2,phase='hold',context=context(known=False))
  self.assertEqual(d.command_knowledge_after,'UNKNOWN');self.assertFalse(d.candidate_executed)
 def test_command_state_never_becomes_measured_state(self):
  g=self.open_supervisor();d=g.resolve_with_context(.8,phase='grasp_close',context=context(accepted=True,executed=True))
  self.assertEqual(d.command_knowledge_after,'COMMAND_CLOSED');self.assertIsNone(d.measured_gripper_state)
 def test_rejected_candidate_not_applied(self):
  g=self.open_supervisor();d=g.resolve_with_context(.8,phase='grasp_close',context=context(accepted=False))
  self.assertEqual(d.candidate_command,'CLOSE');self.assertFalse(d.candidate_executed)
  self.assertEqual(g.state,CommandKnowledge.COMMAND_OPEN);self.assertEqual(g.close_count,0)

if __name__=='__main__':unittest.main()
