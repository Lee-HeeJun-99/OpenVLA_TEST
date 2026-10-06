import math,sys,unittest
from pathlib import Path
R=Path(__file__).resolve().parents[1];sys.path.insert(0,str(R/'02_safety'))
from open_loop_gripper_supervisor import OpenLoopGripperSupervisor,CommandKnowledge

class Tests(unittest.TestCase):
 def ready(self):
  g=OpenLoopGripperSupervisor();g.confirm_initial_open(True);return g
 def test_initial_unknown_blocks(self):
  d=OpenLoopGripperSupervisor().resolve(.9,phase='grasp_close')
  self.assertFalse(d.accepted);self.assertEqual('UNKNOWN',d.command_knowledge_after);self.assertFalse(d.lift_allowed)
 def test_operator_confirmation_required(self):
  with self.assertRaises(RuntimeError):OpenLoopGripperSupervisor().confirm_initial_open(False)
 def test_close_only_once_and_only_grasp_close(self):
  g=self.ready();d=g.resolve(.9,phase='alignment');self.assertFalse(d.accepted);self.assertEqual('COMMAND_OPEN',d.command_knowledge_after)
  d=g.resolve(.9,phase='grasp_close');self.assertTrue(d.accepted);self.assertEqual('CLOSE',d.candidate_command);self.assertEqual(1,d.close_count)
  d=g.resolve(.8,phase='lift');self.assertTrue(d.accepted);self.assertTrue(d.command_suppressed);self.assertTrue(d.lift_allowed)
  g.resolve(.1,phase='post_lift');d=g.resolve(.9,phase='grasp_close');self.assertFalse(d.accepted);self.assertEqual('close_limit_exceeded',d.reason)
 def test_threshold_hysteresis_and_duplicate(self):
  g=self.ready();self.assertEqual('duplicate_suppressed',g.resolve(.1,phase='alignment').reason)
  self.assertEqual('hysteresis_hold',g.resolve(.5,phase='alignment').reason)
 def test_faults_force_unknown_and_block_lift(self):
  for kwargs in ({'fresh':False},{'communication_ok':False},{'logger_ok':False},{'command_result_known':False}):
   g=self.ready();d=g.resolve(.1,phase='alignment',**kwargs);self.assertEqual('UNKNOWN',d.command_knowledge_after);self.assertFalse(d.lift_allowed)
 def test_nan_invalid(self):
  d=self.ready().resolve(math.nan,phase='grasp_close');self.assertFalse(d.accepted);self.assertEqual('UNKNOWN',d.command_knowledge_after)
 def test_no_hardware_capability(self):
  s=(R/'02_safety/open_loop_gripper_supervisor.py').read_text().lower()
  for bad in ('rclpy','create_publisher','create_client','call_async','digital_output','movej','movel'):self.assertNotIn(bad,s)

if __name__=='__main__':unittest.main()
