import math,sys,unittest
from pathlib import Path
R=Path(__file__).resolve().parents[1];sys.path.insert(0,str(R/'02_safety'))
from command_disabled_runtime import CommandDisabledPolicyRuntime

POSE=dict(current_position_m=(.4,0,.5),current_abc_deg=(179.74,-155.72,3.30))
class Tests(unittest.TestCase):
 def test_openvla_candidate_null_delivery(self):
  r=CommandDisabledPolicyRuntime('openvla',operator_confirmed_initial_open=True).evaluate([.001,0,0,0,0,0,0],action_id='a',source_monotonic=1,now_monotonic=1,phase='alignment',**POSE)
  self.assertTrue(r['technical_valid']);self.assertIsNone(r['executed_action']);self.assertIsNone(r['robot_delivered_command']);self.assertFalse(r['command_issued']);self.assertFalse(r['pre_motion_ready'])
 def test_oft_k5_order(self):
  q=CommandDisabledPolicyRuntime('oft',operator_confirmed_initial_open=True);q.enqueue_oft([[.001,0,0,0,0,0,0] for _ in range(5)],'c',1)
  rows=[q.next_oft(1+.2*i,phase='alignment',**POSE) for i in range(5)]
  self.assertEqual([0,1,2,3,4],[r['chunk_index'] for r in rows]);self.assertTrue(all(not r['command_issued'] for r in rows))
 def test_premature_close_and_fault_injection(self):
  r=CommandDisabledPolicyRuntime('openvla',operator_confirmed_initial_open=True).evaluate([0,0,0,0,0,0,.9],action_id='a',source_monotonic=1,now_monotonic=1,phase='alignment',**POSE)
  self.assertIn('premature_gripper_close',r['technical_blockers']);self.assertTrue(r['hold_required'])
  r=CommandDisabledPolicyRuntime('openvla',operator_confirmed_initial_open=True).evaluate([0]*7,action_id='b',source_monotonic=1,now_monotonic=1,phase='alignment',camera_ok=False,**POSE)
  self.assertIn('camera_failure',r['technical_blockers']);self.assertFalse(r['command_issued'])
 def test_no_delivery_capability(self):
  s=(R/'02_safety/command_disabled_runtime.py').read_text().lower()
  for bad in ('rclpy','create_publisher','create_client','call_async','movej','movel','servol','speedl','actionclient'):self.assertNotIn(bad,s)
  self.assertFalse(hasattr(CommandDisabledPolicyRuntime,'publish'));self.assertFalse(hasattr(CommandDisabledPolicyRuntime,'execute'))
if __name__=='__main__':unittest.main()
