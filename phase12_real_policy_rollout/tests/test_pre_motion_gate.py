import sys,unittest
from pathlib import Path
R=Path(__file__).resolve().parents[1];sys.path.insert(0,str(R/'02_safety'))
from pre_motion_gate import PreMotionGate
from workspace_contract import PHASE12_DATA_DERIVED_WORKSPACE

def signals(value=True):return {k:value for k in PreMotionGate.REQUIRED_SIGNALS}
class Tests(unittest.TestCase):
 def test_all_explicit_pass_required(self):
  g=PreMotionGate(PHASE12_DATA_DERIVED_WORKSPACE,30)
  d=g.inspect(signals=signals(),elapsed_s=1,current_position_m=(.4,0,.5),camera_ok=True,joint_state_ok=True,tcp_ok=True,logger_ok=True,model_ok=True)
  self.assertTrue(d.ready);self.assertFalse(d.command_issued)
 def test_current_missing_hardware_contract_blocks(self):
  s=signals();
  for k in ('protective_stop_clear','servo_state_verified','hold_ack_verified','gripper_feedback_or_approved_no_feedback_policy','workspace_site_approved'):s[k]=False
  d=PreMotionGate(PHASE12_DATA_DERIVED_WORKSPACE,30).inspect(signals=s,elapsed_s=1,current_position_m=(.4,0,.5),camera_ok=True,joint_state_ok=True,tcp_ok=True,logger_ok=True,model_ok=True)
  self.assertFalse(d.ready);self.assertTrue(d.hold_required);self.assertFalse(d.command_issued)
  self.assertIn('hold_ack_verified',d.blockers);self.assertIn('workspace_site_approved',d.blockers)
 def test_timeout_workspace_and_data_fail_closed(self):
  g=PreMotionGate(PHASE12_DATA_DERIVED_WORKSPACE,30)
  d=g.inspect(signals=signals(),elapsed_s=31,current_position_m=(.8,0,.5),camera_ok=False,joint_state_ok=True,tcp_ok=True,logger_ok=True,model_ok=True)
  self.assertEqual(('rollout_timeout','camera_unavailable','workspace_violation'),d.blockers)
 def test_no_command_capability(self):
  s=(R/'02_safety/pre_motion_gate.py').read_text().lower()
  for bad in ('rclpy','create_publisher','create_client','call_async','movej','movel','servol','speedl'):self.assertNotIn(bad,s)
if __name__=='__main__':unittest.main()
