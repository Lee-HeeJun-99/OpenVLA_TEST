import sys, unittest
from pathlib import Path
R=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(R/'02_safety'))
from hard_command_gate import HardCommandGate, UNSAFE_FLAGS

SAFE={"mode":"command_disabled_pre_motion","include_action_adapter":False,"include_doosan_bridge":False,
      **{name:False for name in UNSAFE_FLAGS}}

class TestHardCommandGate(unittest.TestCase):
 def test_safe_config_has_zero_capabilities(self):
  gate=HardCommandGate(SAFE)
  self.assertTrue(all(value is False for key,value in gate.capability_report.items() if key != 'sink'))
 def test_every_unsafe_flag_rejected(self):
  for name in UNSAFE_FLAGS:
   config=dict(SAFE);config[name]=True
   with self.assertRaises(RuntimeError):HardCommandGate(config)
 def test_missing_flag_rejected(self):
  config=dict(SAFE);config.pop('allow_robot_command')
  with self.assertRaises(RuntimeError):HardCommandGate(config)
 def test_candidate_never_delivered(self):
  row=HardCommandGate(SAFE).audit_candidate({'action_id':'x','technical_valid':True,'technical_blockers':[]})
  self.assertTrue(row['hard_gate']['delivery_blocked']);self.assertIsNone(row['executed_action'])
  self.assertIsNone(row['robot_delivered_command']);self.assertFalse(row['command_issued'])
 def test_phase12_boundary_has_no_ros_or_command_api(self):
  text=(R/'02_safety/hard_command_gate.py').read_text().lower()
  for token in ('import rclpy','create_publisher','create_client','actionclient','call_async','movel','movej','servol','speedl'):
   self.assertNotIn(token,text)
 def test_original_command_nodes_excluded_from_phase12_config(self):
  text=(R/'01_configs/command_disabled_runtime.yaml').read_text().lower()
  self.assertIn('include_action_adapter: false',text);self.assertIn('include_doosan_bridge: false',text)

if __name__=='__main__':unittest.main()
