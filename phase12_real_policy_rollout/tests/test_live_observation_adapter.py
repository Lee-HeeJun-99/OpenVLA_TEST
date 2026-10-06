import ast,importlib.util,math,unittest
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
spec=importlib.util.spec_from_file_location('ro',ROOT/'03_shadow_mode/read_only_state_adapter.py');m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m)
class Tests(unittest.TestCase):
 def test_allowlist_rejects_desired_and_motion(self):
  for n,t in [('/dsr01/aux_control/get_desired_posx','dsr_msgs2/srv/GetDesiredPosx'),('/dsr01/motion/move_line','dsr_msgs2/srv/MoveLine')]:
   with self.assertRaises(RuntimeError):m.require_allowed_service(n,t)
 def test_name_based_order_effort_nan_allowed(self):
  names=['joint_1','joint_2','joint_4','joint_5','joint_3','joint_6'];p=[1,2,4,5,3,6];v=[.1,.2,.4,.5,.3,.6]
  r=m.canonicalize_joint_state(names,p,v,[math.nan]*6,10)
  self.assertEqual(r['position'],[1,2,3,4,5,6]);self.assertIsNone(r['effort'])
 def test_missing_nonfinite_and_stale_rejected(self):
  with self.assertRaises(ValueError):m.canonicalize_joint_state(['joint_1'],[1],[1],[],1)
  with self.assertRaises(ValueError):m.canonicalize_joint_state(list(m.CANONICAL_JOINTS),[0]*5+[math.nan],[0]*6,[],1)
  with self.assertRaises(ValueError):m.canonicalize_joint_state(list(m.CANONICAL_JOINTS),[0]*6,[0]*6,[],1,1)
 def test_tcp_raw_preserved_and_null_execution(self):
  r=m.tcp_record([1,2,3,4,5,6,7],success=True,receive_ros_ns=8,receive_monotonic_ns=9,latency_sec=.1)
  self.assertEqual(r['abc_deg_raw'],[4,5,6]);self.assertIsNone(r['source_timestamp']);self.assertIsNone(r['executed_action']);self.assertFalse(r['command_issued'])
 def test_live_script_has_only_allowlisted_clients(self):
  text=(ROOT/'03_shadow_mode/query_doosan_state_once.py').read_text();tree=ast.parse(text)
  for banned in ('create_publisher','ActionClient','/motion/','get_desired_posx','servo_off','gripper','move_home'):
   self.assertNotIn(banned,text)
 def test_stability_monitor_is_passive(self):
  text=(ROOT/'03_shadow_mode/monitor_doosan_state_stack.py').read_text()
  for banned in ('create_publisher','create_client','call_async','ActionClient','/motion/','set_','servo_off'):
   self.assertNotIn(banned,text)
if __name__=='__main__':unittest.main()
