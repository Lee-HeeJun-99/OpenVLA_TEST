import sys,unittest
from pathlib import Path
R=Path(__file__).resolve().parents[1]
class Tests(unittest.TestCase):
 def test_offline_pipeline_has_no_ros_command_api(self):
  files=['canonical_action.py','command_sinks.py','tcp_sources.py','safety_pipeline.py','pre_motion_runtime_facade.py','hard_command_gate.py']
  for name in files:
   text=(R/'02_safety'/name).read_text().lower()
   for token in ('import rclpy','create_publisher','create_client','actionclient','call_async','servolstream','speedlstream','moveline'):
    self.assertNotIn(token,text,(name,token))
 def test_future_real_config_disabled(self):
  t=(R/'01_configs/future_real_runtime.yaml').read_text().lower()
  for line in ('motion_enabled: false','command_sink: null','explicit_motion_approval: false','create_ros_command_capability: false'):self.assertIn(line,t)
if __name__=='__main__':unittest.main()
