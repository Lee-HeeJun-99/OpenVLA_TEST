import importlib.util,json,pathlib,sys,tempfile,unittest
R=pathlib.Path(__file__).resolve().parents[1];P=R/'03_shadow_mode'/'offline_contract_integration.py'
spec=importlib.util.spec_from_file_location('offline_contract_integration',P);m=importlib.util.module_from_spec(spec);sys.modules[spec.name]=m;spec.loader.exec_module(m)

class TestIntegration(unittest.TestCase):
 def test_actual_recorded_predictions_command_free(self):
  base=pathlib.Path('/home/ubuntu/a0509_vla_linux_field_bundle_20260903/lhj/phase11_sim_condition_replication/policy_sensitive_analysis/01_predictions')
  ov=base/'openvla/baseline/episode_000001/samples.jsonl';of=base/'oft_run2/baseline/episode_000001/samples.jsonl'
  with tempfile.TemporaryDirectory() as d:
   out=pathlib.Path(d)/'audit.jsonl';s=m.run(ov,of,'/home/ubuntu/robot_ws/src/doosan-robot2/dsr_description2/urdf/a0509.urdf',out)
   self.assertEqual(6,s['records']);self.assertEqual([0,1,2,3,4],s['oft_chunk_indices'])
   self.assertTrue(s['all_executed_action_null']);self.assertTrue(s['all_robot_delivered_command_null']);self.assertTrue(s['all_command_issued_false'])
   rows=[json.loads(x) for x in out.read_text().splitlines()]
   self.assertTrue(all(not r['command_candidate']['legacy_2800_gain_used'] for r in rows))
   self.assertTrue(all(r['pose_proxy']['pose_kind']=='computed_link6_flange_not_measured_tcp' for r in rows))
   self.assertTrue(all(r['safety_limits_status']=='OPERATOR_SELECTED_20_PROFILE_OFFLINE_ONLY_NOT_MOTION_APPROVED' for r in rows))
   self.assertTrue(all(r['safety_limits']['max_translation_step_m']==.004 for r in rows))
   self.assertTrue(all(r['safety_limits']['max_translation_velocity_m_s']==.02 for r in rows))
   self.assertTrue(all(r['workspace_result']['accepted'] for r in rows))
  self.assertTrue(all(r['workspace_result']['status']=='DATA_DERIVED_AND_OPERATOR_APPROVED_2026_10_02' for r in rows))
 def test_no_command_or_ros_capability(self):
  text=P.read_text().lower()
  for bad in ('rclpy','create_publisher','create_client','actionclient','movej','movel','servol','speedl','call_async'):
   self.assertNotIn(bad,text)
if __name__=='__main__':unittest.main()
