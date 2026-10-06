import glob,json,math,sys,unittest
from pathlib import Path
R=Path(__file__).resolve().parents[1];sys.path.insert(0,str(R/'02_safety'))
from workspace_contract import CartesianWorkspace,PHASE12_DATA_DERIVED_WORKSPACE
class Tests(unittest.TestCase):
 def test_bounds_and_delta(self):
  w=PHASE12_DATA_DERIVED_WORKSPACE
  self.assertTrue(w.inspect((.4,0,.5)).accepted)
  self.assertEqual('workspace_violation',w.inspect((.6,0,.5)).reason)
  self.assertEqual('workspace_violation',w.inspect_delta((.553,0,.5),(.002,0,0)).reason)
 def test_invalid_and_command_free(self):
  self.assertEqual('invalid_tcp_position',PHASE12_DATA_DERIVED_WORKSPACE.inspect((math.nan,0,0)).reason)
  s=(R/'02_safety/workspace_contract.py').read_text().lower()
  for bad in ('rclpy','create_publisher','create_client','call_async','movej','movel','servol','speedl'):self.assertNotIn(bad,s)
 def test_all_ten_reference_episode_planned_poses_inside_candidate(self):
  files=sorted(glob.glob('/home/ubuntu/a0509_vla_linux_field_bundle_20260903/data/real_world/raw_dataset_oft/episodes/episode_[0-9][0-9][0-9][0-9][0-9][0-9]/steps_with_actions.jsonl'))
  self.assertEqual(10,len(files));count=0
  for p in files:
   with open(p) as stream:
    for line in stream:
     row=json.loads(line);self.assertEqual('planned_actual_duration',row['pose_source'])
     self.assertTrue(PHASE12_DATA_DERIVED_WORKSPACE.inspect(row['tcp_pose'][:3]).accepted);count+=1
  self.assertEqual(452,count)
if __name__=='__main__':unittest.main()
