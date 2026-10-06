#!/usr/bin/env python3
import csv, json, unittest
from pathlib import Path

ROOT=Path(__file__).resolve().parents[1]
P11=ROOT.parent
class Phase11OfflineTests(unittest.TestCase):
    def test_same_episode_pairing(self):
        with (ROOT/'03_alignment/aligned_samples.csv').open() as handle:
            rows=list(csv.DictReader(handle))
        self.assertTrue(rows)
        self.assertTrue(all(r['episode_id'].startswith('episode_') for r in rows))
    def test_no_excluded_pairs_after_lighting_recollection(self):
        x=json.loads((ROOT/'00_audit/excluded_samples.json').read_text())
        self.assertEqual([],x)
    def test_valid_pair_count(self):
        x=json.loads((ROOT/'00_audit/final_valid_pairs.json').read_text())
        self.assertEqual(15,x['valid_pair_count'])
        self.assertEqual(5,sum(r['condition']=='lighting_low' for r in x['pairs']))
    def test_no_robot_control_code(self):
        for p in (ROOT/'scripts').glob('*'):
            if p.is_file():
                s=p.read_text(errors='ignore')
                for forbidden in ('rclpy','move_line(','move_joint(','gripper.close(','--execute'):
                    self.assertNotIn(forbidden,s)
    def test_prediction_execution_null_contract(self):
        runner=Path('/home/ubuntu/a0509_vla_linux_field_bundle_20260903/lhj/phase10_planner_based_shadow_mode/06_shadow_collection/shadow_mode_runner.py').read_text()
        self.assertIn('"openvla_executed_action": None',runner)
        self.assertIn('"oft_executed_action": None',runner)
    def test_actual_feature_manifests_complete(self):
        expected={'baseline':228,'lighting_low':227,'extra_object':229,'distractor_swap':228}
        for model in ('openvla','oft'):
            for condition,count in expected.items():
                x=json.loads((ROOT/f'02_features/{model}/{condition}/feature_manifest.json').read_text())
                self.assertTrue(x['full_features'])
                self.assertEqual(count,x['sample_count'])
if __name__=='__main__': unittest.main()
