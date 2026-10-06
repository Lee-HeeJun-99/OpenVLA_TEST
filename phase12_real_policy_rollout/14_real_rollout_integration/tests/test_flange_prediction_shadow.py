import unittest
from types import SimpleNamespace
from flange_prediction_shadow import joint_gate,run

class FlangeShadowTests(unittest.TestCase):
    def samples(self):return [dict(receive=i/100,source=i/100,valid=True) for i in range(2001)]
    def test_recent_continuity(self):self.assertEqual(joint_gate(self.samples(),20)['status'],'PASS')
    def test_source_gap_blocks(self):
        r=self.samples();r[1000]['source']+=3
        self.assertEqual(joint_gate(r,20)['status'],'FAIL')
    def test_stale_blocks(self):self.assertEqual(joint_gate(self.samples(),21)['status'],'FAIL')
    def test_non_dry_rejected_before_ros(self):
        with self.assertRaises(PermissionError):run(SimpleNamespace(dry_run=False,live=True,model='openvla',protocol='short_horizon'))
