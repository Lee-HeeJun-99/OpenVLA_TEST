import json,tempfile,unittest
from pathlib import Path
from types import SimpleNamespace
from controller.joint_limits import load_joint_limits

BASE=json.loads(Path(__file__).with_name('pendant_limits.json').read_text())

class JointLimitTests(unittest.TestCase):
    def load(self,mutate):
        raw=json.loads(json.dumps(BASE));mutate(raw)
        with tempfile.TemporaryDirectory() as d:
            p=Path(d)/'limits.json';p.write_text(json.dumps(raw))
            return load_joint_limits(p)

    def test_host_outside_pendant_rejected(self):
        with self.assertRaisesRegex(ValueError,'HOST_STOP_NOT_STRICTLY_INSIDE'):
            self.load(lambda x:x.__setitem__('host_margin_deg_candidate',[-1]*6))

    def test_pendant_above_configurable_max_rejected(self):
        with self.assertRaisesRegex(ValueError,'PENDANT_EXCEEDS'):
            self.load(lambda x:x['pendant_upper_deg'].__setitem__(2,161))

    def test_extended_limits_require_both_evidence_gates(self):
        limits=self.load(lambda x:(x['pendant_upper_deg'].__setitem__(1,100),
                                   x.__setitem__('profile','extended'),
                                   x.__setitem__('extension_reason','test'),
                                   x.__setitem__('extended_range_clearance_record',{
                                     'date':'2026-11-13','operator':'tester',
                                     'joints_checked':['J2'],'angles_checked_deg':{'J2':[-96,96]}})))
        gate=SimpleNamespace(entries={
            'pendant_limits_recorded':SimpleNamespace(passed=False,evidence=None),
            'extended_range_clearance_checked':SimpleNamespace(passed=False,evidence=None)})
        with self.assertRaisesRegex(PermissionError,'EXTENDED_RANGE_GATE_BLOCKED'):
            limits.require_extension_gates(gate)

if __name__=='__main__':unittest.main()
