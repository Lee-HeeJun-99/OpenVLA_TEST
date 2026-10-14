import unittest
from run_arm_only_once import require_operator_confirmation

class ApprovalTests(unittest.TestCase):
    def evidence(self):return dict(operator_present=True,workspace_clear=True,estop_accessible=True,robot_stationary=True,tcp_arm_only_acceptance=True,explicit_one_arm_command=True,confirmed_wall_time=100)
    def test_fresh(self):require_operator_confirmation(self.evidence(),110)
    def test_expired(self):
        with self.assertRaises(PermissionError):require_operator_confirmation(self.evidence(),161)
    def test_unconfirmed(self):
        record=self.evidence();record['workspace_clear']=None
        with self.assertRaises(PermissionError):require_operator_confirmation(record,110)
