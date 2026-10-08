import unittest
from run_joint_start_pose_once import TARGET, check_approval, ordered_degrees
import math

class StartPoseTests(unittest.TestCase):
    def approval(self):
        keys=('operator_present','workspace_clear','estop_accessible','robot_stationary',
              'authority','servo','protective_stop_clear','emergency_stop_clear','explicit_joint_reset_approval')
        return dict(**{k:True for k in keys},robot_model='a0509',target_joint_deg=TARGET,approved_wall_time=100)
    def test_joint_names_not_array_order(self):
        names=['joint_1','joint_2','joint_4','joint_5','joint_3','joint_6']
        actual=ordered_degrees(dict(names=names,position=[math.radians(int(n[-1])) for n in names]))
        for a,b in zip(actual,[1,2,3,4,5,6]):self.assertAlmostEqual(a,b)
    def test_actual_confirmation_required(self):
        a=self.approval();a['workspace_clear']=False
        with self.assertRaises(PermissionError):check_approval(a,101)
    def test_unknown_not_approval(self):
        a=self.approval();a['servo']='UNKNOWN'
        with self.assertRaises(PermissionError):check_approval(a,101)
    def test_expiry(self):
        with self.assertRaises(PermissionError):check_approval(self.approval(),161)
    def test_wrong_model(self):
        a=self.approval();a['robot_model']='m1013'
        with self.assertRaises(PermissionError):check_approval(a,101)
    def test_target_is_exact(self):
        self.assertEqual(TARGET,[-182.55277,-1.07182,-41.00585,1.25662,-113.50009,1.49975])
    def test_approved(self):
        check_approval(self.approval(),101)
