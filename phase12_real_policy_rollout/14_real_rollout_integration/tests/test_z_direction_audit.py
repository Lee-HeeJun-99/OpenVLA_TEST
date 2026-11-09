import unittest
import numpy as np
from analyze_z_direction import direction, agrees, before, trajectory, ROOT, A0509FlangeFK


class OfflineAuditTests(unittest.TestCase):
    def test_direction_units_and_unknown(self):
        self.assertEqual(direction(-.3),'DOWN')
        self.assertEqual(direction(.3),'UP')
        self.assertEqual(direction(0),'ZERO')
        self.assertIsNone(agrees(None,.3))
        self.assertFalse(agrees(-.3,.3))

    def test_matching_never_selects_future_joint_sample(self):
        times=np.array([1.,1.01,1.02])
        self.assertEqual(before(times,1.019),1)
        self.assertEqual(before(times,1.02),2)

    def test_batch_fk_checks_real_saved_joint_order_without_ros(self):
        fk=A0509FlangeFK('/home/ubuntu/robot_ws/src/doosan-robot2/dsr_description2/urdf/a0509.urdf')
        rows,times,xyz=trajectory(ROOT/'real_trials/20261106_132009_arm_only_moveline_episode',fk)
        self.assertGreater(len(rows),1000)
        self.assertEqual(xyz.shape,(len(rows),3))
        expected=fk.compute(dict(zip(rows[0]['names'],rows[0]['position'])))['position_m']
        np.testing.assert_allclose(xyz[0],np.array(expected)*1000,atol=1e-9,rtol=0)
