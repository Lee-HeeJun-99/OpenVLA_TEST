import unittest

import numpy as np
from scipy.spatial.transform import Rotation

from kinematics import (
    DLSIKSolver, IKJointChangeError, Pose, PyKDLKinematics,
    deg_to_rad, rad_to_deg,
)


class SolverTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.kin = PyKDLKinematics()
        cls.ik = DLSIKSolver(cls.kin, damping=1e-3, max_iterations=300)
        cls.lo = cls.kin.urdf_lower_rad
        cls.hi = cls.kin.urdf_upper_rad

    def solve(self, target, seed, max_change=np.deg2rad(40.0)):
        # 40 deg is test input only, not a proposed real-robot safety limit.
        return self.ik.solve(target, seed, joint_lower_rad=self.lo,
                             joint_upper_rad=self.hi,
                             max_joint_change_rad=max_change)

    def test_degree_round_trip(self):
        q = np.array([-180, -1, 0, 1, 180, 45.5])
        np.testing.assert_allclose(rad_to_deg(deg_to_rad(q)), q)

    def test_fk_ik_round_trip_and_seed_locality(self):
        q = np.deg2rad([-170, -10, -35, 15, -80, 20])
        seed = q + np.deg2rad([1, -1, 1, -1, 1, -1])
        solved = self.solve(self.kin.fk(q), seed)
        pose_error = DLSIKSolver._error(self.kin.fk(solved), self.kin.fk(q))
        self.assertLess(np.linalg.norm(pose_error[:3]), 2e-6)
        self.assertLess(np.linalg.norm(pose_error[3:]), 2e-6)
        self.assertLess(np.linalg.norm(solved - seed), np.deg2rad(5))

    def test_near_wrist_singularity_j5_zero(self):
        q = np.deg2rad([-170, -12, -40, 20, 0.2, 15])
        seed = q + np.deg2rad([0.1, -0.1, 0.1, 0.1, 0.1, -0.1])
        solved = self.solve(self.kin.fk(q), seed)
        self.assertLess(np.linalg.norm(DLSIKSolver._error(
            self.kin.fk(solved), self.kin.fk(q))), 5e-6)

    def test_near_urdf_joint_limit(self):
        q = np.array([0.0, 0.0, self.hi[2] - 0.01, 0.0, 0.3, 0.0])
        seed = q.copy(); seed[2] -= 0.005
        solved = self.solve(self.kin.fk(q), seed)
        self.assertTrue(np.all(solved >= self.lo) and np.all(solved <= self.hi))

    def test_excessive_change_is_rejected_not_clamped(self):
        q = np.deg2rad([-170, -10, -35, 15, -80, 20])
        target_q = q.copy(); target_q[0] += np.deg2rad(2)
        with self.assertRaises(IKJointChangeError):
            self.solve(self.kin.fk(target_q), q, max_change=np.deg2rad(0.5))


if __name__ == "__main__":
    unittest.main()
