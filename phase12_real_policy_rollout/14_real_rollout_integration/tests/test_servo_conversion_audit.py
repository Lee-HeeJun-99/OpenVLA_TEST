import unittest
from audit_servo_conversion import joint_crossing,rotation_error
from scipy.spatial.transform import Rotation


class AuditTests(unittest.TestCase):
    def test_no_pretrial_crossing_or_future_publish_before_cross(self):
        import math
        def row(t,q):return dict(receive=t,valid=True,names=['joint_5'],position=[math.radians(q)])
        rows=[row(0,-140),row(1,-134),row(2,-135),row(3,-137)]
        cmds=[dict(published_monotonic=t) for t in [1.5,2.0,2.1,2.5]]
        r=joint_crossing(rows,cmds,1,-135)
        self.assertEqual(r['first_crossing_monotonic'],2)
        self.assertEqual(r['publish_returns_after_first_crossing'],3)
    def test_no_crossing_is_not_fabricated(self):
        import math
        r=joint_crossing([dict(receive=1,valid=True,names=['joint_5'],position=[math.radians(-130)])],[],0,-135)
        self.assertIsNone(r['first_crossing_monotonic']);self.assertEqual(r['publish_returns_after_first_crossing'],0)
    def test_euler_representation_flip_is_not_physical_jump(self):
        a=Rotation.from_euler('ZYZ',[179,-155,3],degrees=True)
        b=Rotation.from_euler('ZYZ',[-1,155,-177],degrees=True)
        self.assertLess(rotation_error(a,b),1e-10)
