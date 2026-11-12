import math
import unittest
import numpy as np
from scipy.spatial.transform import Rotation
from run_real_rollout import SafetyPipeline
from action_contract import rotation_step_geodesic_rad
from servo_experimental.controller import continuous_zyz,interpolate_segment
from servo_experimental.stop_evidence import evaluate_stop


class RotationGeometryTests(unittest.TestCase):
    def test_linear_position_slerp_short_arc(self):
        start=Rotation.from_euler('ZYZ',[179.,0,0],degrees=True)
        end=Rotation.from_euler('ZYZ',[-179.,0,0],degrees=True)
        p,r=interpolate_segment([0,0,0],start,[2,4,6],end,.5)
        np.testing.assert_allclose(p,[1,2,3])
        self.assertAlmostEqual(math.degrees((start.inv()*r).magnitude()),1.)

    def test_singularities_and_wrap_reconstruct(self):
        for b in (0.,1e-9,-1e-9,179.999999999,180.,-180.,-179.999999999,155.):
            for a,c in ((179.9,-179.9),(-179.9,179.9),(380.,-400.)):
                original=Rotation.from_euler('ZYZ',[a,b,c],degrees=True)
                previous=np.array([a+2,b,c-2])
                result=continuous_zyz(original,previous)
                self.assertLess((Rotation.from_euler('ZYZ',result,degrees=True).inv()*original).magnitude(),1e-6)
                if abs(math.sin(math.radians(b)))<1e-8:
                    self.assertEqual(result[0],previous[0])

    def test_fake_jumps_pass_real_large_rotation_rejected(self):
        self.assertLess(rotation_step_geodesic_rad([10,-180,20],[190,180,200]),1e-6)
        self.assertLess(rotation_step_geodesic_rad([179.99,0,0],[-179.99,0,0]),math.radians(4))
        self.assertGreater(rotation_step_geodesic_rad([0,0,0],[5,0,0]),math.radians(4))
        for angle,expected in ((3.999,True),(4.,True),(4.001,False)):
            pipeline=SafetyPipeline('openvla',operator_confirmed_initial_open=False,arm_only=True)
            decision=pipeline.runtime.evaluate([0,0,0,0,0,math.radians(angle),1],
                action_id='r',source_monotonic=1.,now_monotonic=1.,
                current_position_m=(.4,0,.5),current_abc_deg=(179.99,-180,-179.99),
                phase='alignment',arm_only=True)
            self.assertEqual('raw_rotation_step_limit' not in decision['technical_blockers'],expected)

    def test_stop_ros_epoch_is_not_compared_to_monotonic(self):
        rows=[dict(receive=10.01+i*.01,source=1794475511.+i*.01,
                   velocity=[0.]*6,valid=True) for i in range(25)]
        result=evaluate_stop(ack=True,requested_at=10.,robot_state=dict(success=True,robot_state=1),
            state_received_at=10.24,joint_rows=rows,now=10.26)
        self.assertTrue(result['confirmed'],result)
        self.assertEqual(result['comparison_clock'],'HOST_MONOTONIC_RECEIVE_TIME')
        missing=evaluate_stop(ack=True,requested_at=10.,robot_state={},state_received_at=None,
                              joint_rows=rows,now=10.26)
        self.assertEqual(missing['reason'],'missing_stop_timestamps')


if __name__=='__main__':unittest.main()
