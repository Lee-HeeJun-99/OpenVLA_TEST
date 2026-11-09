import ast,math,unittest
from pathlib import Path
import numpy as np
import yaml
from scipy.spatial.transform import Rotation
from servo_experimental.controller import ServoShadowController,continuous_zyz

LIMITS=yaml.safe_load((Path(__file__).resolve().parents[2]/'01_configs/safety_limits.yaml').read_text())


class ServoShadowTests(unittest.TestCase):
    def make(self):return ServoShadowController([400,0,500,179,-155,3],LIMITS)
    def test_finite_and_workspace(self):
        for value in [float('nan'),float('inf')]:
            with self.assertRaises(ValueError):self.make().set_target([value,0,500,0,0,0],0,0)
        c=self.make()
        with self.assertRaisesRegex(ValueError,'workspace'):c.set_target([999,0,500,0,0,0],0,0)
        self.assertTrue(c.aborted)
    def test_speed_and_actual_dt_acceleration(self):
        c=self.make();c.set_target([450,0,500,180,-154,4],0,0);previous=np.zeros(3)
        for i in range(50):
            row=c.tick(i*.02);v=np.array(row['translation_velocity_mm_s'])
            self.assertLessEqual(np.linalg.norm(v),20+1e-9)
            self.assertLessEqual(np.linalg.norm(v-previous),20*row['actual_tick_dt_s']+1e-9)
            self.assertLessEqual(np.linalg.norm(row['rotation_velocity_rad_s']),math.radians(20)+1e-9)
            self.assertFalse(row['command_issued']);self.assertIsNone(row['ack']);previous=v
    def test_fresh_target_does_not_reset_velocity(self):
        c=self.make();c.set_target([450,0,500,179,-155,3],0,0)
        for i in range(10):c.tick(i*.02)
        v=c.v.copy();c.set_target([451,1,500,179,-155,3],.19,.19)
        np.testing.assert_equal(c.v,v)
    def test_transport_tuning_does_not_raise_planning_acceleration(self):
        profile=dict(velocity_mm_s_deg_s=[20.,20.],acceleration_mm_s2_deg_s2=[40.,40.],target_time_s=.7)
        c=ServoShadowController([400,0,500,179,-155,3],LIMITS,transport=profile)
        c.set_target([450,0,500,180,-154,4],0,0)
        previous=np.zeros(3)
        for i in range(20):
            row=c.tick(i*.05)
            self.assertEqual(row['vel'],[20.,20.]);self.assertEqual(row['acc'],[40.,40.])
            self.assertEqual(row['time'],.7)
            v=np.array(row['translation_velocity_mm_s'])
            self.assertLessEqual(np.linalg.norm(v-previous),20*row['actual_tick_dt_s']+1e-9)
            self.assertAlmostEqual(row['planning_acceleration_mm_s2_deg_s2'][0],20.)
            previous=v
    def test_invalid_transport_profile_rejected(self):
        profile=dict(velocity_mm_s_deg_s=[20.,20.],acceleration_mm_s2_deg_s2=[40.,40.],target_time_s=.7)
        for field,value in [('target_time_s',float('nan')),('target_time_s',0),
                            ('acceleration_mm_s2_deg_s2',[float('inf'),40]),
                            ('velocity_mm_s_deg_s',[10,20]),('velocity_mm_s_deg_s',[20])]:
            with self.subTest(field=field,value=value):
                with self.assertRaises(ValueError):
                    ServoShadowController([400,0,500,179,-155,3],LIMITS,transport=dict(profile,**{field:value}))
    def test_stale_and_idempotent_stop(self):
        c=self.make();c.set_target([401,0,500,179,-155,3],0,0)
        for i in range(60):c.tick(i*.02)
        with self.assertRaisesRegex(RuntimeError,'stale'):c.tick(1.21)
        c.abort('again');self.assertEqual(c.stop_requests,1)
        with self.assertRaisesRegex(RuntimeError,'inhibited'):c.tick(1.22)
    def test_sensor_fault_inhibits(self):
        c=self.make()
        with self.assertRaisesRegex(RuntimeError,'sensor'):c.tick(0,health_ok=False)
        with self.assertRaisesRegex(RuntimeError,'inhibited'):c.set_target([400,0,500,179,-155,3],0,0)
    def test_tick_gap(self):
        c=self.make();c.tick(0)
        with self.assertRaisesRegex(RuntimeError,'tick_gap'):c.tick(.101)
    def test_rotation_branch_and_matrix(self):
        previous=np.array([179.,-155.,3.]);r=Rotation.from_euler('ZYZ',previous,degrees=True)
        abc=continuous_zyz(r,previous);np.testing.assert_allclose(abc,previous,atol=1e-8)
        self.assertLess((Rotation.from_euler('ZYZ',abc,degrees=True)*r.inv()).magnitude(),1e-8)
    def test_no_publisher_or_io_transport(self):
        source=(Path(__file__).resolve().parents[1]/'run_servo_shadow.py').read_text()
        tree=ast.parse(source)
        calls=[n.func.attr for n in ast.walk(tree) if isinstance(n,ast.Call) and isinstance(n.func,ast.Attribute)]
        self.assertNotIn('create_publisher',calls);self.assertNotIn('publish',calls)
