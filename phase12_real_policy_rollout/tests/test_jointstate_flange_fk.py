import importlib.util, math, pathlib, sys, unittest

ROOT=pathlib.Path(__file__).resolve().parents[1]
PATH=ROOT/'03_shadow_mode'/'jointstate_flange_fk.py'
spec=importlib.util.spec_from_file_location('jointstate_flange_fk',PATH);m=importlib.util.module_from_spec(spec);sys.modules[spec.name]=m;spec.loader.exec_module(m)
URDF='/home/ubuntu/robot_ws/src/doosan-robot2/dsr_description2/urdf/a0509.urdf'

class TestFlangeFK(unittest.TestCase):
    def setUp(self): self.fk=m.A0509FlangeFK(URDF)
    def test_chain_and_zero_pose(self):
        out=self.fk.compute({n:0.0 for n in m.CANONICAL_JOINTS})
        self.assertEqual(out['child_frame'],'link_6'); self.assertFalse(out['valid_as_measured_tcp'])
        self.assertFalse(out['valid_for_motion_readiness']); self.assertFalse(out['tool_offset_applied'])
        self.assertTrue(all(math.isfinite(x) for x in out['position_m']+out['quaternion_wxyz']))
        self.assertAlmostEqual(sum(x*x for x in out['quaternion_wxyz']),1.0,places=12)
    def test_name_reorder(self):
        names=['joint_1','joint_2','joint_4','joint_5','joint_3','joint_6']
        pos=[1,2,4,5,3,6]
        out=self.fk.compute(m.reorder_joint_state(names,pos))
        self.assertEqual(out['joint_position_rad'],[1,2,3,4,5,6])
    def test_missing_and_nonfinite_rejected(self):
        with self.assertRaises(ValueError): self.fk.compute({'joint_1':0})
        q={n:0.0 for n in m.CANONICAL_JOINTS}; q['joint_4']=float('nan')
        with self.assertRaises(ValueError): self.fk.compute(q)
    def test_no_command_capability(self):
        text=PATH.read_text()
        for token in ('create_publisher','create_client','create_service','ActionClient','movej','movel','servo_off'):
            self.assertNotIn(token,text)

if __name__=='__main__': unittest.main()
