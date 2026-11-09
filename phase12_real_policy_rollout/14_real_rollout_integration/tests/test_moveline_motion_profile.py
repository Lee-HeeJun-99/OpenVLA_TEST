"""Fake transport only; a successful test is not physical validation."""
import unittest
from pathlib import Path
import yaml
from moveline_motion_profile import validate_profile
from real_sink import RealDoosanCommandSink, Authorization, FakeRosTransport
from arm_only_episode_contract import ArmOnlyEpisodeSink
from run_real_rollout import Controller


class Log:
    def append(self,row):pass


class MotionProfileTests(unittest.TestCase):
    def test_explicit_120_translation_20_rotation_request(self):
        t=FakeRosTransport();s=RealDoosanCommandSink(lambda:t,Authorization(True,True,True,'MOTION_ENABLED'))
        result=s.send_pose([400,0,500,10,20,30],velocity=[120,20],acceleration=[120,20])
        self.assertEqual(result['state'],'completed')
        self.assertEqual(t.calls[0][2]['vel'],[120.,20.])
        self.assertEqual(t.calls[0][2]['acc'],[120.,20.])
        self.assertEqual(t.calls[0][2]['sync_type'],0)

    def test_bad_bounds_or_units_refuse_before_transport(self):
        for bad in ([120,120],[120.1,20],[120,20.1],[0,20],[-1,20],[float('nan'),20],
                    [float('inf'),20],[120],[120,20,20]):
            for which in ('velocity','acceleration'):
                with self.subTest(bad=bad,which=which):
                    t=FakeRosTransport();s=RealDoosanCommandSink(lambda:t,Authorization(True,True,True,'MOTION_ENABLED'))
                    kwargs=dict(velocity=[120,20],acceleration=[120,20]);kwargs[which]=bad
                    with self.assertRaises(ValueError):s.send_pose([400,0,500,10,20,30],**kwargs)
                    self.assertEqual(t.calls,[])

    def test_legacy_default_remains_20(self):
        t=FakeRosTransport();s=RealDoosanCommandSink(lambda:t,Authorization(True,True,True,'MOTION_ENABLED'))
        s.send_pose([400,0,500,10,20,30])
        self.assertEqual(t.calls[0][2]['vel'],[20.,20.])
        self.assertEqual(t.calls[0][2]['acc'],[20.,20.])

    def test_live_config_to_controller_to_full_delta_request(self):
        profile=yaml.safe_load((Path(__file__).resolve().parents[1]/'configs/hardware_interface.yaml').read_text())
        self.assertEqual(validate_profile(profile['velocity'],profile['acceleration']),((120.,20.),(120.,20.)))
        self.assertIs(profile['moveline_delta_limiter_enabled'],False)
        t=FakeRosTransport();s=ArmOnlyEpisodeSink(lambda:t,Authorization(True,True,True,'MOTION_ENABLED'))
        c=Controller('openvla','full_task',s,Log(),dry_run=False,execution_mode='ARM_ONLY_FULL_EPISODE',pose_motion_profile=profile,
            moveline_full_delta=not profile['moveline_delta_limiter_enabled'])
        for step in range(2):
            now=1+step*3
            obs=dict(receive_monotonic=now,tcp_m_abc=[.4,0,.5,10,20,30],phase='alignment',robot_state_ok=True,
                camera_ok=True,joint_state_ok=True,tcp_ok=True,model_ok=True,communication_ok=True)
            row=c.step([.013,0,0,0,0,0,.9],obs,inference_time=now,chunk_id=str(step),chunk_index=0)
            self.assertAlmostEqual(row['filtered_action'][0],.013)
            self.assertFalse(row['delta_limiter_applied'])
            self.assertEqual(row['moveline_velocity_mm_s_deg_s'],[120.,20.])
        self.assertFalse(c.aborted)
        self.assertTrue(all(v['vel']==[120.,20.] and v['acc']==[120.,20.] for _,_,v in t.calls))
        self.assertTrue(all(name.endswith('move_line') for name,_,_ in t.calls))

    def test_raw_limit_still_inhibits_120_profile(self):
        t=FakeRosTransport();s=ArmOnlyEpisodeSink(lambda:t,Authorization(True,True,True,'MOTION_ENABLED'))
        c=Controller('openvla','full_task',s,Log(),dry_run=False,execution_mode='ARM_ONLY_FULL_EPISODE',
            pose_motion_profile=dict(velocity=[120,20],acceleration=[120,20]))
        obs=dict(receive_monotonic=1,tcp_m_abc=[.4,0,.5,10,20,30],phase='alignment',robot_state_ok=True,
            camera_ok=True,joint_state_ok=True,tcp_ok=True,model_ok=True,communication_ok=True)
        c.step([.0501,0,0,0,0,0,0],obs,inference_time=1,chunk_id='x',chunk_index=0)
        self.assertTrue(c.aborted)
        self.assertFalse(any(name.endswith('move_line') for name,_,_ in t.calls))
        count=len(t.calls)
        with self.assertRaises(RuntimeError):c.step([0]*7,obs,inference_time=1,chunk_id='y',chunk_index=0)
        self.assertEqual(len(t.calls),count)
