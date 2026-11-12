"""No ROS: raw endpoint invariants and fail-closed fake transport tests."""
import math
from pathlib import Path
import unittest
from unittest.mock import patch
import numpy as np
from scipy.spatial.transform import Rotation
import yaml
from run_real_rollout import Controller, SafetyPipeline, RuntimeState, CanonicalAction
from arm_only_episode_contract import ArmOnlyEpisodeSink
from real_sink import Authorization, FakeRosTransport
from full_policy_live_mode import FullPolicyLiveMode


class Log:
    def append(self,row):pass


class FullDeltaTests(unittest.TestCase):
    def setup_path(self,full=False,raw_gate_enabled=None):
        profile=yaml.safe_load((Path(__file__).resolve().parents[1]/'configs/hardware_interface.yaml').read_text())
        transport=FakeRosTransport()
        if full:
            import time
            mode=FullPolicyLiveMode('UNUSED_TEST_FILE')
            approval=dict(execution_mode='FULL_POLICY_ROLLOUT',confirmed_wall_time=time.time(),
                **dict.fromkeys(('operator_present','workspace_clear','estop_accessible','robot_stationary',
                                'tcp_engineering_acceptance','gripper_mapping_confirmed','explicit_one_episode'),True))
            sink=mode.make_sink(lambda:transport,approval)
        else:sink=ArmOnlyEpisodeSink(lambda:transport,Authorization(True,True,True,'MOTION_ENABLED'))
        c=Controller('openvla','full_task',sink,Log(),dry_run=False,
            execution_mode='FULL_POLICY_ROLLOUT' if full else 'ARM_ONLY_FULL_EPISODE',
            gripper_contract=dict(abort_behavior='BOTH_OUTPUTS_OFF',operator_approved=True) if full else None,
            pose_motion_profile=profile,moveline_full_delta=True,
            moveline_raw_translation_gate_enabled=profile['moveline_raw_translation_gate_enabled'] if raw_gate_enabled is None else raw_gate_enabled)
        return c,transport

    def step(self,c,raw,**overrides):
        now=1+c.count*.3
        obs=dict(receive_monotonic=now,tcp_m_abc=[.4,0,.5,10,20,30],phase='alignment',robot_state_ok=True,
                 camera_ok=True,joint_state_ok=True,tcp_ok=True,model_ok=True,communication_ok=True)
        obs.update(overrides)
        return c.step(raw,obs,inference_time=now,chunk_id=str(c.count),chunk_index=0)

    def test_13mm_and_immediate_sign_reversal_not_scaled(self):
        c,t=self.setup_path()
        for value in (.013,-.013,.020):
            raw=[value,0,0,0,0,0,.9];r=self.step(c,raw)
            self.assertFalse(c.aborted)
            for name in ('RAW_MODEL_ACTION','POST_HARD_GATE','POST_VELOCITY_LIMIT','POST_ACCELERATION_LIMIT'):
                self.assertEqual(r[name],raw)
            self.assertEqual(r['FINAL_DELIVERED_ARM_ACTION'],raw[:6])
            self.assertFalse(r['delta_limiter_applied'])
            self.assertAlmostEqual(r['command_receipt']['request']['pos'][0],400+value*1000)
            self.assertEqual(r['command_receipt']['request']['vel'],[120.,20.])
            self.assertEqual(r['command_receipt']['request']['acc'],[120.,20.])
            self.assertEqual(r['command_receipt']['request']['sync_type'],0)
        self.assertEqual(len(t.calls),3)
        self.assertEqual(c.pipeline.runtime.limiter.previous_translation_velocity,(0.,0.,0.))

    def test_full_delta_rotation_is_raw_world_rotvec_not_euler_addition(self):
        c,t=self.setup_path();raw=[.003,-.004,.012,.01,-.02,.03,.1]
        r=self.step(c,raw)
        target=r['command_receipt']['request']['pos']
        np.testing.assert_allclose(target[:3],[403,-4,512],atol=1e-10)
        expected=Rotation.from_rotvec(raw[3:6])*Rotation.from_euler('ZYZ',[10,20,30],degrees=True)
        actual=Rotation.from_euler('ZYZ',target[3:],degrees=True)
        self.assertLess((actual*expected.inv()).magnitude(),1e-10)

    def test_exact_hard_boundaries_allowed_without_scaling(self):
        c,t=self.setup_path();raw=[.030,.040,0,math.radians(4),0,0,0]
        r=self.step(c,raw)
        self.assertFalse(c.aborted)
        self.assertEqual(r['filtered_action'],raw)
        np.testing.assert_allclose(t.calls[0][2]['pos'][:3],[430,40,500],atol=1e-10)

    def test_faults_no_motion_then_future_dispatch_inhibited(self):
        faults=[([.401,0,0,0,0,0,0],{}),([.401,.4,0,0,0,0,0],{}),
                ([0,0,0,math.radians(4.1),0,0,0],{}),([float('nan'),0,0,0,0,0,0],{}),
                ([float('inf'),0,0,0,0,0,0],{}),([.013,0,0,0,0,0,0],dict(camera_ok=False)),
                ([.013,0,0,0,0,0,0],dict(joint_state_ok=False)),
                ([.013,0,0,0,0,0,0],dict(model_ok=False)),
                ([.013,0,0,0,0,0,0],dict(communication_ok=False)),
                ([.020,0,0,0,0,0,0],dict(tcp_m_abc=[.79,0,.5,10,20,30]))]
        for raw,override in faults:
            with self.subTest(raw=raw,override=override):
                c,t=self.setup_path();self.step(c,raw,**override)
                self.assertTrue(c.aborted)
                self.assertFalse(any(name.endswith('move_line') for name,_,_ in t.calls))
                count=len(t.calls)
                with self.assertRaises(RuntimeError):self.step(c,[0]*7)
                self.assertEqual(len(t.calls),count)

    def test_full_policy_gripper_sequence_preserved(self):
        c,t=self.setup_path(full=True)
        with patch.object(c.sink.gripper_cancel,'wait',return_value=False):
            c.full_policy_gripper.initialize_open()
            c.full_policy_gripper.confirm_observed_open(observed_open=True,evidence_source='FAKE_TEST_ONLY')
            for _ in range(2):r=self.step(c,[.013,0,0,0,0,0,.9])
        self.assertFalse(c.aborted)
        self.assertEqual(r['logical_gripper_after'],'CLOSED')
        writes=[v for _,typ,v in t.calls if typ=='SetToolDigitalOutput']
        self.assertEqual([(v['index'],v['value']) for v in writes],
                         [(1,0),(2,0),(1,1),(1,0),(1,1),(1,0),(2,1),(2,0)])
        self.assertEqual(sum(typ=='MoveLine' for _,typ,_ in t.calls),2)

    def test_default_pipeline_still_limited_for_servo_legacy_callers(self):
        p=SafetyPipeline('openvla',operator_confirmed_initial_open=False,arm_only=True)
        a=CanonicalAction.from_vector([.013,0,0,0,0,0,0],timestamp_monotonic=1,sequence_id='x',source_model='openvla')
        d=p.inspect(a,RuntimeState(1,(.4,0,.5),(10,20,30),'alignment'))
        self.assertTrue(d.accepted)
        self.assertAlmostEqual(d.filtered_action[0],.0008)
        self.assertTrue(d.candidate['delta_limiter_applied'])

    def test_timing_gates_remain_in_full_endpoint_mode(self):
        def pipeline():return SafetyPipeline('openvla',operator_confirmed_initial_open=False,arm_only=True,full_endpoint_delta=True)
        def action(stamp=1,sequence='x'):
            return CanonicalAction.from_vector([.013,0,0,0,0,0,0],timestamp_monotonic=stamp,sequence_id=sequence,source_model='openvla')
        p=pipeline()
        self.assertTrue(p.inspect(action(),RuntimeState(1,(.4,0,.5),(10,20,30),'alignment')).accepted)
        self.assertIn('duplicate_action',p.inspect(action(),RuntimeState(1.3,(.4,0,.5),(10,20,30),'alignment')).reason)
        p=pipeline();p.inspect(action(),RuntimeState(1,(.4,0,.5),(10,20,30),'alignment'))
        self.assertIn('command_rate_limit',p.inspect(action(1.1,'y'),RuntimeState(1.1,(.4,0,.5),(10,20,30),'alignment')).reason)
        self.assertIn('stale_action',pipeline().inspect(action(),RuntimeState(2.3,(.4,0,.5),(10,20,30),'alignment')).reason)

    def test_scope_guard_does_not_enable_full_delta_for_legacy_or_oft(self):
        with self.assertRaises(ValueError):SafetyPipeline('oft',operator_confirmed_initial_open=False,full_endpoint_delta=True)
        with self.assertRaises(PermissionError):Controller('openvla','minimum_motion',None,Log(),moveline_full_delta=True)

    def test_above_50mm_accepted_unscaled_only_when_full_target_in_workspace(self):
        for raw in ([.0501,0,0,0,0,0,0],[.080,0,0,0,0,0,0],[.040,.040,0,0,0,0,0]):
            c,t=self.setup_path();r=self.step(c,raw)
            self.assertFalse(c.aborted)
            self.assertFalse(r['raw_translation_gate_enabled'])
            self.assertIsNone(r['effective_raw_translation_limit_m'])
            self.assertEqual(r['filtered_action'],raw)
            self.assertEqual(r['FINAL_DELIVERED_ARM_ACTION'],raw[:6])
            np.testing.assert_allclose(t.calls[0][2]['pos'][:3],np.array([400,0,500])+np.array(raw[:3])*1000,atol=1e-9)

    def test_50mm_gate_remains_available_for_unchanged_or_opt_in_paths(self):
        c,t=self.setup_path(raw_gate_enabled=True);self.step(c,[.0501,0,0,0,0,0,0])
        self.assertTrue(c.aborted)
        self.assertFalse(any(name.endswith('move_line') for name,_,_ in t.calls))
        p=SafetyPipeline('openvla',operator_confirmed_initial_open=False,arm_only=True)
        a=CanonicalAction.from_vector([.0501,0,0,0,0,0,0],timestamp_monotonic=1,sequence_id='z',source_model='openvla')
        self.assertIn('raw_translation_step_limit',p.inspect(a,RuntimeState(1,(.4,0,.5),(10,20,30),'alignment')).reason)

    def test_disabled_gate_cannot_leak_into_default_limited_mode(self):
        with self.assertRaises(PermissionError):
            SafetyPipeline('openvla',operator_confirmed_initial_open=False,enforce_raw_translation_limit=False)
        with self.assertRaises(PermissionError):
            Controller('openvla','full_task',None,Log(),moveline_raw_translation_gate_enabled=False)

    def test_large_or_nonfinite_values_still_never_dispatch(self):
        for value in (1e100,1e300,float('nan'),float('inf'),-float('inf')):
            with self.subTest(value=value):
                c,t=self.setup_path();self.step(c,[value,0,0,0,0,0,0])
                self.assertTrue(c.aborted)
                self.assertFalse(any(name.endswith('move_line') for name,_,_ in t.calls))
