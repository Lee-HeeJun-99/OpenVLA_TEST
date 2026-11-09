import ast
from pathlib import Path
import threading
import unittest
from unittest.mock import patch

from servo_experimental.live_guard import StreamGuard, require_stream_liveness
from start_servo_episode_confirmed import episode_limits, start_error, TARGET
import math
import yaml
from run_real_rollout import SafetyPipeline, RuntimeState, CanonicalAction
from servo_experimental.controller import ServoShadowController


class LiveServoTests(unittest.TestCase):
    def test_no_command_no_stop(self):
        calls=[]; g=StreamGuard(lambda:calls.append('stop'))
        g.finish('preflight_failed'); g.finish('again')
        self.assertEqual(calls,[])
        with self.assertRaises(RuntimeError): g.publish(lambda:calls.append('publish'))

    def test_normal_exit_stop_once_inhibits(self):
        calls=[]; g=StreamGuard(lambda:calls.append('stop'))
        g.publish(lambda:calls.append('publish')); g.finish(); g.finish()
        self.assertEqual(calls,['publish','stop'])
        self.assertEqual((g.attempts,g.submitted),(1,1))
        with self.assertRaises(RuntimeError): g.publish(lambda:calls.append('publish'))

    def test_all_faults_inhibit_and_preserve_first(self):
        for reason in ['jointstate','camera_stream_loss','model_failure','raw_translation_step_limit',
                       'raw_rotation_step_limit','workspace_rejection','logger_failure',
                       'robot_abnormal_state','operator_abort','stale_action','servo_stream_timeout']:
            with self.subTest(reason=reason):
                calls=[]; g=StreamGuard(lambda:calls.append('stop'))
                g.publish(lambda:calls.append('publish')); g.finish(reason); g.finish('later')
                with self.assertRaises(RuntimeError): g.publish(lambda:calls.append('bad'))
                self.assertEqual(g.reason,reason); self.assertEqual(calls,['publish','stop'])

    def test_submission_exception_delivery_unknown_still_stop(self):
        calls=[]; g=StreamGuard(lambda:calls.append('stop'))
        def bad(): raise RuntimeError('unknown_delivery')
        with self.assertRaises(RuntimeError):g.publish(bad)
        g.finish('send_failed')
        self.assertEqual(calls,['stop']); self.assertEqual((g.attempts,g.submitted),(1,0))

    def test_concurrent_abort_idempotent(self):
        calls=[]; g=StreamGuard(lambda:calls.append('stop')); g.publish(lambda:None)
        workers=[threading.Thread(target=lambda:g.finish('fault')) for _ in range(30)]
        for w in workers:w.start()
        for w in workers:w.join()
        self.assertEqual(calls,['stop'])

    def test_stream_timeout_threshold_and_clock_regression(self):
        require_stream_liveness(1.099,1,active=True)
        for now in [1.101,.9]:
            with self.assertRaisesRegex(RuntimeError,'timeout'):require_stream_liveness(now,1,active=True)
        require_stream_liveness(100,1,active=False)
        require_stream_liveness(100,None,active=True)

    def test_episode_cap_cannot_increase(self):
        count, seconds=episode_limits()
        self.assertGreater(count,0); self.assertGreater(seconds,0)
        self.assertEqual(episode_limits(seconds),(count,seconds))
        self.assertEqual(episode_limits(1),(count,1))
        for duration in [seconds+1,0,-1,float('inf'),float('nan')]:
            with self.assertRaises(ValueError):episode_limits(duration)

    def test_servo_budget_2000_keeps_time_and_other_mode_unchanged(self):
        self.assertEqual(episode_limits(),(2000,500.))
        config=yaml.safe_load((Path(__file__).resolve().parents[1]/'configs/full_task.yaml').read_text())
        self.assertEqual(config['maximum_actions'],150)

    def test_name_based_start_pose(self):
        order=[2,0,5,1,3,4]
        row=dict(names=[f'joint_{i+1}' for i in order],position=[math.radians(TARGET[i]) for i in order])
        self.assertLess(start_error(row),1e-10)

    def test_static_no_gripper_or_moveline_and_separate_timers(self):
        source=(Path(__file__).resolve().parents[1]/'start_servo_episode_confirmed.py').read_text()
        tree=ast.parse(source)
        imports=[a.name for n in ast.walk(tree) if isinstance(n,ast.ImportFrom) for a in n.names]
        for forbidden in ['MoveLine','MoveJoint','SetToolDigitalOutput','SetDigitalOutput']:
            self.assertNotIn(forbidden,imports)
        self.assertIn("name='servo-policy-producer'",source)
        self.assertIn("name='servo-watchdog'",source)
        self.assertIn("sample['t0'], time.monotonic()",source)
        self.assertIn('publisher.get_subscription_count()!=1',source)
        self.assertIn('stop_mode = 3',source)
        self.assertIn('current = pose_read()',source)
        self.assertIn("'operator_abort.request'",source)

    def test_real_pipeline_to_fake_stream_and_fault_inhibition(self):
        limits=yaml.safe_load((Path(__file__).resolve().parents[2]/'01_configs/safety_limits.yaml').read_text())
        pipeline=SafetyPipeline('openvla',operator_confirmed_initial_open=False,arm_only=True)
        action=CanonicalAction.from_vector([.01,0,0,0,0,0,1],timestamp_monotonic=0,
            sequence_id='test-servo',source_model='openvla')
        d=pipeline.inspect(action,RuntimeState(.4,(.4,0,.5),(179,-155,3),'alignment'))
        self.assertTrue(d.accepted)
        tracker=ServoShadowController([400,0,500,179,-155,3],limits)
        tracker.set_target(d.candidate['target_pose_candidate_mm_zyz_deg'],0,.4)
        messages=[]; stops=[]; g=StreamGuard(lambda:stops.append(True))
        for i in range(10):
            row=tracker.tick(.4+i*.05)
            g.publish(lambda:messages.append(row))
        self.assertEqual(len(messages),10)
        # Raw gripper=1 must not change the arm-only dispatch contract.
        self.assertTrue(all('gripper_command' not in row for row in messages))
        g.finish('camera_stream_loss')
        for _ in range(3):
            with self.assertRaises(RuntimeError):g.publish(lambda:messages.append('bad'))
        self.assertEqual(len(messages),10); self.assertEqual(stops,[True])

    def test_actual_translation_rejection_dispatches_nothing(self):
        limits=yaml.safe_load((Path(__file__).resolve().parents[2]/'01_configs/safety_limits.yaml').read_text())
        pipeline=SafetyPipeline('openvla',operator_confirmed_initial_open=False,arm_only=True)
        action=CanonicalAction.from_vector([limits['translation_step_m']+.001,0,0,0,0,0,1],
            timestamp_monotonic=0,sequence_id='reject-servo',source_model='openvla')
        d=pipeline.inspect(action,RuntimeState(.4,(.4,0,.5),(179,-155,3),'alignment'))
        self.assertFalse(d.accepted)
        g=StreamGuard(lambda:None); g.finish('safety_rejection')
        with self.assertRaises(RuntimeError):g.publish(lambda:self.fail('must not dispatch'))
        self.assertEqual(g.attempts,0)


if __name__=='__main__':unittest.main()
