"""Inspect subprocess arguments with a fake child; never start ROS."""
import contextlib
import io
import json
from pathlib import Path
import tempfile
import unittest
import time
from unittest.mock import patch
import start_full_policy_confirmed as wrapper


class WrapperTests(unittest.TestCase):
    def test_routes_and_isolates_child_signal_group(self):
        for controller in ('moveline','servo'):
            with tempfile.TemporaryDirectory() as temp:
                root=Path(temp);(root/'configs').mkdir()
                (root/'configs/full_task.yaml').write_text('maximum_duration_s: 500\nmaximum_actions: 150\n')
                (root/'configs/servo_episode.yaml').write_text('maximum_predictions: 2000\n')
                (root/'configs/hardware_interface.yaml').write_text('velocity: [120, 20]\nacceleration: [120, 20]\n')
                class Child:
                    returncode=0
                    def poll(self):return 0
                with patch.object(wrapper,'__file__',str(root/'start.py')), \
                     patch('sys.argv',['start','--controller',controller,'--interactive']), \
                     patch('builtins.input',return_value='yes'), \
                     patch.object(wrapper.subprocess,'Popen',return_value=Child()) as popen, \
                     contextlib.redirect_stdout(io.StringIO()):
                    self.assertEqual(wrapper.main(),0)
                args,kwargs=popen.call_args
                command=args[0]
                self.assertTrue(kwargs['start_new_session'])
                self.assertEqual(kwargs['env']['ROS_LOCALHOST_ONLY'],'1')
                self.assertEqual('--full-policy' in command,controller=='servo')
                self.assertEqual(Path(command[2]).name,
                    'start_servo_episode_confirmed.py' if controller=='servo' else 'run_live_full_policy.py')
                evidence=json.loads(Path(command[command.index('--approval')+1]).read_text())
                self.assertEqual(evidence['physical_controller'],controller)
                if controller=='servo':self.assertTrue(evidence['concurrent_arm_gripper_accepted'])

    def test_denied_approval_starts_nothing(self):
        with patch('sys.argv',['start','--interactive']),patch('builtins.input',return_value='no'), \
             patch.object(wrapper.subprocess,'Popen') as popen,contextlib.redirect_stdout(io.StringIO()):
            self.assertEqual(wrapper.main(),1)
        popen.assert_not_called()

    def fake_config(self,root):
        (root/'configs').mkdir()
        (root/'configs/full_task.yaml').write_text('maximum_duration_s: 500\nmaximum_actions: 150\n')
        (root/'configs/servo_episode.yaml').write_text('maximum_predictions: 2000\n')
        (root/'configs/hardware_interface.yaml').write_text('velocity: [120, 20]\nacceleration: [120, 20]\nmoveline_delta_limiter_enabled: false\nmoveline_raw_translation_gate_enabled: false\n')

    def fake_approval(self,controller):
        return dict(execution_mode='FULL_POLICY_ROLLOUT',physical_controller=controller,
            operator='FAKE_UNIT_TEST_ONLY',confirmed_wall_time=time.time(),
            **dict.fromkeys(('operator_present','workspace_clear','estop_accessible','robot_stationary',
                'tcp_engineering_acceptance','gripper_mapping_confirmed','explicit_one_episode',
                'servo_risks_accepted','concurrent_arm_gripper_accepted','authority_and_stops_confirmed',
                'no_other_motion_executor'),True))

    def test_default_direct_moveline_needs_no_input_or_external_files(self):
        with tempfile.TemporaryDirectory() as temp:
            root=Path(temp);self.fake_config(root)
            class Child:
                returncode=0
                def poll(self):return 0
            with patch.object(wrapper,'__file__',str(root/'start.py')), \
                 patch('sys.argv',['start','--controller','moveline']),patch('builtins.input') as prompt, \
                 patch.object(wrapper.subprocess,'Popen',return_value=Child()) as popen, \
                 contextlib.redirect_stdout(io.StringIO()):
                self.assertEqual(wrapper.main(),0)
            prompt.assert_not_called();popen.assert_called_once()
            command=popen.call_args.args[0]
            self.assertIn('--direct',command);self.assertNotIn('--open-confirmation',command)
            request=json.loads(Path(command[command.index('--approval')+1]).read_text())
            self.assertFalse(request['manual_confirmation_performed'])
            self.assertIsNone(request['operator_present']);self.assertIsNone(request['workspace_clear'])
            self.assertIsNone(request['gripper_mapping_confirmed'])
            self.assertIsNone(request['confirmed_wall_time'])

    def test_servo_has_not_silently_switched_to_direct_mode(self):
        with patch('sys.argv',['start','--controller','servo']),patch('builtins.input') as prompt, \
             patch.object(wrapper.subprocess,'Popen') as popen,contextlib.redirect_stderr(io.StringIO()):
            with self.assertRaises(SystemExit):wrapper.main()
        prompt.assert_not_called();popen.assert_not_called()

    def test_file_mode_no_prompt_and_never_auto_confirms_OPEN(self):
        for controller in ('moveline','servo'):
            with tempfile.TemporaryDirectory() as temp:
                root=Path(temp);self.fake_config(root)
                approval=root/'operator.json';data=self.fake_approval(controller)
                approval.write_text(json.dumps(data))
                confirmation=root/'actual_open.json'
                old=dict(observed_open=False,operator='FAKE_OLD',confirmed_wall_time=1)
                confirmation.write_text(json.dumps(old))
                class Child:
                    returncode=0
                    def __init__(self):self.polls=0
                    def poll(self):
                        self.polls+=1
                        return None if self.polls==1 else 0
                def child_factory(command,**kwargs):
                    output=Path(command[command.index('--output')+1]);output.mkdir()
                    (output/'gripper_initialization.json').write_text(json.dumps(dict(ack_wall_time=time.time())))
                    return Child()
                with patch.object(wrapper,'__file__',str(root/'start.py')), \
                     patch('sys.argv',['start','--controller',controller,'--approval',str(approval),'--open-confirmation',str(confirmation)]), \
                     patch('builtins.input') as prompt, \
                     patch.object(wrapper.subprocess,'Popen',side_effect=child_factory) as popen, \
                     patch.object(wrapper.time,'sleep'),contextlib.redirect_stdout(io.StringIO()):
                    self.assertEqual(wrapper.main(),0)
                prompt.assert_not_called()
                self.assertEqual(json.loads(confirmation.read_text()),old)
                self.assertEqual(json.loads(approval.read_text()),data)
                command=popen.call_args.args[0]
                copy=json.loads(Path(command[command.index('--approval')+1]).read_text())
                self.assertEqual(copy['confirmed_wall_time'],data['confirmed_wall_time'])
                self.assertEqual(command[command.index('--open-confirmation')+1],str(confirmation.resolve()))
                self.assertTrue(popen.call_args.kwargs['start_new_session'])

    def test_file_mode_invalid_or_stale_evidence_never_starts(self):
        for override in (dict(operator_present=False),dict(workspace_clear='true'),dict(operator=''),
                         dict(confirmed_wall_time=1),dict(confirmed_wall_time=time.time()+3600),
                         dict(confirmed_wall_time=float('nan')),dict(confirmed_wall_time=True),
                         dict(physical_controller='servo')):
            with self.subTest(override=override),tempfile.TemporaryDirectory() as temp:
                root=Path(temp);self.fake_config(root);approval=root/'approval.json'
                data=dict(self.fake_approval('moveline'),**override);approval.write_text(json.dumps(data))
                with patch.object(wrapper,'__file__',str(root/'start.py')), \
                     patch('sys.argv',['start','--approval',str(approval),'--open-confirmation',str(root/'open.json')]), \
                     patch('builtins.input') as prompt,patch.object(wrapper.subprocess,'Popen') as popen, \
                     contextlib.redirect_stdout(io.StringIO()):
                    self.assertEqual(wrapper.main(),1)
                prompt.assert_not_called();popen.assert_not_called()
                self.assertFalse((root/'real_trials').exists())
