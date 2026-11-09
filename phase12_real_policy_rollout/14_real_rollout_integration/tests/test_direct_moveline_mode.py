"""No ROS or physical command: prior contract + ACK-only initialization tests."""
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch
import yaml
from direct_moveline_mode import DirectMoveLineMode, MANUAL_FIELDS
from real_sink import FakeRosTransport
from run_real_rollout import Controller


class Log:
    def __init__(self):self.rows=[]
    def append(self,row):self.rows.append(row)


class DirectModeTests(unittest.TestCase):
    def setup_path(self):
        mode=DirectMoveLineMode();request=mode.launch_request('FAKE_TEST_ONLY')
        transport=FakeRosTransport();log=Log()
        sink=mode.make_sink(lambda:transport,request)
        c=Controller('openvla','full_task',sink,log,dry_run=False,execution_mode='FULL_POLICY_ROLLOUT',
            gripper_contract=dict(abort_behavior='BOTH_OUTPUTS_OFF',operator_approved=True,
                                  allow_command_ack_initialization=True))
        return mode,request,c,transport,log

    def test_request_is_not_fabricated_site_confirmation(self):
        mode=DirectMoveLineMode();r=mode.launch_request('FAKE_TEST_ONLY');mode.validate(r)
        self.assertFalse(r['manual_confirmation_performed'])
        self.assertTrue(all(r[k] is None for k in MANUAL_FIELDS))
        self.assertIsNone(r['confirmed_wall_time'])
        r['workspace_clear']=True
        with self.assertRaises(PermissionError):mode.validate(r)

    def test_init_no_prompt_and_no_observed_OPEN_claim(self):
        mode,request,c,t,log=self.setup_path();saved={}
        with patch('builtins.input') as prompt,patch.object(c.sink.gripper_cancel,'wait',return_value=False):
            mode.initialize(c,lambda:None,lambda name,row:saved.update({name:row}))
        prompt.assert_not_called()
        self.assertTrue(c.full_policy_gripper.initialized)
        self.assertEqual(c.full_policy_gripper.state,'OPEN')
        self.assertEqual(c.full_policy_gripper.initialization_basis,'COMMAND_ACK_ONLY')
        self.assertIsNone(saved['gripper_initialization.json']['observed_open'])
        self.assertIsNone(saved['gripper_initialization.json']['measured_gripper_state'])
        self.assertFalse(saved['manual_confirmation.json']['performed'])
        self.assertFalse(any(row.get('observed_open') is True for row in log.rows))
        self.assertEqual([(v['index'],v['value']) for _,typ,v in t.calls if typ=='SetToolDigitalOutput'],
                         [(1,0),(2,0),(1,1),(1,0),(1,1),(1,0)])

    def test_failed_ack_refuses_initialization(self):
        mode,r,c,t,log=self.setup_path();t.outcomes=[{'success':False}]
        with self.assertRaises(RuntimeError):mode.initialize(c,lambda:None,lambda *a:None)
        self.assertFalse(c.full_policy_gripper.initialized)
        self.assertFalse(any(typ=='MoveLine' for _,typ,_ in t.calls))

    def test_current_runtime_fault_is_not_bypassed(self):
        mode,r,c,t,log=self.setup_path()
        def fault():raise RuntimeError('jointstate_fault')
        with self.assertRaises(RuntimeError):mode.initialize(c,fault,lambda *a:None)
        self.assertEqual(t.calls,[])

    def test_ack_only_cannot_leak_into_old_observation_required_mode(self):
        mode,r,c,t,log=self.setup_path()
        with patch.object(c.sink.gripper_cancel,'wait',return_value=False):c.full_policy_gripper.initialize_open()
        c.full_policy_gripper.contract.pop('allow_command_ack_initialization')
        with self.assertRaises(PermissionError):c.full_policy_gripper.accept_commanded_open(policy_source='FAKE')
        self.assertFalse(c.full_policy_gripper.initialized)

    def test_config_change_requires_new_static_mapping_acceptance(self):
        mode=DirectMoveLineMode();request=mode.launch_request('FAKE_TEST_ONLY')
        with tempfile.TemporaryDirectory() as temp:
            p=Path(temp);(p/'gripper.yaml').write_text('DIFFERENT_CONFIG')
            m=DirectMoveLineMode(gripper_path=p/'gripper.yaml')
            with self.assertRaises(PermissionError):m.validate(request)
            config=yaml.safe_load(mode.policy_path.read_text());config['enabled']=False
            (p/'policy.yaml').write_text(yaml.safe_dump(config))
            with self.assertRaises(PermissionError):DirectMoveLineMode(policy_path=p/'policy.yaml').validate(request)
