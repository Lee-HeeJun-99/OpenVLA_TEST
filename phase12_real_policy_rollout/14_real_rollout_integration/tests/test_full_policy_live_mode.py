import json,time,unittest
from pathlib import Path
from tempfile import TemporaryDirectory
from unittest.mock import patch
from full_policy_live_mode import FullPolicyLiveMode
from real_sink import FakeRosTransport
from run_real_rollout import Controller

class Log:
    def __init__(self):self.rows=[]
    def append(self,row):self.rows.append(row)

class LiveModeTests(unittest.TestCase):
    def approval(self):
        return dict(execution_mode='FULL_POLICY_ROLLOUT',operator_present=True,workspace_clear=True,
            estop_accessible=True,robot_stationary=True,tcp_engineering_acceptance=True,
            gripper_mapping_confirmed=True,explicit_one_episode=True,confirmed_wall_time=time.time())
    def controller(self,mode):
        t=FakeRosTransport();log=Log()
        s=mode.make_sink(lambda:t,self.approval())
        c=Controller('openvla','full_task',s,log,dry_run=False,execution_mode='FULL_POLICY_ROLLOUT',
            gripper_contract=dict(abort_behavior='BOTH_OUTPUTS_OFF',operator_approved=True))
        return c,t
    def test_constructor_and_sink_create_no_transport(self):
        mode=FullPolicyLiveMode('unused.json');c,t=self.controller(mode)
        self.assertIsNone(c.sink.transport);self.assertEqual(t.calls,[])
    def test_arm_only_or_stale_approval_rejected(self):
        m=FullPolicyLiveMode('unused.json');a=self.approval();a['execution_mode']='ARM_ONLY_FULL_EPISODE'
        with self.assertRaises(PermissionError):m.validate(a)
        a=self.approval();a['confirmed_wall_time']-=61
        with self.assertRaises(PermissionError):m.validate(a)
    def test_initialization_ack_and_fresh_operator_confirmation(self):
        with TemporaryDirectory() as d:
            path=Path(d)/'open.json';m=FullPolicyLiveMode(path);c,t=self.controller(m)
            def save(name,record):
                if record.get('observed_open') is None:
                    path.write_text(json.dumps(dict(observed_open=True,operator='FAKE_TEST',confirmed_wall_time=time.time())))
            with patch.object(c.sink.gripper_cancel,'wait',return_value=False):m.initialize(c,lambda:None,save)
            self.assertTrue(c.full_policy_gripper.initialized)
            self.assertEqual(c.full_policy_gripper.state,'OPEN')
            self.assertEqual([(v['index'],v['value']) for _,typ,v in t.calls if typ=='SetToolDigitalOutput'],
                [(1,0),(2,0),(1,1),(1,0),(1,1),(1,0)])
    def test_existing_old_OPEN_artifact_cannot_authorize_episode(self):
        with TemporaryDirectory() as d:
            p=Path(d)/'open.json';p.write_text(json.dumps(dict(observed_open=True,operator='FAKE',confirmed_wall_time=1)))
            m=FullPolicyLiveMode(p);c,t=self.controller(m)
            with patch.object(c.sink.gripper_cancel,'wait',return_value=False),patch('full_policy_live_mode.monotonic',side_effect=[0,61]):
                with self.assertRaises(TimeoutError):m.initialize(c,lambda:None,lambda *a:None)
            self.assertFalse(c.full_policy_gripper.initialized)
            self.assertFalse(any(n.endswith('move_line') for n,_,_ in t.calls))
    def test_arm_feedback_hook_before_gripper_and_no_duplicate(self):
        m=FullPolicyLiveMode('unused.json');c,t=self.controller(m)
        with patch.object(c.sink.gripper_cancel,'wait',return_value=False):c.full_policy_gripper.initialize_open()
        c.full_policy_gripper.confirm_observed_open(observed_open=True,evidence_source='FAKE')
        hooked=[]
        c.before_gripper_dispatch=lambda:hooked.append(len(t.calls))
        for i in range(2):
            now=1+i*.3
            obs=dict(receive_monotonic=now,tcp_m_abc=[.4,0,.5,10,20,30],phase='alignment',robot_state_ok=True,
                camera_ok=True,joint_state_ok=True,tcp_ok=True,model_ok=True,communication_ok=True)
            with patch.object(c.sink.gripper_cancel,'wait',return_value=False):
                r=c.step([.001,0,0,0,0,0,.9],obs,inference_time=now,chunk_id=str(i),chunk_index=0)
            self.assertEqual(r['logical_gripper_after'],'CLOSED')
        self.assertEqual(hooked,[7,10]) # 6 init writes, arm, CLOSE pair, second arm
        self.assertEqual(sum(typ=='SetToolDigitalOutput' for _,typ,_ in t.calls),8)
    def test_pre_gripper_feedback_fault_stops_and_OFF_cleans(self):
        m=FullPolicyLiveMode('unused.json');c,t=self.controller(m)
        with patch.object(c.sink.gripper_cancel,'wait',return_value=False):c.full_policy_gripper.initialize_open()
        c.full_policy_gripper.confirm_observed_open(observed_open=True,evidence_source='FAKE')
        def fail():raise RuntimeError('settle_timeout')
        c.before_gripper_dispatch=fail
        obs=dict(receive_monotonic=1,tcp_m_abc=[.4,0,.5,10,20,30],phase='alignment',robot_state_ok=True,
            camera_ok=True,joint_state_ok=True,tcp_ok=True,model_ok=True,communication_ok=True)
        r=c.step([0,0,0,0,0,0,.9],obs,inference_time=1,chunk_id='1',chunk_index=0)
        self.assertTrue(c.aborted);self.assertEqual(r['gripper_abort']['state'],'completed')
        self.assertFalse(any(typ=='SetToolDigitalOutput' and v==dict(index=2,value=1) for _,typ,v in t.calls))
    def test_task_outcome_never_converts_INVALID(self):
        m=FullPolicyLiveMode('unused.json')
        s=dict(status='INVALID',reason='jointstate_gap',runtime_valid=False,task_success=None)
        self.assertIs(m.finalize_outcome(s,dict(task_success=True,operator='FAKE')),s)
    def test_explicit_fresh_OPEN_resume_no_duplicate_IO(self):
        with TemporaryDirectory() as d:
            p=Path(d)
            init=p/'gripper_initialization.json';init.write_text(json.dumps(dict(receipt=dict(state='completed'))))
            (p/'trial_summary.json').write_text(json.dumps(dict(reason='actual_OPEN_confirmation_timeout',arm_commands=0,episode_ended_wall_time=time.time()-1)))
            (p/'gripper_abort_result.json').write_text(json.dumps(dict(state='completed',attempts=[dict(readback=[dict(success=True,value=0),dict(success=True,value=0)])])))
            confirmation=p/'open.json';confirmation.write_text(json.dumps(dict(observed_open=True,operator='FAKE',confirmed_wall_time=time.time())))
            m=FullPolicyLiveMode(confirmation,confirmed_initialization=init);c,t=self.controller(m)
            m.initialize(c,lambda:None,lambda *a:None)
            self.assertTrue(c.full_policy_gripper.initialized);self.assertEqual(t.calls,[])
            bad=FullPolicyLiveMode(confirmation,confirmed_initialization=init);c2,t2=self.controller(bad)
            (p/'gripper_abort_result.json').write_text(json.dumps(dict(state='failed')))
            with self.assertRaises(PermissionError):bad.initialize(c2,lambda:None,lambda *a:None)
            self.assertFalse(c2.full_policy_gripper.initialized);self.assertEqual(t2.calls,[])
    def test_task_outcome_requires_valid_runtime_and_post_episode_input(self):
        m=FullPolicyLiveMode('unused.json');now=time.time()
        s=dict(status='OPERATOR_RESULT_CONFIRMATION_PENDING',reason=None,runtime_valid=True,
            arm_commands=1,episode_ended_wall_time=now-1)
        for success,status in ((True,'SUCCESS'),(False,'FAILURE')):
            r=m.finalize_outcome(s,dict(task_success=success,operator='FAKE',confirmed_wall_time=now))
            self.assertEqual(r['status'],status)
        with self.assertRaises(ValueError):m.finalize_outcome(s,dict(task_success=True,operator='FAKE',confirmed_wall_time=1))

if __name__=='__main__':unittest.main()
