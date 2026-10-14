import unittest
import math
from run_real_rollout import Controller
from real_sink import Authorization, FakeRosTransport, RealDoosanCommandSink
from arm_only_episode_contract import ArmOnlyEpisodeSink, episode_limits, gripper_intent

class Log:
    def append(self, row): pass

class ArmEpisodeTests(unittest.TestCase):
    def setup_controller(self):
        t=FakeRosTransport()
        sink=ArmOnlyEpisodeSink(lambda:t,Authorization(True,True,True,'MOTION_ENABLED'))
        c=Controller('openvla','full_task',sink,Log(),dry_run=False,execution_mode='ARM_ONLY_FULL_EPISODE')
        return c,t
    def step(self,c,g=.99,camera_ok=True,translation=.001):
        now=1+c.count*.3
        obs=dict(receive_monotonic=now,tcp_m_abc=[.4,0,.5,10,20,30],phase='alignment',
            robot_state_ok=True,camera_ok=camera_ok,joint_state_ok=True,tcp_ok=True,model_ok=True,communication_ok=True)
        return c.step([translation,0,0,0,0,0,g],obs,inference_time=now,chunk_id=str(c.count),chunk_index=0)
    def test_multiple_steps_close_intent_zero_IO(self):
        c,t=self.setup_controller()
        for _ in range(3):self.step(c)
        self.assertFalse(c.aborted)
        self.assertEqual(len(t.calls),3)
        self.assertTrue(all(n.endswith('move_line') for n,_,_ in t.calls))
    def test_even_emergency_gripper_write_is_blocked(self):
        c,t=self.setup_controller()
        for kind in ('gripper','tool_output'):
            with self.assertRaises(PermissionError):c.sink._send(kind,dict(index=1,value=0),emergency=True)
        with self.assertRaises(PermissionError):c.sink.send_gripper(False)
        self.assertEqual(t.calls,[])
    def test_abort_has_only_arm_stop_and_no_future_dispatch(self):
        c,t=self.setup_controller();self.step(c)
        c.abort('model_failure');c.abort('again')
        self.assertEqual(len(t.calls),2)
        self.assertTrue(t.calls[-1][0].endswith('move_stop'))
        with self.assertRaises(RuntimeError):self.step(c)
        self.assertEqual(len(t.calls),2)
    def test_camera_and_raw_limit_still_reject(self):
        for kwargs in (dict(camera_ok=False),dict(translation=.0131)):
            c,t=self.setup_controller();self.step(c,**kwargs)
            self.assertTrue(c.aborted)
            self.assertFalse(any(n.endswith('move_line') for n,_,_ in t.calls))
    def test_normal_sink_cannot_enable_episode(self):
        with self.assertRaises(PermissionError):Controller('openvla','full_task',RealDoosanCommandSink(None),Log(),execution_mode='ARM_ONLY_FULL_EPISODE')
    def test_existing_limits_and_gripper_boundaries(self):
        self.assertEqual(episode_limits(),(150,30))
        self.assertEqual([gripper_intent(x) for x in (.2,.3,.7,.8)],['OPEN','HOLD','HOLD','CLOSED'])
    def test_overlapping_steps_refused(self):
        c,t=self.setup_controller();c.full_policy_step_lock.acquire()
        try:
            with self.assertRaises(PermissionError):self.step(c)
        finally:c.full_policy_step_lock.release()
        self.assertEqual(t.calls,[])
    def test_start_pose_uses_named_values_not_dictionary_keys(self):
        from run_arm_only_episode import start_pose_error_deg, TARGET
        names=[f'joint_{i}' for i in range(6,0,-1)]
        positions=[math.radians(TARGET[i-1]) for i in range(6,0,-1)]
        self.assertLess(start_pose_error_deg(names,positions),1e-10)
        with self.assertRaises(ValueError):start_pose_error_deg(names[:-1],positions[:-1])
    def test_duration_override_requires_exact_explicit_approval(self):
        from arm_only_episode_contract import approved_episode_limits
        self.assertEqual(approved_episode_limits({}),(150,30))
        self.assertEqual(approved_episode_limits(dict(explicit_one_episode=True,approved_max_duration_s=70),70),(150,70))
        for a,d in [({},70),({'explicit_one_episode':True,'approved_max_duration_s':70},71),({},float('inf'))]:
            with self.assertRaises(PermissionError):approved_episode_limits(a,d)

if __name__=='__main__':unittest.main()
