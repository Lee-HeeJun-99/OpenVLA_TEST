import unittest
from run_real_rollout import Controller
from real_sink import RealDoosanCommandSink, Authorization, FakeRosTransport
from full_policy_gripper import FullPolicyGripper


class Log:
    def __init__(self): self.rows = []
    def append(self, row): self.rows.append(row)


class FullPolicyTests(unittest.TestCase):
    def setup_path(self):
        transport = FakeRosTransport()
        config = dict(polarity_confirmed=True, abort_value=0, abort_value_confirmed=True,
                      gripper_open_output_index=1, gripper_closed_output_index=2,
                      gripper_open_hardware_value=1, gripper_closed_hardware_value=1,
                      inactive_hardware_value=0, open_pulse_count=1, closed_pulse_count=1,
                      open_pulse_time_s=0, closed_pulse_time_s=0)
        sink = RealDoosanCommandSink(lambda: transport,
                Authorization(True, True, True, 'MOTION_ENABLED'), gripper_config=config)
        contract = dict(abort_behavior='OUTPUT_OFF', abort_behavior_confirmed=True,
                        abort_evidence_source='SYNTHETIC_TEST_ONLY_NOT_HARDWARE_EVIDENCE')
        controller = Controller('openvla', 'full_task', sink, Log(), dry_run=False,
                                execution_mode='FULL_POLICY_ROLLOUT', gripper_contract=contract)
        obs = dict(receive_monotonic=1, tcp_m_abc=[.4, 0, .5, 10, 20, 30],
                   phase='alignment', robot_state_ok=True, camera_ok=True,
                   joint_state_ok=True, tcp_ok=True, model_ok=True, communication_ok=True)
        return controller, transport, obs

    def initialize(self, controller):
        g = controller.full_policy_gripper
        g.initialize_open()
        g.confirm_observed_open(observed_open=True, evidence_source='FAKE_OPERATOR_TEST_ONLY')

    def step(self, c, obs, value, x=0):
        obs = dict(obs, receive_monotonic=1 + c.count * .3)
        return c.step([x, 0, 0, 0, 0, 0, value], obs,
                      inference_time=obs['receive_monotonic'], chunk_id=str(c.count), chunk_index=0)

    def test_unknown_abort_refuses_without_transport(self):
        sink = RealDoosanCommandSink(lambda: self.fail('must not create transport'))
        with self.assertRaises(PermissionError): FullPolicyGripper(sink, Log(), {})
        self.assertIsNone(sink.transport)

    def test_ack_is_not_initial_open_observation(self):
        c, t, obs = self.setup_path()
        c.full_policy_gripper.initialize_open()
        self.step(c, obs, .9)
        self.assertTrue(c.aborted)
        self.assertFalse(any(n.endswith('move_line') for n, _, _ in t.calls))

    def test_initialization_once_and_boundary_hysteresis(self):
        c, t, obs = self.setup_path(); self.initialize(c)
        g = c.full_policy_gripper
        self.assertEqual(g.desired(.3), 'OPEN')
        self.assertEqual(g.desired(.7), 'OPEN')
        with self.assertRaises(PermissionError): g.initialize_open()
        self.assertEqual(len(t.calls), 2)

    def test_transitions_only_and_no_phase_inference(self):
        c, t, obs = self.setup_path(); self.initialize(c)
        for value in (.2, .9, .9, .5, .2, .9): self.step(c, obs, value)
        self.assertFalse(c.aborted)
        self.assertEqual(sum(n.endswith('move_line') for n, _, _ in t.calls), 6)
        # initialization + CLOSE + OPEN + CLOSE; one ON/OFF per transition
        self.assertEqual(sum(n.endswith('set_tool_digital_output') for n, _, _ in t.calls), 8)

    def test_raw_gate_rejects_and_future_dispatch_inhibited(self):
        c, t, obs = self.setup_path(); self.initialize(c)
        self.step(c, obs, .9, .0131)
        self.assertTrue(c.aborted)
        before = len(t.calls)
        with self.assertRaises(RuntimeError): self.step(c, obs, .9)
        self.assertEqual(len(t.calls), before)
        self.assertFalse(any(n.endswith('move_line') for n, _, _ in t.calls))

    def test_nan_gripper_never_dispatches_arm(self):
        c, t, obs = self.setup_path(); self.initialize(c)
        self.step(c, obs, float('nan'))
        self.assertTrue(c.aborted)
        self.assertFalse(any(n.endswith('move_line') for n, _, _ in t.calls))

    def test_gripper_failure_does_not_commit_and_aborts(self):
        c, t, obs = self.setup_path(); self.initialize(c)
        t.outcomes = [{'success': True}, {'success': False}]
        self.step(c, obs, .9)
        self.assertTrue(c.aborted)
        self.assertEqual(c.full_policy_gripper.state, 'OPEN')
        before = len(t.calls)
        with self.assertRaises(RuntimeError): self.step(c, obs, .2)
        self.assertEqual(len(t.calls), before)

    def test_camera_fault_no_arm_or_policy_gripper(self):
        c, t, obs = self.setup_path(); self.initialize(c)
        obs['camera_ok'] = False
        self.step(c, obs, .9)
        self.assertTrue(c.aborted)
        self.assertEqual(sum(n.endswith('set_tool_digital_output') for n, _, _ in t.calls), 2)

    def test_fault_open_and_closed_double_abort_no_new_gripper(self):
        for closed in (False, True):
            with self.subTest(closed=closed):
                c, t, obs = self.setup_path(); self.initialize(c)
                if closed: self.step(c, obs, .9)
                before = len(t.calls)
                c.abort('operator_abort'); c.abort('operator_abort_again')
                self.assertEqual(len(t.calls), before + 1)  # exactly one arm HOLD
                self.assertTrue(t.calls[-1][0].endswith('move_stop'))
                with self.assertRaises(PermissionError): c.full_policy_gripper.transition(.2)
                with self.assertRaises(RuntimeError): self.step(c, obs, .2)
                self.assertEqual(len(t.calls), before + 1)

    def test_fault_classes_share_inhibition(self):
        for reason in ('model_failure', 'jointstate_gap', 'camera_stale',
                       'workspace_rejection', 'arm_ack_failure', 'logger_failure'):
            with self.subTest(reason=reason):
                c, t, obs = self.setup_path(); self.initialize(c)
                before = len(t.calls)
                c.abort(reason)
                with self.assertRaises(PermissionError): c.full_policy_gripper.transition(.9)
                self.assertEqual(len(t.calls), before + 1)

    def test_logger_fault_runs_arm_abort_even_when_log_broken(self):
        c, t, obs = self.setup_path(); self.initialize(c)
        def broken(_): raise OSError('synthetic_logger_fault')
        c.log.append = broken
        with self.assertRaises(OSError): self.step(c, obs, .9)
        self.assertTrue(c.aborted)
        self.assertFalse(any(n.endswith('move_line') for n, _, _ in t.calls))
        self.assertEqual(sum(n.endswith('move_stop') for n, _, _ in t.calls), 1)


if __name__ == '__main__': unittest.main()
