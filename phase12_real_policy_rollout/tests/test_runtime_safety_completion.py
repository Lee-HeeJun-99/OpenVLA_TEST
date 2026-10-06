import json, tempfile, unittest
from collections import namedtuple
from pathlib import Path
import sys

ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT/'02_safety'),str(ROOT/'03_shadow_mode')]
from runtime_safety_supervisor import RuntimeSafetySupervisor
from integrated_logger import FsyncJsonlLogger,LoggerFailure


def supervisor():
    return RuntimeSafetySupervisor(
        max_action_age_sec=.3,min_command_period_sec=.19,
        max_translation_m=.01,max_rotation_rad=.02,
        max_translation_velocity_m_s=.1,max_rotation_velocity_rad_s=.2,
        max_translation_acceleration_m_s2=1.,max_rotation_acceleration_rad_s2=2.)


class RuntimeSafetyCompletionTests(unittest.TestCase):
    def test_operator_selected_20_profile_boundaries(self):
        s=RuntimeSafetySupervisor(max_action_age_sec=1.2,min_command_period_sec=.19,
            max_translation_m=.004,max_rotation_rad=0.06981317007977318,
            max_translation_velocity_m_s=.02,max_rotation_velocity_rad_s=.3490658503988659,
            max_translation_acceleration_m_s2=.02,max_rotation_acceleration_rad_s2=.3490658503988659)
        self.assertTrue(s.inspect(action_id='at_translation_step_limit',action=[.004,0,0,0,0,0,0],source_monotonic=1,now_monotonic=1).accepted)
        self.assertEqual('translation_step_limit',RuntimeSafetySupervisor(max_action_age_sec=1.2,min_command_period_sec=.19,max_translation_m=.004,max_rotation_rad=.06981317007977318,max_translation_velocity_m_s=.02,max_rotation_velocity_rad_s=.3490658503988659,max_translation_acceleration_m_s2=.02,max_rotation_acceleration_rad_s2=.3490658503988659).inspect(action_id='over',action=[.004001,0,0,0,0,0,0],source_monotonic=1,now_monotonic=1).hold_reason)
        self.assertEqual('rotation_step_limit',RuntimeSafetySupervisor(max_action_age_sec=1.2,min_command_period_sec=.19,max_translation_m=.004,max_rotation_rad=.06981317007977318,max_translation_velocity_m_s=.02,max_rotation_velocity_rad_s=.3490658503988659,max_translation_acceleration_m_s2=.02,max_rotation_acceleration_rad_s2=.3490658503988659).inspect(action_id='over_rot',action=[0,0,0,.069814,0,0,0],source_monotonic=1,now_monotonic=1).hold_reason)

    def test_watchdog_and_null_command_contract(self):
        s=supervisor()
        result=s.watchdog(last_camera_monotonic=1,last_state_monotonic=1,last_tcp_monotonic=1,now_monotonic=1.1,timeout_sec=.2)
        self.assertTrue(result.accepted);self.assertFalse(result.command_issued)
        result=s.watchdog(last_camera_monotonic=1,last_state_monotonic=1,last_tcp_monotonic=1,now_monotonic=1.3,timeout_sec=.2)
        self.assertEqual('camera_watchdog_timeout',result.hold_reason);self.assertFalse(result.command_issued)

    def test_stale_duplicate_nan_and_failure_paths(self):
        s=supervisor();a=[.001,0,0,0,0,0,0]
        self.assertEqual('stale_action',s.inspect(action_id='a',action=a,source_monotonic=1,now_monotonic=2).hold_reason)
        self.assertTrue(s.inspect(action_id='a',action=a,source_monotonic=2,now_monotonic=2).accepted)
        self.assertEqual('duplicate_action',s.inspect(action_id='a',action=a,source_monotonic=2.2,now_monotonic=2.2).hold_reason)
        self.assertEqual('invalid_action',s.inspect(action_id='b',action=[float('nan')]+[0]*6,source_monotonic=2.2,now_monotonic=2.2).hold_reason)
        for flag,reason in [('logger_ok','logger_failure'),('communication_ok','communication_failure'),('camera_ok','camera_failure'),('state_ok','joint_state_failure'),('tcp_ok','tcp_failure'),('inference_ok','inference_timeout')]:
            kwargs={k:True for k in ('logger_ok','communication_ok','camera_ok','state_ok','tcp_ok','inference_ok')};kwargs[flag]=False
            self.assertEqual(reason,supervisor().inspect(action_id='x',action=[0]*7,source_monotonic=1,now_monotonic=1,**kwargs).hold_reason)

    def test_step_rate_velocity_acceleration_limits(self):
        self.assertEqual('translation_step_limit',supervisor().inspect(action_id='a',action=[.02,0,0,0,0,0,0],source_monotonic=1,now_monotonic=1).hold_reason)
        s=supervisor();self.assertTrue(s.inspect(action_id='a',action=[.001,0,0,0,0,0,0],source_monotonic=1,now_monotonic=1).accepted)
        self.assertEqual('command_rate_limit',s.inspect(action_id='b',action=[.001,0,0,0,0,0,0],source_monotonic=1.1,now_monotonic=1.1).hold_reason)
        s=supervisor();s.max_translation_velocity_m_s=.02;s.inspect(action_id='a',action=[.001,0,0,0,0,0,0],source_monotonic=1,now_monotonic=1)
        self.assertEqual('translation_velocity_limit',s.inspect(action_id='b',action=[.005,0,0,0,0,0,0],source_monotonic=1.2,now_monotonic=1.2).hold_reason)
        s=supervisor();s.max_translation_acceleration_m_s2=.1;s.inspect(action_id='a',action=[0,0,0,0,0,0,0],source_monotonic=1,now_monotonic=1)
        self.assertTrue(s.inspect(action_id='b',action=[.001,0,0,0,0,0,0],source_monotonic=1.2,now_monotonic=1.2).accepted)
        self.assertEqual('translation_acceleration_limit',s.inspect(action_id='c',action=[.01,0,0,0,0,0,0],source_monotonic=1.4,now_monotonic=1.4).hold_reason)

    def test_atomic_logger_complete_partial_and_disk_full(self):
        Usage=namedtuple('Usage','total used free')
        with tempfile.TemporaryDirectory() as directory:
            path=Path(directory)/'records.jsonl'
            with FsyncJsonlLogger(path,minimum_free_bytes=1) as logger:logger.append({'executed_action':None,'robot_delivered_command':None})
            status=json.loads(path.with_suffix('.jsonl.status.json').read_text())
            row=json.loads(path.read_text())
            self.assertEqual('COMPLETE',status['state']);self.assertEqual(0,row['sequence_number']);self.assertFalse(path.with_suffix('.jsonl.partial').exists())
            with self.assertRaises(LoggerFailure):
                with FsyncJsonlLogger(Path(directory)/'full.jsonl',minimum_free_bytes=2,disk_usage_fn=lambda _:Usage(10,9,1)):pass

    def test_no_command_capabilities(self):
        source=(ROOT/'02_safety/runtime_safety_supervisor.py').read_text()
        for forbidden in ('rclpy','create_publisher','create_client','call_async','movej','movel','servol','speedl'):
            self.assertNotIn(forbidden,source.lower())


if __name__=='__main__':unittest.main()
