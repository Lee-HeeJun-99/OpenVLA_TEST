import json
import itertools
import io
import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace as NS
from unittest.mock import Mock,patch
from rollout_latency import warm_up,archive_request_jpeg,deferred_artifacts,predict_frame
from run_real_rollout import Controller
from real_sink import Authorization,FakeRosTransport
from arm_only_episode_contract import ArmOnlyEpisodeSink

class LatencyTests(unittest.TestCase):
    def sample(self,i,latency):
        return dict(msg=NS(header=NS(stamp=NS(sec=i,nanosec=0))),raw=[0]*7,
            timing=dict(inference_latency=latency,camera_age_at_inference_start=.01))
    def test_cold_request_discarded_two_warm_required(self):
        predict=Mock(side_effect=[self.sample(1,1.1),self.sample(2,.45),self.sample(3,.46)])
        saved=[];warm_up(predict,lambda:None,lambda n,x:saved.append(x),max_action_age_s=1.2,settle_s=.2)
        self.assertEqual(saved[-1]['status'],'PASS');self.assertEqual(predict.call_count,3)
        self.assertTrue(all(r['excluded_from_episode'] and not r['command_issued'] for r in saved[-1]['samples']))
    def test_slow_warm_request_still_blocks(self):
        p=Mock(side_effect=[self.sample(1,1.1),self.sample(2,1.01),self.sample(3,.4)])
        with self.assertRaisesRegex(RuntimeError,'insufficient'):
            warm_up(p,lambda:None,lambda *a:None,max_action_age_s=1.2,settle_s=.2)
    def test_warmup_fault_or_duplicate_never_proceeds(self):
        p=Mock(return_value=self.sample(1,.4))
        with self.assertRaisesRegex(RuntimeError,'duplicate'):
            warm_up(p,lambda:None,lambda *a:None,max_action_age_s=1.2,settle_s=.2)
        p.reset_mock()
        with self.assertRaisesRegex(RuntimeError,'sensor'):
            warm_up(p,Mock(side_effect=RuntimeError('sensor')),lambda *a:None,max_action_age_s=1.2,settle_s=.2)
        p.assert_not_called()
    def test_exact_request_bytes_durable_without_recompression(self):
        with tempfile.TemporaryDirectory() as d:
            p=Path(d)/'input.jpg';archive_request_jpeg(p,b'JPEG_ALREADY_ENCODED')
            self.assertEqual(p.read_bytes(),b'JPEG_ALREADY_ENCODED')
            with self.assertRaisesRegex(RuntimeError,'logger_failure'):archive_request_jpeg(p,b'x')
    def test_stale_camera_never_requests_model(self):
        m=NS(encoding='bgra8',width=1280,height=720)
        rgb=Mock();opener=Mock()
        with patch('live_observation.camera_rgb_image',return_value=rgb):
            with self.assertRaisesRegex(ValueError,'camera_stale'):
                predict_frame((m,1),lambda:100,lambda:None,clock=lambda:1.5,request_open=opener)
        opener.assert_not_called()
    def test_nonfinite_action_rejected_after_response(self):
        m=NS(encoding='bgra8',width=1280,height=720,header=NS(stamp=NS(sec=100,nanosec=0)))
        with patch('live_observation.camera_rgb_image',return_value=Mock()):
            with self.assertRaisesRegex(ValueError,'invalid_model_action'):
                predict_frame((m,1),lambda:100,lambda:None,clock=lambda:1.01,
                    request_open=Mock(return_value=io.BytesIO(json.dumps({'action':[float('inf')]*7}).encode())))
    def test_deferred_work_failure_not_silenced(self):
        with self.assertRaises(OSError):
            deferred_artifacts(Path('/tmp'),0,NS(data=b'x'),Mock(save=Mock(side_effect=OSError('disk'))),b'jpeg')
    def controller(self,log):
        t=FakeRosTransport();sink=ArmOnlyEpisodeSink(lambda:t,Authorization(True,True,True,'MOTION_ENABLED'))
        return Controller('openvla','full_task',sink,log,dry_run=False,execution_mode='ARM_ONLY_FULL_EPISODE'),t
    def step(self,c):
        return c.step([.001,0,0,0,0,0,.1],dict(receive_monotonic=10,live_execution_clock=True,
            tcp_m_abc=[.4,0,.5,0,0,0],phase='alignment',robot_state_ok=True,camera_ok=True,joint_state_ok=True,
            tcp_ok=True,model_ok=True,communication_ok=True),inference_time=10,chunk_id='x',chunk_index=0)
    def test_logger_stall_rechecked_before_transport(self):
        c,t=self.controller(Mock())
        with patch('run_real_rollout.time.monotonic',side_effect=itertools.chain([10],itertools.repeat(11.3))):self.step(c)
        self.assertTrue(c.aborted);self.assertEqual(c.abort_reason,'stale_action')
        self.assertFalse(any(n.endswith('move_line') for n,_,_ in t.calls))
        with self.assertRaises(RuntimeError):self.step(c)
    def test_original_timestamp_not_rebased_and_limit_unchanged(self):
        c,t=self.controller(Mock())
        with patch('run_real_rollout.time.monotonic',side_effect=itertools.chain([10.6],itertools.repeat(11.1))):self.step(c)
        self.assertFalse(c.aborted);self.assertEqual(c.pipeline.runtime.safety.max_action_age_sec,1.2)
        self.assertEqual(sum(n.endswith('move_line') for n,_,_ in t.calls),1)
    def test_service_discovery_stall_cannot_dispatch_expired_action(self):
        now=[10.6]
        class PreparedTransport(FakeRosTransport):
            def prepare(self,*args):
                if args[0].endswith('move_line'):now[0]=11.3
                return args
            def dispatch_prepared(self,args):return self.send(*args)
        t=PreparedTransport();sink=ArmOnlyEpisodeSink(lambda:t,Authorization(True,True,True,'MOTION_ENABLED'))
        c=Controller('openvla','full_task',sink,Mock(),dry_run=False,execution_mode='ARM_ONLY_FULL_EPISODE')
        with patch('run_real_rollout.time.monotonic',side_effect=lambda:now[0]):self.step(c)
        self.assertTrue(c.aborted);self.assertEqual(c.abort_reason,'stale_action')
        self.assertFalse(any(n.endswith('move_line') for n,_,_ in t.calls))
        self.assertTrue(any(n.endswith('move_stop') for n,_,_ in t.calls))

if __name__=='__main__':unittest.main()
