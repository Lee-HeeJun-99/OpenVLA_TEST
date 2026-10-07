import sys,threading,time,unittest
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1];sys.path[:0]=[str(ROOT),str(ROOT.parent/'14_real_rollout_integration')]
from rollout_trial_classifier import RolloutTrial
from real_sink import RealDoosanCommandSink,Authorization,FakeRosTransport
from run_real_rollout import Controller
from test_rollout_trial_classifier import trial

class Log:
    def append(self,r):pass

class InvalidIntegrationTests(unittest.TestCase):
    def test_controller_invalid_no_more_commands(self):
        t=trial();transport=FakeRosTransport();sink=RealDoosanCommandSink(lambda:transport,Authorization(True,True,True,'MOTION_ENABLED'))
        c=Controller('openvla','short_horizon',sink,Log(),dry_run=True,initial_open=True,trial=t)
        obs=dict(receive_monotonic=1,tcp_m_abc=[.4,0,.5,10,20,30],phase='alignment',robot_state_ok=True,camera_ok=False,joint_state_ok=True,tcp_ok=True,model_ok=True,communication_ok=True)
        c.step([0]*7,obs,inference_time=1,chunk_id='x',chunk_index=0)
        self.assertEqual(t.status,'INVALID');self.assertIsNotNone(t.invalid_event['last_prediction'])
        with self.assertRaises(RuntimeError):c.step([0]*7,obs,inference_time=1,chunk_id='y',chunk_index=0)
        self.assertEqual(transport.calls,[])
    def test_invalid_during_prepare_no_dispatch(self):
        ready=threading.Event();release=threading.Event()
        class Transport(FakeRosTransport):
            def prepare(self,*args):ready.set();release.wait(1);return args
            def dispatch_prepared(self,args):return self.send(*args)
        t=trial();transport=Transport();sink=RealDoosanCommandSink(lambda:transport,Authorization(True,True,True,'MOTION_ENABLED'));sink.trial_guard=t
        result=[];worker=threading.Thread(target=lambda:result.append(sink.send_pose([400,0,500,10,20,30])));worker.start()
        self.assertTrue(ready.wait(1));t.invalidate('JOINTSTATE_RECEIVE_GAP');sink.inhibit();release.set();worker.join(1)
        self.assertFalse(worker.is_alive());self.assertEqual(transport.calls,[]);self.assertEqual(t.command_count,0)
    def test_inflight_ack_does_not_block_invalid(self):
        ready=threading.Event();release=threading.Event()
        class Transport(FakeRosTransport):
            def wait(self,*args):ready.set();release.wait(1);return {'success':True}
        t=trial();transport=Transport();sink=RealDoosanCommandSink(lambda:transport,Authorization(True,True,True,'MOTION_ENABLED'));sink.trial_guard=t
        result=[];worker=threading.Thread(target=lambda:result.append(sink.send_pose([400,0,500,10,20,30])));worker.start()
        self.assertTrue(ready.wait(1));start=time.monotonic();t.invalidate('CAMERA_STALE');sink.inhibit();self.assertLess(time.monotonic()-start,.1)
        release.set();worker.join(1);self.assertFalse(worker.is_alive());self.assertEqual(result[0]['state'],'ABORTED_IN_FLIGHT');self.assertEqual(t.invalid_event['command_count_before_invalid'],1)

if __name__=='__main__':unittest.main()
