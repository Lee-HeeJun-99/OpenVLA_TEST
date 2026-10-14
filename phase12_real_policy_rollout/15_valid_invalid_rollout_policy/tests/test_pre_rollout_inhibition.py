import sys,unittest
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1];sys.path[:0]=[str(ROOT),str(ROOT.parent/'14_real_rollout_integration')]
from real_sink import RealDoosanCommandSink,Authorization,FakeRosTransport
from run_real_rollout import Controller
from test_rollout_trial_classifier import trial

class Log:
    def append(self,row):pass

class InhibitionPreflightTests(unittest.TestCase):
    def test_all_faults_terminal_without_dispatch(self):
        for fault in ('jointstate','camera','model','translation','rotation','logger','nan'):
            with self.subTest(fault=fault):
                t=trial();transport=FakeRosTransport();sink=RealDoosanCommandSink(lambda:transport,Authorization(True,True,True,'MOTION_ENABLED'))
                class BrokenLog:
                    def append(self,row):raise IOError('injected_logger_write_failure')
                c=Controller('openvla','short_horizon',sink,BrokenLog() if fault=='logger' else Log(),dry_run=True,initial_open=True,trial=t)
                obs=dict(receive_monotonic=1,tcp_m_abc=[.4,0,.5,10,20,30],phase='alignment',robot_state_ok=True,camera_ok=True,joint_state_ok=True,tcp_ok=True,model_ok=True,communication_ok=True)
                action=[0]*7
                if fault=='jointstate':obs['joint_state_ok']=False
                if fault=='camera':obs['camera_ok']=False
                if fault=='model':obs['model_ok']=False
                if fault=='translation':action[0]=.0131
                if fault=='rotation':action[3]=.071
                if fault=='nan':action[0]=float('nan')
                try:c.step(action,obs,inference_time=1,chunk_id='a',chunk_index=0)
                except IOError:pass
                self.assertEqual(t.status,'INVALID');self.assertTrue(c.aborted)
                self.assertIsNone(t.summary()['task_success'])
                with self.assertRaises(RuntimeError):c.step([0]*7,obs,inference_time=1,chunk_id='b',chunk_index=0)
                self.assertEqual(transport.calls,[])
