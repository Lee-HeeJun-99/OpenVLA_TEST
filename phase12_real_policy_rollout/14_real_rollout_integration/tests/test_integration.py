import sys,tempfile,time,json,unittest
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT))
from real_sink import *
from pre_real_rollout_check import preflight,REQUIRED
from stage_gate import require_stage,save_stage
from run_real_rollout import Controller

class MemoryLog:
    def __init__(self):self.rows=[]
    def append(self,row):self.rows.append(row)

class IntegrationTests(unittest.TestCase):
    def test_each_gate_flag(self):
        for auth in (Authorization(),Authorization(True,False,True,'MOTION_ENABLED'),Authorization(True,True,False,'MOTION_ENABLED'),Authorization(True,True,True,'COMMAND_DISABLED')):
            transport=FakeRosTransport();sink=RealDoosanCommandSink(lambda:transport,auth)
            for fn in (lambda:sink.send_pose([400,0,500,0,0,0]),lambda:sink.send_gripper(False),sink.hold,sink.stop):
                with self.assertRaises(PermissionError):fn()
            self.assertEqual(transport.calls,[])

    def sink(self,outcomes=None):
        transport=FakeRosTransport(outcomes)
        return RealDoosanCommandSink(lambda:transport,Authorization(True,True,True,'MOTION_ENABLED')),transport

    def test_pose_ack(self):
        sink,t=self.sink();r=sink.send_pose([400,0,500,0,0,0]);self.assertEqual(r['state'],'completed')
        self.assertEqual(t.calls[0][:2],SERVICES['pose']);self.assertIsNotNone(r['ack_latency'])
    def test_timeout_hold(self):
        sink,t=self.sink([TimeoutError()]);r=sink.send_pose([400,0,500,0,0,0]);self.assertEqual(r['state'],'timeout')
        boundary=AbortBoundary(sink);boundary.abort('timeout');self.assertEqual(boundary.state,'ABORTED')
        self.assertEqual(t.calls[-1][2],{'stop_mode':3})
    def test_hold_failure_stop(self):
        sink,t=self.sink([{'success':False}]);b=AbortBoundary(sink);b.abort('manual_abort')
        self.assertEqual(t.calls[-1][2],{'stop_mode':0})
    def test_preflight_freshness(self):
        self.assertFalse(preflight({})['MOTION_READY'])
        e={k:True for k in REQUIRED};e['verified_wall_time']=time.time()
        self.assertTrue(preflight(e,dry_run=False)['MOTION_READY'])
        self.assertFalse(preflight(e,dry_run=True)['MOTION_READY'])
        e['verified_wall_time']-=120;self.assertFalse(preflight(e,dry_run=False)['MOTION_READY'])
    def test_stage(self):
        with tempfile.TemporaryDirectory() as d:
            save_stage('minimum_motion',d,{'model':'oft'},passed=True,dry_run=True,details={})
            with self.assertRaises(PermissionError):require_stage('short_horizon',d,{'model':'oft'})
    def test_dry_run_controller(self):
        sink,t=self.sink();log=MemoryLog();c=Controller('openvla','minimum_motion',sink,log,dry_run=True,initial_open=True)
        obs=dict(receive_monotonic=1.,tcp_m_abc=[.4,0,.5,0,0,0],phase='alignment',robot_state_ok=True,
            camera_ok=True,joint_state_ok=True,tcp_ok=True,model_ok=True,communication_ok=True)
        c.step([0,0,0,0,0,0,0],obs,inference_time=1.,chunk_id='a',chunk_index=0)
        self.assertEqual(t.calls,[]);self.assertFalse(log.rows[0]['command_issued'])
    def test_abort_conditions(self):
        for flag in ('camera_ok','joint_state_ok','tcp_ok','model_ok','robot_state_ok'):
            sink,t=self.sink();c=Controller('openvla','minimum_motion',sink,MemoryLog(),dry_run=True,initial_open=True)
            obs=dict(receive_monotonic=1.,tcp_m_abc=[.4,0,.5,0,0,0],phase='alignment',robot_state_ok=True,
                camera_ok=True,joint_state_ok=True,tcp_ok=True,model_ok=True,communication_ok=True);obs[flag]=False
            c.step([0,0,0,0,0,0,0],obs,inference_time=1.,chunk_id='a',chunk_index=0)
            self.assertTrue(c.aborted);self.assertEqual(t.calls,[])

if __name__=='__main__':unittest.main()
