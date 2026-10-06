import sys,time,threading,unittest
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from hardware_watchdog import HardwareWatchdog,REQUIRED
from oft_action_scheduler import ActionScheduler
from task_phase import TaskPhase
from model_health import validate_health
from real_sink import *

class BackgroundTests(unittest.TestCase):
    def test_watchdog_during_blocking_worker(self):
        release=threading.Event();fault=threading.Event();state={k:True for k in REQUIRED}
        scheduler=ActionScheduler(lambda x:(release.wait(1) or {}))
        scheduler.dispatch(0,0,time.monotonic()-1,None)
        watch=HardwareWatchdog(lambda:state,lambda _:fault.set(),.005).start()
        state['camera_ok']=False
        self.assertTrue(fault.wait(.2));release.set();watch.close();scheduler.close()
    def test_watchdog_inputs_fail_closed(self):
        for key in REQUIRED:
            state={k:True for k in REQUIRED};state[key]=False;fault=[]
            HardwareWatchdog(lambda:state,fault.append).check();self.assertIn(key,fault[0])
    def test_no_overlap_and_target_times(self):
        block=threading.Event();scheduler=ActionScheduler(lambda _: (block.wait(.5) and {}),clock=lambda:1.)
        row=scheduler.dispatch(5,0,0,None);self.assertEqual(row['target_dispatch_time'],1.)
        row=scheduler.dispatch(5,1,-.2,None);self.assertEqual(row['status'],'NO_OVERLAPPING_MOTION')
        block.set();scheduler.close()
    def test_phase_sequence(self):
        p=TaskPhase(dict(grasp_pose_m=[.4,0,.3],pregrasp_z_m=.4,grasp_xy_tolerance_m=.01,
            pregrasp_z_tolerance_m=.01,grasp_z_tolerance_m=.01,lift_height_threshold_m=.05,max_duration_s=30))
        self.assertEqual(p.update([.4,0,.4]),'DESCEND')
        self.assertEqual(p.update([.4,0,.3]),'GRASP_CLOSE')
        self.assertEqual(p.update([.4,0,.3],command_closed=True),'LIFT')
        self.assertEqual(p.update([.4,0,.4],command_closed=True),'COMPLETE')
        self.assertEqual(p.report()['physical_grasp_success'],'UNVERIFIED')
    def test_abort_pulse(self):
        cfg=dict(polarity_confirmed=True,abort_value=0,gripper_closed_output_index=2,
            gripper_closed_hardware_value=1,inactive_hardware_value=0,closed_pulse_count=1,closed_pulse_time_s=1)
        t=FakeRosTransport();s=RealDoosanCommandSink(lambda:t,Authorization(True,True,True,'MOTION_ENABLED'),gripper_config=cfg)
        result=[];thread=threading.Thread(target=lambda:result.append(s.send_gripper(True)));thread.start()
        deadline=time.monotonic()+.5
        while not t.calls and time.monotonic()<deadline:time.sleep(.005)
        s.cancel_gripper();thread.join(.5)
        self.assertEqual(result[0]['state'],'aborted');self.assertEqual(t.calls[-1][2]['value'],0)
    def test_abort_value_required(self):
        s=RealDoosanCommandSink(lambda:FakeRosTransport(),Authorization(True,True,True,'MOTION_ENABLED'),gripper_config={'polarity_confirmed':True})
        with self.assertRaises(PermissionError):s.send_gripper(True)
    def test_health_wrong_checkpoint(self):
        expected=dict(checkpoint='vision28560',variant='oftplus_h5_vision',action_dim=7,chunk_size=5,
            requires_proprio=False,center_crop=True,input_resolution=[224,224],color_order='RGB',
            dtype='uint8',normalization='training',instruction='Pick up the orange cube.')
        self.assertTrue(validate_health(expected,expected))
        with self.assertRaises(ValueError):validate_health({**expected,'checkpoint':'6000'},expected)

if __name__=='__main__':unittest.main()
