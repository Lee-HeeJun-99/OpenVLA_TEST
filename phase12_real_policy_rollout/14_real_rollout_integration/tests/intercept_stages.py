"""Process-level stage controller tests: intercepted requests, NEVER physical IO."""
import sys,tempfile,json,time
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from real_sink import Authorization,RealDoosanCommandSink,FakeRosTransport
from run_real_rollout import Controller
from concurrent_logger import ConcurrentLogger
from integrated_logger import FsyncJsonlLogger
from task_phase import TaskPhase
from stage_gate import save_stage,require_stage
import threading
from real_sink import AbortBoundary

class SerializedIntercept(FakeRosTransport):
    def __init__(self,outcomes=None):super().__init__(outcomes);self.requests=[]
    def send(self,name,type_name,values):
        import dsr_msgs2.srv as generated
        request=getattr(generated,type_name).Request()
        for key,value in values.items():setattr(request,key,value)
        self.requests.append(request)
        return super().send(name,type_name,values)

def exercise(model,protocol):
    transport=SerializedIntercept()
    gripper=dict(polarity_confirmed=True,abort_value=0,gripper_closed_output_index=2,
        gripper_closed_hardware_value=1,inactive_hardware_value=0,closed_pulse_count=1,closed_pulse_time_s=0)
    sink=RealDoosanCommandSink(lambda:transport,Authorization(True,True,True,'MOTION_ENABLED'),gripper_config=gripper)
    phase=TaskPhase(dict(grasp_pose_m=[.4,0,.3],pregrasp_z_m=.4,grasp_xy_tolerance_m=.005,
        pregrasp_z_tolerance_m=.005,grasp_z_tolerance_m=.005,lift_height_threshold_m=.05,max_duration_s=30))
    with tempfile.TemporaryDirectory() as d:
        with FsyncJsonlLogger(Path(d)/'log.jsonl',minimum_free_bytes=0) as logger:
            controller=Controller(model,protocol,sink,ConcurrentLogger(logger),dry_run=False,initial_open=True)
            count=1 if protocol=='minimum_motion' else 10 if protocol=='short_horizon' else 4
            for index in range(count):
                tcp=[.4,0,.5] if protocol!='full_task' else [[.4,0,.4],[.4,0,.3],[.4,0,.3],[.4,0,.4]][index]
                closed=controller.pipeline.runtime.gripper.state.value=='COMMAND_CLOSED'
                state=phase.update(tcp,command_closed=closed)
                if state=='COMPLETE':break
                mapped={'APPROACH':'alignment','DESCEND':'descent_to_grasp','GRASP_CLOSE':'grasp_close','LIFT':'lift'}[state]
                t=1+index*.21
                obs=dict(receive_monotonic=t,tcp_m_abc=tcp+[0.,0.,0.],phase=mapped,
                    robot_state_ok=True,camera_ok=True,joint_state_ok=True,tcp_ok=True,model_ok=True,communication_ok=True,
                    matched_pair_id='test_pair',condition='FAKE_TEST_ONLY',sim_observation_id=None,real_observation_id=None)
                vector=[.0005,0,0,0,0,0,0] if protocol=='minimum_motion' else [0]*7
                if protocol=='full_task' and state in ('GRASP_CLOSE','LIFT'):vector[6]=1
                k=index%5 if model=='oft' else 0
                controller.step(vector,obs,inference_time=t,chunk_id=f'chunk-{index//5 if model=="oft" else index}',chunk_index=k)
                assert not controller.aborted,controller.rows[-1]['safety_blockers']
            if protocol=='full_task':assert phase.phase=='COMPLETE',phase.report()
        poses=[values for name,typ,values in transport.calls if typ=='MoveLine']
        assert poses
        if protocol=='minimum_motion':
            assert len(poses)==1 and abs(poses[0]['pos'][0]-400.5)<1e-9
            assert poses[0]['pos'][3:]==[0.,0.,0.] and poses[0]['ref']==0
            assert len(transport.calls)==1
        artifact=save_stage(protocol,d,{'model':model},passed=True,dry_run=True,details={'command_events':sink.events})
        assert json.loads(artifact.read_text())['physical_success'] is None
    print(model,protocol,'INTERCEPT_PASS',len(poses),'pose requests; physical calls=0')

def failure(outcome):
    transport=SerializedIntercept([outcome]);sink=RealDoosanCommandSink(lambda:transport,Authorization(True,True,True,'MOTION_ENABLED'))
    with tempfile.TemporaryDirectory() as d:
        with FsyncJsonlLogger(Path(d)/'events.jsonl',minimum_free_bytes=0) as logger:
            c=Controller('openvla','minimum_motion',sink,ConcurrentLogger(logger),dry_run=False,initial_open=True)
            obs=dict(receive_monotonic=1.,tcp_m_abc=[.4,0,.5,0.,0.,0.],phase='alignment',robot_state_ok=True,
                camera_ok=True,joint_state_ok=True,tcp_ok=True,model_ok=True,communication_ok=True)
            c.step([.0005,0,0,0,0,0,0],obs,inference_time=1.,chunk_id='minimum',chunk_index=0)
            assert c.aborted
            assert sum(e['command_type']=='hold' for e in sink.events)==1
            try:c.step([0]*7,obs,inference_time=1.,chunk_id='late',chunk_index=0)
            except RuntimeError:pass
            else:raise AssertionError('command_after_abort')
        rows=[json.loads(line) for line in (Path(d)/'events.jsonl').read_text().splitlines()]
        assert any(r.get('event')=='ABORT' for r in rows)
    print('COMMAND_FAILURE_INTERCEPT_PASS',type(outcome).__name__)

def pulse_abort():
    transport=SerializedIntercept();cfg=dict(polarity_confirmed=True,abort_value=0,gripper_closed_output_index=2,
        gripper_closed_hardware_value=1,inactive_hardware_value=0,closed_pulse_count=1,closed_pulse_time_s=2)
    sink=RealDoosanCommandSink(lambda:transport,Authorization(True,True,True,'MOTION_ENABLED'),gripper_config=cfg)
    receipts=[];thread=threading.Thread(target=lambda:receipts.append(sink.send_gripper(True)));thread.start()
    deadline=time.monotonic()+1
    while not transport.calls and time.monotonic()<deadline:time.sleep(.005)
    boundary=AbortBoundary(sink);boundary.abort('fake_watchdog_fault');boundary.abort('duplicate_abort')
    thread.join(1);assert not thread.is_alive() and receipts[0]['state']=='aborted'
    assert sum(e['command_type']=='hold' for e in sink.events)==1
    assert transport.calls[-1][2]['value']==0
    print('GRIPPER_ABORT_INTERCEPT_PASS physical calls=0')

if __name__=='__main__':
    exercise('openvla','minimum_motion')
    for model in ('openvla','oft'):exercise(model,'short_horizon')
    exercise('openvla','full_task')
    failure(TimeoutError('fake_ack_timeout'));failure({'success':False})
    pulse_abort()
