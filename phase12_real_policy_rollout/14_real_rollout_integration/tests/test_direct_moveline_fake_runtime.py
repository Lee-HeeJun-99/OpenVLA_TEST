"""Complete direct runner with fake ROS/FakeRosTransport, no real endpoints."""
import contextlib
import io
import json
import math
from pathlib import Path
import tempfile
import time
import types
import unittest
from unittest.mock import patch
import run_arm_only_episode as live
from direct_moveline_mode import DirectMoveLineMode
from real_sink import FakeRosTransport


class DirectRunnerTests(unittest.TestCase):
    def execute(self,bad_hardware=False):
        callbacks=[];transport=FakeRosTransport()
        class Request:pass
        services={name:type(name,(),{'Request':Request}) for name in
                  ('GetRobotSystem','GetRobotMode','GetRobotState','GetCurrentPosx')}
        class Client:
            def __init__(self,name):self.name=name
            def wait_for_service(self,**kw):return True
            def call_async(self,request):
                if self.name.endswith('get_robot_system'):row=dict(success=True,robot_system=0)
                elif self.name.endswith('get_robot_mode'):row=dict(success=True,robot_mode=1)
                elif self.name.endswith('get_robot_state'):row=dict(success=True,robot_state=3 if bad_hardware else 1)
                else:row=dict(success=True,task_pos_info=[dict(data=[305.,-11.,723.,179.,-155.,3.])])
                return types.SimpleNamespace(done=lambda:True,result=lambda:row)
        class Node:
            def create_client(self,kind,name):return Client(name)
            def destroy_client(self,client):pass
            def create_subscription(self,kind,name,cb,qos):callbacks.append((name,cb))
            def get_clock(self):return types.SimpleNamespace(now=lambda:types.SimpleNamespace(nanoseconds=time.time_ns()))
            def destroy_node(self):pass
        def spin_once(node,**kw):
            time.sleep(.004)
            for topic,callback in callbacks:
                now=time.time_ns();stamp=types.SimpleNamespace(sec=now//10**9,nanosec=now%10**9)
                fields=dict(header=types.SimpleNamespace(stamp=stamp))
                if 'joint_states' in topic:fields.update(name=[f'joint_{i}' for i in range(1,7)],
                    position=[math.radians(v) for v in live.TARGET],velocity=[0.]*6)
                else:fields.update(width=1280,height=720,encoding='bgra8')
                callback(types.SimpleNamespace(**fields))
        def predict(selected,ros_now,check):
            check();start=time.monotonic();time.sleep(.01);check();end=time.monotonic()
            raw=[.06,0,0,0,0,0,.9]
            return dict(msg=selected[0],rgb=None,jpeg=b'FAKE_TEST_JPEG',raw=raw,prediction=dict(action=raw),
                t0=start,t1=end,timing=dict(inference_start_monotonic=start,inference_end_monotonic=end,
                inference_latency=end-start,camera_age_at_inference_start=start-selected[1],camera_age_at_inference_end=end-selected[1]))
        class Startup(live.JointStateStartup):CLEAN_SECONDS=0
        class Mode(DirectMoveLineMode):
            def make_sink(self,factory,request):
                sink=super().make_sink(lambda:transport,request)
                sink.gripper_cancel.wait=lambda seconds:False # TEST ONLY: no pulse dwell.
                return sink
        def module(**fields):return types.SimpleNamespace(**fields)
        fake_modules={
            'rclpy':module(init=lambda **kw:None,create_node=lambda name:Node(),spin_once=spin_once,shutdown=lambda:None),
            'rclpy.signals':module(SignalHandlerOptions=module(NO=0)),
            'rclpy.qos':module(qos_profile_sensor_data=object()),
            'sensor_msgs':module(),'sensor_msgs.msg':module(Image=type('Image',(),{}),JointState=type('JointState',(),{})),
            'dsr_msgs2':module(),'dsr_msgs2.srv':module(**services),
            'rosidl_runtime_py':module(),'rosidl_runtime_py.convert':module(message_to_ordereddict=lambda row:row)}
        mode=Mode();request=mode.launch_request('FAKE_TEST_ONLY')
        with tempfile.TemporaryDirectory() as temp:
            output=Path(temp)/'episode'
            with patch.dict('sys.modules',fake_modules),patch.object(live,'JointStateStartup',Startup), \
                 patch.object(live,'approved_episode_limits',return_value=(2,10)), \
                 patch.object(live,'predict_frame',side_effect=predict),patch.object(live,'warm_up'), \
                 patch.object(live,'validate_identity'), \
                 patch.object(live,'urlopen',side_effect=lambda *a,**kw:io.StringIO('{"status":"ready"}')), \
                 patch.object(live,'archive_request_jpeg'),patch.object(live,'deferred_artifacts',return_value={}), \
                 patch('builtins.input') as prompt,contextlib.redirect_stdout(io.StringIO()):
                live.run(output,request,_full_policy_mode=mode)
            prompt.assert_not_called()
            summary=json.loads((output/'trial_summary.json').read_text())
            init=json.loads((output/'gripper_initialization.json').read_text()) if (output/'gripper_initialization.json').exists() else None
        return summary,init,transport.calls

    def test_direct_episode_no_manual_input_and_complete_original_delta(self):
        summary,init,calls=self.execute()
        self.assertEqual(summary['status'],'OPERATOR_RESULT_CONFIRMATION_PENDING',summary)
        self.assertEqual(summary['steps_completed'],2)
        self.assertTrue(summary['runtime_valid'])
        self.assertFalse(summary['manual_confirmation_performed'])
        self.assertIsNone(summary['initial_gripper_observed_open'])
        self.assertEqual(summary['initial_gripper_basis'],'COMMAND_ACK_ONLY')
        self.assertIsNone(init['observed_open']);self.assertEqual(init['initialization_basis'],'COMMAND_ACK_ONLY')
        arm=[r for _,typ,r in calls if typ=='MoveLine']
        self.assertEqual(len(arm),2)
        self.assertTrue(all(abs(r['pos'][0]-365)<1e-8 and r['vel']==[120.,20.] for r in arm))
        self.assertEqual(sum(typ=='SetToolDigitalOutput' for _,typ,r in calls),8)

    def test_direct_mode_still_refuses_abnormal_current_hardware(self):
        summary,init,calls=self.execute(bad_hardware=True)
        self.assertEqual(summary['status'],'NOT_EXECUTED')
        self.assertIsNone(init);self.assertEqual(calls,[])
