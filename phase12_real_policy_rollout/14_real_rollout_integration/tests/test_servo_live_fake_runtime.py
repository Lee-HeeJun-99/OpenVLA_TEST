"""Run the complete launcher with fake ROS only: never import a real transport."""
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

import start_servo_episode_confirmed as live


class FakeRuntimeTests(unittest.TestCase):
    def execute(self, *, bad_state=False, full=False, stop_fail=False):
        published=[]; service_calls=[]; state_reads=[0]; callbacks=[]
        def now_stamp():
            now=time.time_ns()
            return types.SimpleNamespace(sec=now//10**9,nanosec=now%10**9)
        class Request:pass
        services={name:type(name,(),{'Request':Request}) for name in
                  ['GetRobotSystem','GetRobotMode','GetRobotState','GetCurrentPosx','MoveStop']}
        class Client:
            def __init__(self,name):self.name=name
            def wait_for_service(self,**kw):return True
            def service_is_ready(self):return True
            def call_async(self,request):
                service_calls.append((self.name,vars(request).copy()))
                if self.name.endswith('get_robot_system'):r=dict(success=True,robot_system=0)
                elif self.name.endswith('get_robot_mode'):r=dict(success=True,robot_mode=1)
                elif self.name.endswith('get_robot_state'):
                    state_reads[0]+=1
                    r=dict(success=True,robot_state=3 if bad_state and state_reads[0]>=4 else 1)
                elif self.name.endswith('get_current_posx'):
                    r=dict(success=True,task_pos_info=[dict(data=[305.,-11.,723.,179.,-155.,3.])])
                else:r=dict(success=not stop_fail)
                return types.SimpleNamespace(done=lambda:True,result=lambda:r)
        class Publisher:
            def get_subscription_count(self):return 1
            def publish(self,msg):published.append(vars(msg).copy())
        class Node:
            def create_client(self,kind,name):return Client(name)
            def destroy_client(self,client):pass
            def create_subscription(self,kind,name,cb,qos):callbacks.append((name,cb))
            def count_publishers(self,name):return 0
            def create_publisher(self,*a):return Publisher()
            def get_clock(self):return types.SimpleNamespace(now=lambda:types.SimpleNamespace(nanoseconds=time.time_ns()))
            def destroy_node(self):pass
        def spin_once(node,**kw):
            time.sleep(.004)
            for name,callback in callbacks:
                stamp=now_stamp()
                if 'joint_states' in name:
                    msg=types.SimpleNamespace(header=types.SimpleNamespace(stamp=stamp),
                        name=[f'joint_{i}' for i in range(1,7)],
                        position=[math.radians(v) for v in live.TARGET],velocity=[0.]*6)
                else:msg=types.SimpleNamespace(header=types.SimpleNamespace(stamp=stamp),width=1280,height=720,encoding='bgra8')
                callback(msg)
        def predict(selected,ros_now,check):
            check(); start=time.monotonic();time.sleep(.03);check()
            return dict(raw=[.01,0,0,0,0,0,1],t0=start,jpeg=b'fake-input',timing=dict(inference_latency=.03))
        class QuickStartup(live.JointStateStartup):CLEAN_SECONDS=0
        def module(**fields):return types.SimpleNamespace(**fields)
        fake_modules={
            'rclpy':module(init=lambda **kw:None,create_node=lambda name:Node(),spin_once=spin_once,shutdown=lambda:None),
            'rclpy.signals':module(SignalHandlerOptions=module(NO=0)),
            'rclpy.qos':module(qos_profile_sensor_data=object(),QoSProfile=lambda **kw:object(),ReliabilityPolicy=module(RELIABLE=1)),
            'sensor_msgs':module(), 'sensor_msgs.msg':module(Image=type('Image',(),{}),JointState=type('JointState',(),{})),
            'dsr_msgs2':module(), 'dsr_msgs2.msg':module(ServolStream=type('ServolStream',(),{})),
            'dsr_msgs2.srv':module(**services),
            'rosidl_runtime_py':module(), 'rosidl_runtime_py.convert':module(message_to_ordereddict=lambda x:x),
        }
        with tempfile.TemporaryDirectory() as temp:
            out=Path(temp)/'episode'
            from real_sink import FakeRosTransport
            from full_policy_live_mode import FullPolicyLiveMode
            io_transport=FakeRosTransport()
            class FakeOperatorMode(FullPolicyLiveMode):
                def approved_gripper_config(self,approval):
                    cfg=super().approved_gripper_config(approval)
                    for key in ('open_first_high_hold_s','open_second_high_hold_s','open_between_pulses_low_hold_s','close_high_hold_s'):cfg[key]=0
                    return cfg
                def initialize(self,controller,check,save):
                    def observed_save(name,row):
                        save(name,row)
                        if row.get('ack_wall_time'):
                            self.open_confirmation.write_text(json.dumps(dict(observed_open=True,
                                operator='FAKE_TEST_NOT_HARDWARE',confirmed_wall_time=time.time())))
                    return super().initialize(controller,check,observed_save)
            mode=FakeOperatorMode(Path(temp)/'open.json') if full else None
            approval=dict(execution_mode='FULL_POLICY_ROLLOUT',physical_controller='servo',confirmed_wall_time=time.time(),
                **dict.fromkeys(('operator_present','workspace_clear','estop_accessible','robot_stationary',
                'tcp_engineering_acceptance','gripper_mapping_confirmed','explicit_one_episode',
                'servo_risks_accepted','concurrent_arm_gripper_accepted',
                'authority_and_stops_confirmed','no_other_motion_executor'),True))
            with patch.dict('sys.modules',fake_modules),patch.dict('os.environ',{}), \
                 patch('builtins.input',return_value='yes'), \
                 patch.object(live,'JointStateStartup',QuickStartup), \
                 patch.object(live,'predict_frame',side_effect=predict), \
                 patch.object(live,'validate_identity'), \
                 patch('real_sink.RosServiceTransport',return_value=io_transport), \
                 patch.object(live,'urlopen',side_effect=lambda *a,**k:io.StringIO('{"status":"ready"}')), \
                 contextlib.redirect_stdout(io.StringIO()):
                live.run(out,.35,_full_policy_mode=mode,_approval=approval if full else None)
            summary=json.loads((out/'trial_summary.json').read_text())
            if full:summary['fake_IO_calls']=io_transport.calls
        return summary,published,service_calls

    def test_full_policy_fake_episode_initializes_and_transitions(self):
        summary,messages,calls=self.execute(full=True)
        self.assertTrue(summary['runtime_valid'],summary)
        self.assertEqual(summary['status'],'OPERATOR_RESULT_CONFIRMATION_PENDING')
        writes=[v for _,typ,v in summary['fake_IO_calls'] if typ=='SetToolDigitalOutput']
        self.assertEqual([(v['index'],v['value']) for v in writes],
            [(1,0),(2,0),(1,1),(1,0),(1,1),(1,0),(2,1),(2,0)])
        self.assertGreater(len(messages),0)

    def test_full_policy_stop_failure_also_OFF_cleans(self):
        summary,messages,calls=self.execute(full=True,stop_fail=True)
        self.assertFalse(summary['runtime_valid'])
        self.assertEqual(summary['status'],'INVALID')
        self.assertFalse(summary['stop_ack'])
        self.assertEqual(summary['gripper_summary']['abort_result']['state'],'completed')
        writes=[v for _,typ,v in summary['fake_IO_calls'] if typ=='SetToolDigitalOutput']
        self.assertEqual(writes[-2:],[dict(index=1,value=0),dict(index=2,value=0)])

    def test_normal_complete_fake_episode(self):
        summary,messages,calls=self.execute()
        self.assertEqual(summary['status'],'ARM_ONLY_SERVO_EPISODE_OPERATOR_RESULT_PENDING')
        self.assertTrue(summary['runtime_valid']);self.assertTrue(summary['stop_ack'])
        self.assertGreater(len(messages),0)
        self.assertTrue(all(m['vel']==[20.,20.] and m['acc']==[40.,40.] and m['time']==.7 for m in messages))
        self.assertEqual(summary['servo_publish_returned'],len(messages))
        stops=[r for name,r in calls if name.endswith('move_stop')]
        self.assertEqual(stops,[{'stop_mode':3}])
        self.assertEqual(summary['gripper_commands'],0)
        self.assertTrue(all('digital_output' not in name and 'move_line' not in name for name,r in calls))

    def test_robot_fault_stops_fake_episode(self):
        summary,messages,calls=self.execute(bad_state=True)
        self.assertEqual(summary['status'],'ARM_ONLY_SERVO_EPISODE_INVALID')
        self.assertIn('robot_abnormal_state',summary['reason'])
        self.assertFalse(summary['runtime_valid'])
        self.assertGreater(len(messages),0)
        self.assertEqual(len([name for name,r in calls if name.endswith('move_stop')]),1)


if __name__=='__main__':unittest.main()
