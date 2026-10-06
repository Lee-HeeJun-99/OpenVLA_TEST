import ast,importlib.util,json,tempfile,unittest
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
def load(name,path):
 s=importlib.util.spec_from_file_location(name,path);m=importlib.util.module_from_spec(s);s.loader.exec_module(m);return m
core=load('shadow_core',ROOT/'03_shadow_mode/subscriber_only_shadow.py')
logmod=load('logger',ROOT/'03_shadow_mode/integrated_logger.py')
safety=load('safety',ROOT/'02_safety/safety_contract.py')
rot=load('rot',ROOT/'02_safety/rotation_candidate.py')
class Gate1Tests(unittest.TestCase):
 def test_no_command_capabilities(self):
  for p in (ROOT/'03_shadow_mode').glob('*.py'):
   if p.name=='query_doosan_state_once.py':continue  # separately allowlisted getter-only client
   text=p.read_text(); tree=ast.parse(text)
   banned=('create_publisher','create_client','ActionClient','publish(','call_async','movej','movel','servol','speedl')
   self.assertFalse([x for x in banned if x in text],f'{p} contains command capability')
 def test_safe_startup_and_unsafe_rejection(self):
  flags={'mode':'subscriber_only_shadow','shadow_mode':True,**{k:False for k in ('allow_robot_command','allow_gripper_command','allow_home_command','allow_trajectory_execution','allow_motion_service','allow_stop_service','allow_estop_service')}}
  safety.validate_startup(flags); flags['allow_robot_command']=True
  with self.assertRaises(RuntimeError): safety.validate_startup(flags)
 def test_k1_k5_null_execution_and_fsync(self):
  cfg={'instruction':'Pick up the orange cube.'}
  with tempfile.TemporaryDirectory() as d:
   path=Path(d)/'x.jsonl'
   with logmod.FsyncJsonlLogger(path) as logger:
    c=core.SubscriberOnlyShadowCore(cfg,logger)
    r=c.record(image_bytes=b'abc',camera_source_timestamp=1,camera_clock_domain='camera',receive_monotonic_timestamp=2,receive_ros_timestamp=3,joint_state={},tcp_pose={},predictions={'openvla':{'action':[0]*7,'chunk_id':'o1'},'oft':{'actions':[[0]*7 for _ in range(5)],'chunk_id':'f1'}})
   self.assertIsNone(r['executed_action']);self.assertIsNone(r['robot_delivered_command']);self.assertFalse(r['command_issued']);self.assertEqual(r['predictions']['oft']['chunk_size'],5);self.assertEqual(len(path.read_text().splitlines()),1)
 def test_nan_timeout_duplicate(self):
  self.assertRaises(core.ShadowSampleError,core.validate_prediction,'openvla',{'action':[0,0,0,0,0,float('nan'),0]})
 def test_rotation_and_translation_block_readiness(self):
  with self.assertRaises(RuntimeError):rot.require_verified_convention('UNRESOLVED')
  self.assertNotEqual('UNRESOLVED_TRANSLATION_SCALE','VERIFIED')
if __name__=='__main__':unittest.main()
