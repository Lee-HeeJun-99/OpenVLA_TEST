import json,math,tempfile,unittest
from pathlib import Path
import sys
R=Path(__file__).resolve().parents[1];sys.path[:0]=[str(R/'02_safety'),str(R/'03_shadow_mode')]
from gripper_contract import ClosednessHysteresis
from oft_chunk_queue import OFTChunkQueue,ChunkSafetyError
from safety_contract import validate_action,validate_joint_state,validate_startup
from integrated_logger import FsyncJsonlLogger,LoggerFailure
from action_contract import (compose_world_rotvec_with_doosan_zyz,
                             doosan_zyz_deg_to_matrix,translation_m_to_mm)
import numpy as np
from scipy.spatial.transform import Rotation

class Tests(unittest.TestCase):
 def test_gripper_polarity_hysteresis_suppression(self):
  g=ClosednessHysteresis();self.assertFalse(g.resolve(.1).commanded_closed);self.assertTrue(g.resolve(.5).command_suppressed);self.assertTrue(g.resolve(.9).commanded_closed);self.assertTrue(g.resolve(.8).command_suppressed);self.assertTrue(g.resolve(.5).commanded_closed);self.assertFalse(g.resolve(.2).commanded_closed)
 def test_k5_order_and_first_only(self):
  c=[[i,0,0,0,0,0,0] for i in range(5)];q=OFTChunkQueue();q.enqueue(c,'a',1.0);self.assertEqual(list(range(5)),[q.pop(1.1+i*.2).chunk_index for i in range(5)])
  q=OFTChunkQueue('first_only');q.enqueue(c,'a',1);self.assertEqual(0,q.pop(1.1).chunk_index);self.assertRaises(ChunkSafetyError,q.pop,1.2)
 def test_translation_contract_rejects_legacy_gain(self):
  converted=translation_m_to_mm([.001,-.002,.003]);self.assertEqual((1.,-2.,3.),converted.delta_mm)
  self.assertRaises(RuntimeError,translation_m_to_mm,[.001,0,0],empirical_gain=2.8)
 def test_doosan_zyz_roundtrip_by_matrix(self):
  for abc in ([10.,20.,30.],[-170.,1.,179.],[0.,90.,0.]):
   matrix=doosan_zyz_deg_to_matrix(abc)
   out,_=compose_world_rotvec_with_doosan_zyz(abc,[0,0,0])
   self.assertTrue(np.allclose(matrix,doosan_zyz_deg_to_matrix(out),atol=1e-10))
 def test_world_rotvec_composition(self):
  abc=[15.,40.,-20.]
  for axis in ([math.radians(1),0,0],[0,math.radians(5),0],[0,0,-math.radians(5)]):
   out,matrix=compose_world_rotvec_with_doosan_zyz(abc,axis)
   expected=Rotation.from_rotvec(axis).as_matrix()@doosan_zyz_deg_to_matrix(abc)
   self.assertTrue(np.allclose(matrix,expected,atol=1e-10))
   self.assertTrue(np.allclose(doosan_zyz_deg_to_matrix(out),expected,atol=1e-10))
 def test_stale_duplicate_invalid_and_underrun(self):
  c=[[0]*7 for _ in range(5)];q=OFTChunkQueue(max_age_sec=.5);q.enqueue(c,'a',1);self.assertRaises(ChunkSafetyError,q.enqueue,c,'a',1.1);self.assertRaises(ChunkSafetyError,q.pop,2);self.assertRaises(ChunkSafetyError,q.pop,2)
  c[2][0]=math.nan;q=OFTChunkQueue();self.assertRaises(ChunkSafetyError,q.enqueue,c,'b',1)
 def test_timeout_communication_workspace_limits(self):
  base=dict(action=[0]*7,current_pose=[0]*6,workspace_min=[-1]*6,workspace_max=[1]*6,max_translation_m=.01,max_rotation_rad=.02)
  for extra,reason in [({'timed_out':True},'inference_timeout'),({'communication_ok':False},'communication_loss'),({'logger_ok':False},'logger_failure')]:self.assertEqual(reason,validate_action(**base,**extra).reason)
  for action,reason in [([math.nan]+[0]*6,'invalid_action'),([.1]+[0]*6,'excessive_translation'),([0]*3+[.1]+[0]*3,'excessive_rotation')]:self.assertEqual(reason,validate_action(**{**base,'action':action}).reason)
  self.assertEqual('workspace_violation',validate_action(**{**base,'action':[.01,0,0,0,0,0,0],'current_pose':[.999,0,0,0,0,0]}).reason)
 def test_unsafe_startup_rejected(self):
  safe={'mode':'offline_preparation','shadow_mode':True,**{k:False for k in ('allow_robot_command','allow_gripper_command','allow_home_command','allow_trajectory_execution','allow_motion_service','allow_stop_service','allow_estop_service')}};validate_startup(safe)
  bad=dict(safe);bad['allow_robot_command']=True;self.assertRaises(RuntimeError,validate_startup,bad)
 def test_joint_and_velocity_limits(self):
  args=([0]*6,[0]*6,[-1]*6,[1]*6,[.5]*6);self.assertTrue(validate_joint_state(*args).accepted)
  self.assertEqual('joint_limit_violation',validate_joint_state([2]+[0]*5,*args[1:]).reason)
  self.assertEqual('joint_velocity_violation',validate_joint_state([0]*6,[1]+[0]*5,*args[2:]).reason)
 def test_fsync_logger_and_executed_null(self):
  with tempfile.TemporaryDirectory() as d:
   p=Path(d)/'x.jsonl'
   with FsyncJsonlLogger(p) as log:log.append({'ai_executed_action':None,'measured_state':None})
   self.assertIsNone(json.loads(p.read_text())['ai_executed_action'])
 def test_no_ros_or_command_dependency(self):
  separately_audited_getter={'query_doosan_state_once.py'}
  passive_ros={'monitor_doosan_state_stack.py','analyze_joint_bag.py','probe_jointstate_continuity.py','probe_zed_streams.py','probe_jointstate_fk_once.py','live_prediction_only_shadow.py'}
  for d in ('02_safety','03_shadow_mode'):
   for p in (R/d).glob('*.py'):
    if p.name in separately_audited_getter:continue
    if p.name in passive_ros:
     s=p.read_text().lower()
     for bad in ('call_async','create_publisher','create_client','move_line(','move_joint(','actionclient','servol','speedl'):self.assertNotIn(bad,s)
     continue  # separately scoped passive/subscriber-only ROS tools
    s=p.read_text();
    for bad in ('rclpy','call_async','create_publisher','move_line(','move_joint('):self.assertNotIn(bad,s)
if __name__=='__main__':unittest.main()
