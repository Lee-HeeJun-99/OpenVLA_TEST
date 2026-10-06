import sys,tempfile,unittest
from pathlib import Path
R=Path(__file__).resolve().parents[1];sys.path[:0]=[str(R/'02_safety'),str(R/'03_shadow_mode')]
from pre_motion_runtime_facade import PreMotionRuntimeFacade
from integrated_logger import FsyncJsonlLogger

POSE=dict(current_position_m=(.4,0,.5),current_abc_deg=(179.74,-155.72,3.30),phase='alignment')
CTX=dict(action_id='x',source_monotonic=1.,now_monotonic=1.,**POSE)

class BrokenLogger:
 def append(self,_):raise OSError('disk full')

class Tests(unittest.TestCase):
 def test_initial_open_confirmation_required(self):
  with self.assertRaises(RuntimeError):PreMotionRuntimeFacade('openvla',operator_confirmed_initial_open=False)
 def test_openvla_boundary_null_delivery(self):
  f=PreMotionRuntimeFacade('openvla',operator_confirmed_initial_open=True)
  row=f.inspect_openvla([.001,0,0,0,0,0,0],**CTX)
  self.assertTrue(row['technical_valid']);self.assertTrue(row['hard_gate']['delivery_blocked'])
  self.assertIsNone(row['executed_action']);self.assertIsNone(row['robot_delivered_command']);self.assertFalse(row['command_issued'])
 def test_oft_sequential_k5_and_premature_close_block(self):
  f=PreMotionRuntimeFacade('oft',operator_confirmed_initial_open=True)
  actions=[[0,0,0,0,0,0,g] for g in (.1,.2,.5,.8,.9)]
  f.enqueue_oft(actions,'c',1.)
  rows=[f.inspect_next_oft(1.+.2*i,phase='alignment',current_position_m=(.4,0,.5),current_abc_deg=(0,0,0)) for i in range(5)]
  self.assertEqual([0,1,2,3,4],[r['chunk_index'] for r in rows])
  self.assertTrue(all(not r['command_issued'] for r in rows));self.assertFalse(rows[-1]['technical_valid'])
 def test_logger_failure_faults_closed(self):
  f=PreMotionRuntimeFacade('openvla',operator_confirmed_initial_open=True,logger=BrokenLogger())
  row=f.inspect_openvla([0]*7,**CTX);self.assertTrue(row['hold_required']);self.assertIn('logger_failure',row['technical_blockers'])
  with self.assertRaises(RuntimeError):f.inspect_openvla([0]*7,**{**CTX,'action_id':'y','now_monotonic':1.2})
 def test_fsync_logger(self):
  with tempfile.TemporaryDirectory() as d:
   with FsyncJsonlLogger(Path(d)/'boundary.jsonl',minimum_free_bytes=0) as log:
    row=PreMotionRuntimeFacade('openvla',operator_confirmed_initial_open=True,logger=log).inspect_openvla([0]*7,**CTX)
   self.assertFalse(row['command_issued'])
 def test_zero_command_capability_static(self):
  s=(R/'02_safety/pre_motion_runtime_facade.py').read_text().lower()
  for token in ('rclpy','create_publisher','create_client','actionclient','call_async','movej','movel','servol','speedl','set_tool_digital'):
   self.assertNotIn(token,s)
  caps=PreMotionRuntimeFacade('openvla',operator_confirmed_initial_open=True).capability_report
  self.assertFalse(any(v for k,v in caps.items() if k!='sink'))

if __name__=='__main__':unittest.main()
