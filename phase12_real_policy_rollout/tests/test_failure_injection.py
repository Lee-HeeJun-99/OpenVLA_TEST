import math,sys,unittest
from pathlib import Path
R=Path(__file__).resolve().parents[1];sys.path.insert(0,str(R/'02_safety'))
from canonical_action import CanonicalAction
from safety_pipeline import SafetyPipeline,RuntimeState
class Tests(unittest.TestCase):
 def test_all_runtime_failures_reject(self):
  cases=('camera_ok','joint_state_ok','tcp_ok','model_ok','communication_ok','logger_ok','joint_limits_ok')
  for field in cases:
   p=SafetyPipeline('openvla',operator_confirmed_initial_open=True)
   a=CanonicalAction.from_vector([0]*7,timestamp_monotonic=1,sequence_id=field,source_model='openvla')
   kw={field:False};s=RuntimeState(1,(.4,0,.5),(10,20,30),'alignment',**kw);d=p.inspect(a,s)
   self.assertFalse(d.accepted,field);self.assertFalse(d.command_issued)
 def test_large_workspace_velocity_acceleration_duplicate_stale_nan(self):
  vectors=([.1,0,0,0,0,0,0],[0,0,0,1,0,0,0])
  for i,v in enumerate(vectors):
   p=SafetyPipeline('openvla',operator_confirmed_initial_open=True);a=CanonicalAction.from_vector(v,timestamp_monotonic=1,sequence_id=str(i),source_model='openvla')
   self.assertFalse(p.inspect(a,RuntimeState(1,(.4,0,.5),(10,20,30),'alignment')).accepted)
  with self.assertRaises(ValueError):CanonicalAction.from_vector([math.nan]+[0]*6,timestamp_monotonic=1,sequence_id='n',source_model='openvla')
if __name__=='__main__':unittest.main()
