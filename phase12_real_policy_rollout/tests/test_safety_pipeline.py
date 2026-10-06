import sys,unittest
from pathlib import Path
R=Path(__file__).resolve().parents[1];sys.path.insert(0,str(R/'02_safety'))
from canonical_action import CanonicalAction
from safety_pipeline import SafetyPipeline,RuntimeState
def action(v=None,i='a'):return CanonicalAction.from_vector(v or [0]*7,timestamp_monotonic=1,sequence_id=i,source_model='openvla')
def state(**kw):return RuntimeState(1,(.4,0,.5),(10,20,30),'alignment',**kw)
class Tests(unittest.TestCase):
 def test_accept_and_joint_limit_reject(self):
  self.assertTrue(SafetyPipeline('openvla',operator_confirmed_initial_open=True).inspect(action(),state()).accepted)
  d=SafetyPipeline('openvla',operator_confirmed_initial_open=True).inspect(action(),state(joint_limits_ok=False));self.assertFalse(d.accepted);self.assertTrue(d.hold_required)
if __name__=='__main__':unittest.main()
