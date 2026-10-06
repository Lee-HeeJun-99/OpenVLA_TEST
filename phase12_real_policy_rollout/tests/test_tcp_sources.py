import sys,unittest
from pathlib import Path
R=Path(__file__).resolve().parents[1];sys.path.insert(0,str(R/'02_safety'))
from tcp_sources import MeasuredTcpSource,JointStateFkTcpSource,RecordedTcpSource,UnavailableTcpSource
class Tests(unittest.TestCase):
 def test_provenance(self):
  self.assertEqual('MEASURED_CONTROLLER_TCP',MeasuredTcpSource().sample({'measured_tcp_pose':[1]})['source'])
  self.assertEqual('RECORDED_TCP',RecordedTcpSource().sample({'recorded_tcp_pose':[1]})['source'])
  fk=JointStateFkTcpSource(lambda j:{'position_m':[j['joint_1'],0,0]})
  r=fk.sample({'joint_positions_by_name':{'joint_1':1}});self.assertEqual('FK_ESTIMATED_TCP',r['source']);self.assertFalse(r['measured'])
 def test_unavailable_and_missing_fail(self):
  for source,obs in ((MeasuredTcpSource(),{}),(RecordedTcpSource(),{}),(UnavailableTcpSource(),{})):
   with self.assertRaises(RuntimeError):source.sample(obs)
if __name__=='__main__':unittest.main()
