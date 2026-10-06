import math,sys,unittest
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
R=Path(__file__).resolve().parents[1];sys.path.insert(0,str(R/'02_safety'))
from action_contract import compose_world_rotvec_with_doosan_zyz,doosan_zyz_deg_to_matrix
class Tests(unittest.TestCase):
 def test_identity_axes_wrap_and_inverse_by_matrix(self):
  current=(179.9,.1,-179.8);base=doosan_zyz_deg_to_matrix(current)
  vectors=([0,0,0],[math.radians(1),0,0],[0,math.radians(1),0],[0,0,math.radians(1)],
           [math.radians(5),0,0],[0,math.radians(5),0],[0,0,math.radians(5)],[1e-12,0,0])
  for v in vectors:
   abc,m=compose_world_rotvec_with_doosan_zyz(current,v)
   self.assertTrue(np.allclose(doosan_zyz_deg_to_matrix(abc),m,atol=1e-9))
   self.assertTrue(np.allclose(m,Rotation.from_rotvec(v).as_matrix()@base,atol=1e-9))
  _,f=compose_world_rotvec_with_doosan_zyz(current,[.02,-.01,.03])
  inv=Rotation.from_rotvec([.02,-.01,.03]).inv().as_matrix()@f
  self.assertTrue(np.allclose(inv,base,atol=1e-9))
if __name__=='__main__':unittest.main()
