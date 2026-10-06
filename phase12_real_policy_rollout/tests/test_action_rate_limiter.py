import math,sys,unittest
from pathlib import Path
R=Path(__file__).resolve().parents[1];sys.path.insert(0,str(R/'02_safety'))
from action_rate_limiter import CanonicalActionRateLimiter

class Tests(unittest.TestCase):
 def test_first_step_accelerates_from_zero_and_gripper_unchanged(self):
  r=CanonicalActionRateLimiter().limit([.004,0,0,math.radians(4),0,0,.8])
  self.assertAlmostEqual(.0008,r.limited_action[0],places=9)
  self.assertAlmostEqual(math.radians(.8),r.limited_action[3],places=9)
  self.assertEqual(.8,r.limited_action[6]);self.assertTrue(r.translation_acceleration_clipped);self.assertTrue(r.rotation_acceleration_clipped)
 def test_constant_command_ramps_without_exceeding_acceleration(self):
  l=CanonicalActionRateLimiter();prev=0
  for _ in range(5):
   r=l.limit([.004,0,0,0,0,0,0]);v=r.translation_velocity_m_s[0]
   self.assertLessEqual(abs(v-prev),.0040000001);self.assertLessEqual(abs(v),.020000001);prev=v
 def test_direction_reversal_is_acceleration_limited(self):
  l=CanonicalActionRateLimiter();
  for _ in range(5):l.limit([.004,0,0,0,0,0,0])
  r=l.limit([-.004,0,0,0,0,0,0]);self.assertGreaterEqual(r.translation_velocity_m_s[0],.0159999);self.assertTrue(r.translation_acceleration_clipped)
 def test_invalid_rejected_and_no_command_capability(self):
  self.assertRaises(ValueError,CanonicalActionRateLimiter().limit,[float('nan')]+[0]*6)
  text=(R/'02_safety/action_rate_limiter.py').read_text().lower()
  for bad in ('rclpy','create_publisher','create_client','call_async','movej','movel','servol','speedl'):self.assertNotIn(bad,text)
if __name__=='__main__':unittest.main()
