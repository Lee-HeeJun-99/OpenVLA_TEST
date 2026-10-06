import csv,importlib.util
from pathlib import Path
import unittest

H=Path(__file__).resolve().parents[1]
class TestHeatmapPreparation(unittest.TestCase):
 def test_selected_frames(self):
  with (H/'01_selected_frames/selected_frames.csv').open() as f:r=list(csv.DictReader(f))
  self.assertTrue(r);self.assertEqual(18,sum(x['high_cost'].lower()=='true' for x in r));self.assertTrue(all(Path(x['baseline_image']).exists() and Path(x['condition_image']).exists() for x in r))
 def test_no_robot_ros_dependency(self):
  for p in (H/'scripts').glob('*.py'):
   s=p.read_text();
   for forbidden in ('rclpy','move_line(','move_joint(','gripper.close(','--execute'):
    self.assertNotIn(forbidden,s)
 def test_models_separated(self):
  with (H/'01_selected_frames/selected_frames.csv').open() as f:r=list(csv.DictReader(f))
  self.assertEqual({'openvla','oft'},{x['model'] for x in r})
 def test_occlusion_inventory_and_chunks(self):
  import numpy as np
  with (H/'heatmap_inventory.csv').open() as f:r=list(csv.DictReader(f))
  self.assertEqual(72,len(r))
  for x in r:
   z=np.load(x['heatmap_file']);a=z['maps'];self.assertTrue(np.isfinite(a).all());self.assertEqual(4,a.shape[1]);self.assertEqual(5 if x['model']=='oft' else 1,a.shape[0])
if __name__=='__main__':unittest.main()
