import unittest
from types import SimpleNamespace
from live_observation import camera_rgb_array

class CameraEncodingTests(unittest.TestCase):
    def test_bgra_to_rgb_with_padding(self):
        m=SimpleNamespace(encoding='bgra8',width=2,height=1,step=10,data=bytes([1,2,3,255,4,5,6,0,99,99]))
        self.assertEqual(camera_rgb_array(m).tolist(),[[[3,2,1],[6,5,4]]])
    def test_rgb_preserved(self):
        m=SimpleNamespace(encoding='rgb8',width=1,height=1,step=3,data=bytes([1,2,3]))
        self.assertEqual(camera_rgb_array(m).tolist(),[[[1,2,3]]])
    def test_unknown_refused(self):
        with self.assertRaises(ValueError):camera_rgb_array(SimpleNamespace(encoding='mono8'))
