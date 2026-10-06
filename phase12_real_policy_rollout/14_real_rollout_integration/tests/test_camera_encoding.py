import unittest
from types import SimpleNamespace
from live_observation import camera_rgb_array,camera_rgb_image

class CameraEncodingTests(unittest.TestCase):
    def test_bgra_to_rgb_with_padding(self):
        m=SimpleNamespace(encoding='bgra8',width=2,height=1,step=10,data=bytes([1,2,3,255,4,5,6,0,99,99]))
        self.assertEqual(camera_rgb_array(m).tolist(),[[[3,2,1],[6,5,4]]])
    def test_rgb_preserved(self):
        m=SimpleNamespace(encoding='rgb8',width=1,height=1,step=3,data=bytes([1,2,3]))
        self.assertEqual(camera_rgb_array(m).tolist(),[[[1,2,3]]])
    def test_unknown_refused(self):
        with self.assertRaises(ValueError):camera_rgb_array(SimpleNamespace(encoding='mono8'))
    def test_fast_raw_decoder_equivalence(self):
        for encoding,data,step in [('bgra8',[1,2,3,255,4,5,6,0,99,99],10),('rgba8',[3,2,1,255,6,5,4,0,99,99],10),('bgr8',[1,2,3,4,5,6,99],7),('rgb8',[3,2,1,6,5,4,99],7)]:
            m=SimpleNamespace(encoding=encoding,width=2,height=1,step=step,data=bytes(data))
            self.assertEqual(camera_rgb_image(m).tobytes(),camera_rgb_array(m).tobytes())
