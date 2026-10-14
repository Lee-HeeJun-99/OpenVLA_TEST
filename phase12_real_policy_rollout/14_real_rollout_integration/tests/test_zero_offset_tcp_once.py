import unittest
from validate_zero_offset_tcp_once import compare

class PoseComparisonTests(unittest.TestCase):
    def test_identity(self):
        result=compare([1,2,3,10,20,30],[1,2,3,10,20,30])
        self.assertAlmostEqual(result['translation_error_mm'],0)
        self.assertAlmostEqual(result['orientation_error_deg'],0)

    def test_equivalent_euler(self):
        result=compare([0,0,0,10,20,30],[0,0,0,-170,-20,210])
        self.assertAlmostEqual(result['orientation_error_deg'],0)

    def test_translation(self):
        self.assertEqual(compare([0]*6,[3,4,0,0,0,0])['translation_error_mm'],5)
