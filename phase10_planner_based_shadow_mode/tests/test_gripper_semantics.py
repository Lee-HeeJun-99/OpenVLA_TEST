from pathlib import Path
import sys
import unittest

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'05_model_adapters'))
from action_canonicalizer import canonicalize_chunk  # noqa:E402
from gripper_semantics import dataset_closedness,runtime_gripper_open,runtime_oft_action,runtime_sequence  # noqa:E402


class GripperSemanticsSyntheticFixture(unittest.TestCase):
    """SYNTHETIC_FIXTURE using copied, source-cited Real runtime semantics."""
    def setUp(self):
        self.chunk=[[0,0,0,0,0,0,g] for g in (.2,.8,.9,.6,.1)]

    def test_scale_canonicalization_and_chunk_order(self):
        canonical=canonicalize_chunk(self.chunk,source='fixture')
        self.assertEqual([a.gripper_closedness for a in canonical],[.2,.8,.9,.6,.1])
        self.assertEqual(runtime_oft_action(self.chunk),self.chunk[0])

    def test_polarity_mismatch_is_explicit(self):
        self.assertFalse(dataset_closedness(.2))
        self.assertFalse(runtime_gripper_open(.2,last_open=True))  # False means hardware CLOSE
        self.assertTrue(dataset_closedness(.8))
        self.assertTrue(runtime_gripper_open(.8,last_open=False))  # True means hardware OPEN

    def test_threshold_hysteresis_and_repeated_close(self):
        self.assertEqual(runtime_sequence([.5,.2,.2,.5,.8],initial_open=True),
                         [True,False,False,False,True])

    def test_no_hidden_clipping(self):
        with self.assertRaises(ValueError):
            canonicalize_chunk([[0,0,0,0,0,0,1.2]]*5,source='fixture')
        self.assertTrue(runtime_gripper_open(1.2,last_open=False))
