from pathlib import Path
import json
import sys
import unittest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "05_model_adapters"))
from openvla_adapter import OpenVLAAdapter  # noqa: E402
from oft_adapter import OFTAdapter  # noqa: E402


class RecordedModelPipelineSyntheticFixture(unittest.TestCase):
    """SYNTHETIC_FIXTURE responses; not research/model-server results."""
    def test_same_recorded_image_k1_k5_and_null_execution(self):
        episode = Path("/home/ubuntu/a0509_vla_linux_field_bundle_20260903/data/real_world/raw_dataset_oft/episodes/episode_000004")
        step = json.loads((episode / "steps.jsonl").read_text().splitlines()[0])
        image = Path("/home/ubuntu/a0509_vla_linux_field_bundle_20260903/data/real_world/raw_dataset_oft") / step["image"]
        openvla = OpenVLAAdapter(predictor=lambda _: {"variant": "openvla_token", "action": [0]*7})
        oft = OFTAdapter(predictor=lambda _: {"variant": "oftplus_h5_vision", "actions": [[0]*7 for _ in range(5)]})
        one = openvla.predict_recorded(image, step["instruction"])
        five = oft.predict_recorded(image, step["instruction"])
        self.assertIsNone(one["executed_action"])
        self.assertIsNone(five["executed_action"])
        self.assertEqual(len(five["canonical_action_chunk"]), 5)

