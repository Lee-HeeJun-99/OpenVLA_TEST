from pathlib import Path
import sys
import unittest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "04_planner_reference"))
from reference_command_extractor import extract  # noqa: E402


class ReferenceExtractorRecordedDataTest(unittest.TestCase):
    """RECORDED_DATA_TEST: no model or robot execution."""
    def test_episode4_reference_commands(self):
        episode = Path("/home/ubuntu/a0509_vla_linux_field_bundle_20260903/data/real_world/raw_dataset_oft/episodes/episode_000004")
        rows = extract(episode)
        self.assertEqual(len(rows), 45)
        self.assertTrue(all(row["pose_source"] == "planned_commanded_pose" for row in rows))
        self.assertTrue(all(row["measured_action"] is None for row in rows))
        self.assertTrue(all(len(row["canonical_action"]) == 7 for row in rows))

