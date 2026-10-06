import csv
import json
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parent


class TestOFTGripperAudit(unittest.TestCase):
    def test_required_outputs_and_command_invariant(self):
        results = ROOT / "results"
        for name in (
            "summary.json", "training_gripper_by_k.csv",
            "episode_gripper_timing.csv", "checkpoint_vs_reference.csv",
        ):
            self.assertTrue((results / name).is_file(), name)
        summary = json.loads((results / "summary.json").read_text())
        self.assertFalse(summary["normalization_contract_problem"])
        self.assertFalse(summary["checkpoint_contract"]["affine_normalization_applied_to_gripper"])
        self.assertTrue(all(value == 0 for value in summary["execution_counts"].values()))

    def test_k_indices_and_episode4(self):
        with (ROOT / "results/training_gripper_by_k.csv").open() as handle:
            rows = list(csv.DictReader(handle))
        by_source = {}
        for row in rows:
            by_source.setdefault(row["source_provenance"], set()).add(int(row["k_index"]))
        self.assertTrue(all(indices == set(range(5)) for indices in by_source.values()))
        summary = json.loads((ROOT / "results/summary.json").read_text())
        episode4 = summary["phase10_episode4"]
        self.assertEqual(episode4["first_close_target_step"], 2)
        self.assertEqual(episode4["reference_close_step"], 26)
        self.assertAlmostEqual(episode4["lead_seconds"], 4.8)

    def test_saved_checkpoint_outputs_show_k_trend(self):
        summary = json.loads((ROOT / "results/summary.json").read_text())
        means = [row["mean"] for row in summary["phase11_prediction"]["by_k"]]
        self.assertEqual(means, sorted(means))
        self.assertEqual(summary["phase11_prediction"]["episode_condition_pairs"], 20)


if __name__ == "__main__":
    unittest.main()
