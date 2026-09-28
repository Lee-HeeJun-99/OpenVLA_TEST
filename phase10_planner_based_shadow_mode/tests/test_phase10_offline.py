#!/usr/bin/env python3

from __future__ import annotations

import json
import csv
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "05_model_adapters"))
sys.path.insert(0, str(ROOT / "02_safety"))
from action_canonicalizer import canonicalize_action, integrated_displacement  # noqa: E402
from safety_gate import ShadowCommandGate, decide  # noqa: E402


class Phase10OfflineTests(unittest.TestCase):
    def test_canonical_action_and_horizon(self):
        action = canonicalize_action([1, 2, 3, 180, 0, -180, 1], source="test", translation_unit="mm", rotation_unit="deg")
        self.assertAlmostEqual(action.delta_translation_m[0], 0.001)
        self.assertAlmostEqual(action.delta_rotation_rotvec_rad[0], 3.141592653589793)
        result = integrated_displacement([action, action])
        self.assertAlmostEqual(result["time_horizon_seconds"], 0.4)

    def test_shadow_ai_publish_is_structurally_blocked(self):
        with self.assertRaises(PermissionError):
            ShadowCommandGate().publish_ai_action([0.0] * 7)

    def test_ai_timeout_does_not_force_planner_hold(self):
        result = decide({"oft_timeout": True})
        self.assertFalse(result.hold_required)
        self.assertTrue(result.planner_may_continue)
        self.assertTrue(result.invalid_prediction)

    def test_planner_timeout_forces_hold(self):
        result = decide({"planner_timeout": True})
        self.assertTrue(result.hold_required)
        self.assertFalse(result.planner_may_continue)

    def test_recorded_input_smoke(self):
        episode = Path("/home/ubuntu/a0509_vla_linux_field_bundle_20260903/data/real_world/raw_dataset_oft/episodes/episode_000004")
        if not episode.exists():
            self.skipTest("local Phase 8 episode fixture unavailable")
        with tempfile.TemporaryDirectory() as temp:
            output = Path(temp) / "shadow"
            command = [
                sys.executable, str(ROOT / "06_shadow_collection" / "shadow_mode_runner.py"),
                "--episode-dir", str(episode), "--output-dir", str(output),
                "--trial-id", "smoke_trial", "--condition-id", "fixture_only",
                "--layout-id", "episode4", "--max-steps", "2",
                "--openvla-fixture", str(ROOT / "tests/fixtures/openvla_response.json"),
                "--oft-fixture", str(ROOT / "tests/fixtures/oft_response.json"),
            ]
            subprocess.run(command, check=True, capture_output=True, text=True)
            rows = [json.loads(line) for line in (output / "samples.jsonl").read_text().splitlines()]
            self.assertEqual(len(rows), 2)
            self.assertIsNone(rows[0]["openvla_executed_action"])
            self.assertIsNone(rows[0]["oft_executed_action"])
            self.assertEqual(len(rows[0]["oft_denormalized_action_chunk"]), 5)
            self.assertTrue(rows[0]["fixture_smoke_test_only"])
            aligned = Path(temp) / "aligned.csv"
            subprocess.run([
                sys.executable, str(ROOT / "08_time_alignment/align_recorded_samples.py"),
                "--samples", str(output / "samples.jsonl"), "--output", str(aligned),
            ], check=True, capture_output=True, text=True)
            with aligned.open(newline="", encoding="utf-8") as handle:
                aligned_rows = list(csv.DictReader(handle))
            self.assertEqual(aligned_rows[0]["clock_domain_comparable"], "False")
            self.assertEqual(aligned_rows[0]["camera_state_delta_sec"], "")
            self.assertEqual(aligned_rows[0]["alignment_method"], "frame_id_with_recorded_callback_ages")

    def test_planner_static_dry_run(self):
        episode = Path("/home/ubuntu/a0509_vla_linux_field_bundle_20260903/data/real_world/raw_dataset_oft/episodes/episode_000004")
        if not episode.exists():
            self.skipTest("local Phase 8 episode fixture unavailable")
        with tempfile.TemporaryDirectory() as temp:
            output = Path(temp) / "planner_dry_run.json"
            subprocess.run([
                sys.executable, str(ROOT / "04_planner_reference/planner_dry_run.py"),
                "--episode-dir", str(episode), "--output", str(output),
            ], check=True, capture_output=True, text=True)
            result = json.loads(output.read_text())
            self.assertEqual(result["status"], "PASS_OFFLINE_ONLY")
            self.assertEqual(result["robot_commands_published"], 0)
            self.assertEqual(result["pose_source"], "planned_commanded_pose")


if __name__ == "__main__":
    unittest.main()
