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
sys.path.insert(0, str(ROOT / "06_shadow_collection"))
from action_canonicalizer import canonicalize_action, integrated_displacement  # noqa: E402
from safety_gate import ShadowCommandGate, decide  # noqa: E402
from real_runtime_adapter import RealRuntimeObserverAdapter, RemoteSafetyConfig  # noqa: E402
from real_episode_recorder import RealEpisodeRecorder  # noqa: E402


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

    def test_every_remote_command_path_is_blocked(self):
        gate = ShadowCommandGate()
        for method in (gate.publish_robot_command, gate.command_gripper, gate.command_home,
                       gate.execute_trajectory, gate.call_hold_service, gate.call_estop_service,
                       gate.publish_ai_action):
            with self.assertRaises(PermissionError):
                method(None)
        with self.assertRaises(PermissionError):
            RemoteSafetyConfig(allow_service_call=True).validate()

    def test_synthetic_real_runtime_mapping_and_sync(self):
        config = ROOT / "06_shadow_collection" / "real_shadow_config.yaml"
        adapter = RealRuntimeObserverAdapter(config)
        camera = adapter.camera(frame_id="SYNTHETIC_FIXTURE", stamp_sec=10.0,
                                clock_domain="ros_time", image_path="SYNTHETIC_FIXTURE.png")
        state = adapter.measured_state(stamp_sec=10.01, clock_domain="ros_time",
                                       joint_position=[0] * 6, joint_velocity=[0] * 6,
                                       ee_pose_mm_deg=[400, 0, 300, 0, 180, 0], gripper_open=True)
        planner = adapter.observed_action(kind="planner_raw_action", stamp_sec=10.02,
                                          clock_domain="ros_time", values=[0] * 7,
                                          source="SYNTHETIC_FIXTURE")
        executed = adapter.observed_action(kind="executed_command", stamp_sec=10.02,
                                           clock_domain="ros_time", values=[0] * 6,
                                           source="SYNTHETIC_FIXTURE")
        row = RealEpisodeRecorder(config).combine(camera, state, planner, executed)
        self.assertTrue(row["valid"])
        self.assertTrue(row["executed_command_observation"]["observation_only"])
        self.assertIsNone(row["openvla_executed_action"])
        self.assertIsNone(row["oft_executed_action"])
        other = dict(state, clock_domain="monotonic")
        row = RealEpisodeRecorder(config).combine(camera, other, planner)
        self.assertFalse(row["clock_domain_comparable"])
        self.assertIsNone(row["alignment_deltas"])

    def test_real_wrappers_have_no_ros_command_primitives(self):
        for name in ("real_runtime_adapter.py", "real_episode_recorder.py"):
            source = (ROOT / "06_shadow_collection" / name).read_text(encoding="utf-8")
            for forbidden in ("import rclpy", "create_publisher", "create_client", "ActionClient", "call_async"):
                self.assertNotIn(forbidden, source)

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
