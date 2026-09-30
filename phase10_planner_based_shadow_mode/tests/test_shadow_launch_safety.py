from dataclasses import replace
from pathlib import Path
import sys
import unittest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "06_shadow_collection"))
from real_runtime_adapter import RemoteSafetyConfig, load_config  # noqa: E402


class ShadowLaunchSafetySyntheticFixture(unittest.TestCase):
    """SYNTHETIC_FIXTURE static launch/config safety."""
    def test_config_and_all_unsafe_switches(self):
        config, _ = load_config(ROOT / "06_shadow_collection" / "shadow_runtime.yaml")
        config.validate()
        for field in ("allow_robot_command", "allow_gripper_command", "allow_home_command",
                      "allow_trajectory_execution", "allow_service_call", "allow_motion_service",
                      "allow_stop_service", "allow_estop_service", "publish_ai_action"):
            with self.assertRaises(PermissionError):
                replace(config, **{field: True}).validate()

    def test_launch_has_no_nodes_or_command_clients(self):
        source = (ROOT / "06_shadow_collection" / "shadow_runtime.launch.py").read_text()
        for forbidden in ("launch_ros", "Node(", "ExecuteProcess", "doosan_bridge", "action_adapter"):
            self.assertNotIn(forbidden, source)

