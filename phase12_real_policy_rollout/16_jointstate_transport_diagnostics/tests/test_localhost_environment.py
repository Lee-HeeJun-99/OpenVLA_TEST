"""Validate local-only entry paths without creating any ROS participants."""
import ast
import os
from pathlib import Path
import unittest
from unittest.mock import patch

REPO = Path(__file__).resolve().parents[3]
WS = Path('/home/ubuntu/robot_ws')


class LocalhostEnvironmentTests(unittest.TestCase):
    def test_python_entrypoints_set_environment_before_init(self):
        directories = [REPO / 'phase12_real_policy_rollout' / name for name in
                       ('03_shadow_mode', '14_real_rollout_integration',
                        '15_valid_invalid_rollout_policy', '16_jointstate_transport_diagnostics')]
        directories += [WS / 'src/openvla_doosan_runtime/openvla_doosan_runtime',
                        WS / 'install/openvla_doosan_runtime/lib/python3.10/site-packages/openvla_doosan_runtime']
        checked = 0
        for directory in directories:
            for path in directory.glob('*.py'):
                source = path.read_text()
                ast.parse(source, filename=str(path))
                for line in source.splitlines():
                    if 'rclpy.init(' in line:
                        with self.subTest(path=path):
                            prefix = line.split('rclpy.init(', 1)[0]
                            self.assertIn('__import__("os").environ["ROS_LOCALHOST_ONLY"] = "1";', prefix)
                            assignment = prefix[prefix.index('__import__'):].strip().rstrip(';')
                            with patch.dict(os.environ, {'ROS_LOCALHOST_ONLY': '0', 'ROS_DOMAIN_ID': '57', 'rt_host': 'unchanged'}):
                                exec(assignment, {})
                                self.assertEqual(os.environ['ROS_LOCALHOST_ONLY'], '1')
                                self.assertEqual(os.environ['ROS_DOMAIN_ID'], '57')
                                self.assertEqual(os.environ['rt_host'], 'unchanged')
                        checked += 1
        self.assertGreaterEqual(checked, 30)

    def test_launch_environment_action_precedes_nodes(self):
        paths = [WS / 'src/doosan-robot2/dsr_bringup2/launch/dsr_bringup2_rviz.launch.py',
                 WS / 'install/dsr_bringup2/share/dsr_bringup2/launch/dsr_bringup2_rviz.launch.py',
                 WS / 'src/zed-ros2-wrapper/zed_wrapper/launch/zed_camera.launch.py',
                 REPO / 'phase12_real_policy_rollout/03_shadow_mode/jointstate_observer_minimal.launch.py']
        for directory in ('src/openvla_doosan_runtime/launch', 'install/openvla_doosan_runtime/share/openvla_doosan_runtime/launch'):
            paths += [WS / directory / name for name in ('runtime.launch.py', 'dataset_replay.launch.py', 'steamvr_teleop.launch.py')]
        for path in paths:
            with self.subTest(path=path):
                tree = ast.parse(path.read_text())
                environment_calls = [node for node in ast.walk(tree) if isinstance(node, ast.Call)
                                     and isinstance(node.func, ast.Name) and node.func.id == 'SetEnvironmentVariable']
                self.assertEqual(len(environment_calls), 1)
                calls = [node for node in ast.walk(tree) if isinstance(node, ast.Call)
                         and isinstance(node.func, ast.Name) and node.func.id == 'LaunchDescription']
                self.assertTrue(calls)
                for call in calls:
                    expression = call.args[0]
                    while isinstance(expression, ast.BinOp):
                        expression = expression.left
                    first = expression.elts[0]
                    self.assertEqual(first.func.id, 'SetEnvironmentVariable')
                    self.assertEqual([arg.value for arg in first.args], ['ROS_LOCALHOST_ONLY', '1'])

    def test_shell_export_precedes_ros_launch(self):
        source = (WS / 'src/openvla_doosan_runtime/scripts/run_oft_runtime.sh').read_text()
        self.assertLess(source.index('export ROS_LOCALHOST_ONLY=1'), source.index('exec ros2 launch'))


if __name__ == '__main__':
    unittest.main()
