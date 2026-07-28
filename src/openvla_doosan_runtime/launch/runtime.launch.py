import os
import sys

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config = LaunchConfiguration("config")
    inference_python = LaunchConfiguration("inference_python")
    default_inference_python = os.environ.get("OPENVLA_PYTHON", "")
    if not default_inference_python:
        default_inference_python = os.path.join(
            os.environ.get("CONDA_PREFIX", ""), "bin", "python3"
        )
    if not os.path.exists(default_inference_python):
        default_inference_python = sys.executable

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "config",
                default_value=PathJoinSubstitution(
                    [
                        FindPackageShare("openvla_doosan_runtime"),
                        "config",
                        "runtime.yaml",
                    ]
                ),
                description="Absolute path to runtime.yaml",
            ),
            DeclareLaunchArgument(
                "inference_python",
                default_value=default_inference_python,
                description="Python executable used for the OpenVLA inference node",
            ),
            Node(
                package="openvla_doosan_runtime",
                executable="camera_adapter",
                name="camera_adapter",
                output="screen",
                parameters=[config],
            ),
            Node(
                package="openvla_doosan_runtime",
                executable="openvla_inference",
                name="openvla_inference",
                output="screen",
                prefix=inference_python,
                parameters=[config],
            ),
            Node(
                package="openvla_doosan_runtime",
                executable="action_adapter",
                name="action_adapter",
                output="screen",
                parameters=[config],
            ),
            Node(
                package="openvla_doosan_runtime",
                executable="doosan_bridge",
                name="doosan_bridge",
                output="screen",
                parameters=[config],
            ),
            Node(
                package="openvla_doosan_runtime",
                executable="episode_manager",
                name="episode_manager",
                output="screen",
                parameters=[config],
            ),
        ]
    )
