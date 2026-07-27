\
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    config = LaunchConfiguration("config")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "config",
                default_value="",
                description="Absolute path to runtime.yaml",
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
