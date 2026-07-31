from launch import LaunchDescription
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    config = os.path.join(
        get_package_share_directory("vla_data_collector"),
        "config",
        "data_collection.yaml",
    )

    return LaunchDescription([
        Node(
            package="vla_data_collector",
            executable="handguide_recorder_lhj",
            namespace="dsr01",
            parameters=[config],
            output="screen",
        ),
    ])
