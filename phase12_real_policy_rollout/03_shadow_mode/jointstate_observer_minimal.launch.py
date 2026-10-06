"""Phase 12 minimal Doosan JointState observer launch candidate.

This deliberately excludes dsr_controller2 and every application command node.
It is NOT intrinsically read-only: loading dsr_hardware2 in real mode currently
calls CONTROL_SERVO_ON from DRHWInterface::on_init().  For that reason the
launch refuses to start unless a local operator explicitly sets
operator_approved_hardware_initialization:=true.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _validated_nodes(context):
    approved = LaunchConfiguration("operator_approved_hardware_initialization").perform(context)
    if approved.strip().lower() not in {"true", "1", "yes"}:
        raise RuntimeError(
            "Refusing hardware initialization: dsr_hardware2 real-mode on_init() "
            "calls CONTROL_SERVO_ON. A local operator must explicitly approve it."
        )

    name = LaunchConfiguration("name")
    update_rate = LaunchConfiguration("update_rate")
    robot_description = {
        "robot_description": Command(
            [
                PathJoinSubstitution([FindExecutable(name="xacro")]),
                " ",
                PathJoinSubstitution(
                    [FindPackageShare("dsr_description2"), "xacro", LaunchConfiguration("model")]
                ),
                ".urdf.xacro",
                " name:=", name,
                " host:=", LaunchConfiguration("host"),
                " rt_host:=", LaunchConfiguration("rt_host"),
                " port:=", LaunchConfiguration("port"),
                " mode:=real",
                " model:=", LaunchConfiguration("model"),
                " update_rate:=", update_rate,
            ]
        )
    }

    controller_config = PathJoinSubstitution(
        [FindPackageShare("dsr_controller2"), "config", "dsr_controller2.yaml"]
    )

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        namespace=name,
        parameters=[robot_description, controller_config],
        output="both",
    )
    joint_state_spawner = Node(
        package="controller_manager",
        executable="spawner",
        namespace=name,
        arguments=[
            "joint_state_broadcaster",
            "-c", "controller_manager",
            "--controller-manager-timeout", "30",
            "--service-call-timeout", "30",
        ],
        output="both",
    )
    return [control_node, joint_state_spawner]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("name", default_value="dsr01"),
            DeclareLaunchArgument("host", default_value="192.168.0.110"),
            DeclareLaunchArgument("rt_host", default_value="192.168.137.50"),
            DeclareLaunchArgument("port", default_value="12345"),
            DeclareLaunchArgument("model", default_value="a0509"),
            DeclareLaunchArgument("update_rate", default_value="100"),
            DeclareLaunchArgument(
                "operator_approved_hardware_initialization",
                default_value="false",
                description="Required because dsr_hardware2 real on_init calls CONTROL_SERVO_ON",
            ),
            OpaqueFunction(function=_validated_nodes),
        ]
    )

