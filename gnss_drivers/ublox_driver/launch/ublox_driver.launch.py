from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config_path_arg = DeclareLaunchArgument(
        "config_path",
        default_value=PathJoinSubstitution(
            [FindPackageShare("ublox_driver"), "config", "driver_config.yaml"]
        ),
        description="Path to the ublox driver YAML configuration file.",
    )

    ublox_config_path_arg = DeclareLaunchArgument(
        "ublox_config_path",
        default_value=PathJoinSubstitution(
            [FindPackageShare("ublox_driver"), "config", "ucenter_config_f9p_gvins.txt"]
        ),
        description="Path to the u-blox receiver config file.",
    )

    ublox_driver_node = Node(
        package="ublox_driver",
        executable="ublox_driver",
        name="ublox_driver",
        output="screen",
        arguments=[
            "--config-path",
            LaunchConfiguration("config_path"),
            "--ublox-config-path",
            LaunchConfiguration("ublox_config_path"),
        ],
    )

    return LaunchDescription([
        config_path_arg,
        ublox_config_path_arg,
        ublox_driver_node,
    ])
