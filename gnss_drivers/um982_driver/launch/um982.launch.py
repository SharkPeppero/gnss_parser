from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config_file = LaunchConfiguration('config_file')

    return LaunchDescription([
        DeclareLaunchArgument(
            'config_file',
            default_value=PathJoinSubstitution([
                FindPackageShare('um982_driver'),
                'config',
                'serial_setting.yaml',
            ]),
            description='Path to the UM982 serial and NTRIP configuration file',
        ),
        Node(
            package='um982_driver',
            executable='um982_node',
            name='um982_driver',
            output='screen',
            parameters=[{
                'config_file': ParameterValue(config_file, value_type=str),
            }],
        ),
    ])
