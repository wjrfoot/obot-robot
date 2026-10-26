"""Launch the robot-side valve server."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Return the launch description for the valve action server."""
    config = PathJoinSubstitution([
        FindPackageShare('obot_valve_controller'),
        'config',
        'valve_controller.yaml',
    ])

    return LaunchDescription([
        DeclareLaunchArgument('mock_gpio', default_value='false'),
        Node(
            package='obot_valve_controller',
            executable='valve_server',
            name='valve_server',
            output='screen',
            parameters=[
                config,
                {
                    'mock_gpio': ParameterValue(
                        LaunchConfiguration('mock_gpio'),
                        value_type=bool,
                    )
                },
            ],
        ),
    ])
