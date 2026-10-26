"""Launch the PS4 action client without starting another joy_node."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Return the launch description for the configurable Joy client."""
    config = PathJoinSubstitution([
        FindPackageShare('obot_valve_controller'),
        'config',
        'valve_controller.yaml',
    ])

    return LaunchDescription([
        DeclareLaunchArgument('circle_button', default_value='-1'),
        DeclareLaunchArgument('square_button', default_value='-1'),
        DeclareLaunchArgument('triangle_button', default_value='-1'),
        DeclareLaunchArgument('cross_button', default_value='-1'),
        Node(
            package='obot_valve_controller',
            executable='ps4_valve_client',
            name='ps4_valve_client',
            output='screen',
            parameters=[
                config,
                {
                    'circle_button': ParameterValue(
                        LaunchConfiguration('circle_button'),
                        value_type=int,
                    ),
                    'square_button': ParameterValue(
                        LaunchConfiguration('square_button'),
                        value_type=int,
                    ),
                    'triangle_button': ParameterValue(
                        LaunchConfiguration('triangle_button'),
                        value_type=int,
                    ),
                    'cross_button': ParameterValue(
                        LaunchConfiguration('cross_button'),
                        value_type=int,
                    ),
                },
            ],
        ),
    ])
