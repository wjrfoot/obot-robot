"""Launch the command-line motion tester and its RViz view."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Launch the motion tester with RViz visualization."""
    rviz_config = PathJoinSubstitution([
        FindPackageShare('obot'),
        'rviz',
        'obot_cmd_motion.rviz',
    ])

    return LaunchDescription([
        DeclareLaunchArgument('linear_speed', default_value='0.20'),
        DeclareLaunchArgument('angular_speed', default_value='0.35'),
        Node(
            package='obot',
            executable='obot_cmd_motion',
            name='obot_cmd_motion',
            output='screen',
            emulate_tty=True,
            parameters=[{
                'linear_speed': LaunchConfiguration('linear_speed'),
                'angular_speed': LaunchConfiguration('angular_speed'),
            }],
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='obot_cmd_motion_rviz',
            output='screen',
            arguments=['-d', rviz_config],
        ),
    ])
