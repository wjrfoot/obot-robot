from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='obot',
            executable='wifi_signal_sampler',
            name='wifi_signal_sampler',
            output='screen',
            parameters=[{
                'sample_rate_hz': 2.0,
                'wifi_interface': 'wlan0',
                'robot_base_frame': 'base_link',
                'map_frame': 'map',
                'topic_sample': '/wifi/sample',
            }],
        ),
    ])
