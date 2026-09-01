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
        Node(
            package='obot',
            executable='wifi_area_surveyor',
            name='wifi_area_surveyor',
            output='screen',
            parameters=[{
                'map_topic': '/map',
                'map_frame': 'map',
                'robot_base_frame': 'base_link',
                'nav_action_name': '/navigate_to_pose',
                'clicked_point_topic': '/clicked_point',
                'boundary_marker_topic': '/wifi/survey_boundary',
                'use_interactive_bounds': True,
                'use_polygon_bounds': True,
                'polygon_close_distance_m': 0.5,
                'stride_cells': 8,
                'free_threshold': 20,
                'goal_tolerance_m': 0.35,
                'min_x': float('nan'),
                'max_x': float('nan'),
                'min_y': float('nan'),
                'max_y': float('nan'),
            }],
        ),
    ])
