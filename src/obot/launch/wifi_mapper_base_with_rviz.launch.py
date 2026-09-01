from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    rviz_config = PathJoinSubstitution([
        FindPackageShare('obot'),
        'rviz',
        'wifi_heatmap.rviz',
    ])

    return LaunchDescription([
        Node(
            package='obot',
            executable='wifi_heatmap_aggregator',
            name='wifi_heatmap_aggregator',
            output='screen',
            parameters=[{
                'map_frame': 'map',
                'topic_sample': '/wifi/sample',
                'topic_markers': '/wifi/heatmap_markers',
                'cell_size_m': 0.5,
                'min_samples_per_cell': 1,
                'publish_rate_hz': 1.0,
                'good_rssi_dbm': -60.0,
                'marginal_rssi_dbm': -75.0,
            }],
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='wifi_heatmap_rviz',
            output='screen',
            arguments=['-d', rviz_config],
        ),
    ])
