from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("log_dir", default_value="~/obot_logs"),
        DeclareLaunchArgument("can_interface", default_value="can0"),
        DeclareLaunchArgument("wifi_interface", default_value="wlan0"),
        Node(
            package="obot",
            executable="obot_status_node",
            name="obot_status_node",
            output="screen",
            parameters=[{
                "log_dir": LaunchConfiguration("log_dir"),
                "can_interface": LaunchConfiguration("can_interface"),
                "wifi_interface": LaunchConfiguration("wifi_interface"),
            }],
        ),
    ])
