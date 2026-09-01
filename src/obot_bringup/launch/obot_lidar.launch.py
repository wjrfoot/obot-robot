from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    sllidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("sllidar_ros2"),
                "launch",
                "sllidar_a2m12_launch.py",
            ])
        ),
        launch_arguments={
            "channel_type": "serial",
            "serial_port": "/dev/ttyUSB0",
            "serial_baudrate": "256000",
            "frame_id": "laser",
            "inverted": "false",
            "angle_compensate": "true",
        }.items(),
    )

    return LaunchDescription([
        sllidar_launch,
    ])
