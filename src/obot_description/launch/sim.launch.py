from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
    UnsetEnvironmentVariable,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    world = LaunchConfiguration("world")
    map_name = LaunchConfiguration("map")
    headless = LaunchConfiguration("headless")
    xacro_file = PathJoinSubstitution([
        FindPackageShare("obot_description"), "urdf", "obot.urdf.xacro"
    ])
    robot_description = {
        "robot_description": ParameterValue(
            Command(["xacro ", xacro_file, " use_sim:=true"]),
            value_type=str,
        )
    }

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare("ros_gz_sim"), "launch", "gz_sim.launch.py"
        ])),
        launch_arguments={"gz_args": ["-r ", headless, " ", world]}.items(),
    )

    return LaunchDescription([
        UnsetEnvironmentVariable("CYCLONEDDS_URI"),
        SetEnvironmentVariable(
            name="LD_PRELOAD",
            value="/lib/x86_64-linux-gnu/libpthread.so.0",
        ),
        DeclareLaunchArgument(
            "map",
            default_value="map.sdf",
            description="Generated map world filename: map.sdf, map2.sdf, or map3.sdf",
        ),
        DeclareLaunchArgument(
            "world",
            default_value=PathJoinSubstitution([
                FindPackageShare("obot_description"), "worlds", map_name
            ]),
            description="Gazebo Sim world file; overrides map",
        ),
        DeclareLaunchArgument(
            "headless",
            default_value="",
            description="Set to -s to run Gazebo without its GUI",
        ),
        gazebo,
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            output="screen",
            parameters=[robot_description, {"use_sim_time": True}],
        ),
        Node(
            package="ros_gz_sim",
            executable="create",
            arguments=["-topic", "robot_description", "-name", "obot", "-z", "0.2"],
            output="screen",
        ),
        Node(
            package="ros_gz_bridge",
            executable="parameter_bridge",
            arguments=[
                "/model/obot/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist",
                "/model/obot/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry",
                "/model/obot/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V",
                "/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan",
            ],
            parameters=[{"use_sim_time": True}],
            output="screen",
        ),
    ])