from launch import LaunchDescription
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Default keeps OBot-1 behaviour; OBot-2 passes twist_mux_obot2.yaml.
    twist_mux_config = LaunchConfiguration("twist_mux_config")
    twist_mux_config_arg = DeclareLaunchArgument(
        "twist_mux_config",
        default_value="twist_mux.yaml",
        description="twist_mux YAML file name inside obot_bringup/config",
    )

    package_share = FindPackageShare("obot_description")
    bringup_share = FindPackageShare("obot_bringup")

    xacro_file = PathJoinSubstitution(
        [package_share, "urdf", "obot.urdf.xacro"]
    )

    controllers_file = PathJoinSubstitution(
        [package_share, "config", "obot_controllers.yaml"]
    )

    robot_description = {
        "robot_description": ParameterValue(
            Command(["xacro ", xacro_file]),
            value_type=str,
        )
    }

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[robot_description],
    )

    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        name="controller_manager",
        output="screen",
        parameters=[robot_description, controllers_file],
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
        ],
        output="screen",
    )

    diff_drive_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "diff_drive_controller",
            "--controller-manager",
            "/controller_manager",
        ],
        output="screen",
    )

    cameras = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                bringup_share,
                "launch",
                "obot_cameras.launch.py",
            ])
        )
    )

    lidar = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                bringup_share,
                "launch",
                "obot_lidar.launch.py",
            ])
        )
    )

    twist_mux = Node(
        package="twist_mux",
        executable="twist_mux",
        name="twist_mux",
        output="screen",
        parameters=[
            PathJoinSubstitution([
                bringup_share,
                "config",
                twist_mux_config,
            ])
        ],
        remappings=[
            ("cmd_vel_out", "/diff_drive_controller/cmd_vel"),
        ],
    )

    return LaunchDescription([
        twist_mux_config_arg,
        robot_state_publisher,
        controller_manager,
        joint_state_broadcaster_spawner,
        diff_drive_controller_spawner,
        cameras,
        lidar,
        twist_mux,
    ])
