from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    joy_topic = LaunchConfiguration("joy_topic")
    cmd_vel_joy = LaunchConfiguration("cmd_vel_joy")
    left_axis = LaunchConfiguration("left_axis")
    right_axis = LaunchConfiguration("right_axis")
    invert_left = LaunchConfiguration("invert_left")
    invert_right = LaunchConfiguration("invert_right")
    deadband = LaunchConfiguration("deadband")
    track_width = LaunchConfiguration("track_width")
    max_linear = LaunchConfiguration("max_linear")
    joy_dev = LaunchConfiguration("joy_dev")

    enable_button = LaunchConfiguration("enable_button")
    enable_axis = LaunchConfiguration("enable_axis")
    enable_axis_threshold = LaunchConfiguration("enable_axis_threshold")
    require_enable = LaunchConfiguration("require_enable")

    fast_mode_button = LaunchConfiguration("fast_mode_button")
    low_speed_scale = LaunchConfiguration("low_speed_scale")
    slow_mode_scale = LaunchConfiguration("slow_mode_scale")
    single_stick_low_speed_scale = LaunchConfiguration(
        "single_stick_low_speed_scale"
    )

    return LaunchDescription([
        # PS4 / joystick input
        DeclareLaunchArgument("joy_topic", default_value="/joy"),
        DeclareLaunchArgument("joy_dev", default_value="/dev/input/js0"),

        # Intermediate Twist topic from the generic SprayBot tank mixer.
        DeclareLaunchArgument("cmd_vel_joy", default_value="/cmd_vel_joy"),

        # Tank-stick mapping
        DeclareLaunchArgument("left_axis", default_value="1"),
        DeclareLaunchArgument("right_axis", default_value="4"),
        DeclareLaunchArgument("invert_left", default_value="false"),
        DeclareLaunchArgument("invert_right", default_value="false"),
        DeclareLaunchArgument("deadband", default_value="0.20"),

        # OBot calibrated drivetrain geometry.
        DeclareLaunchArgument("track_width", default_value="0.486"),

        # Conservative initial teleop limit. Raise later after testing.
        DeclareLaunchArgument("max_linear", default_value="1.5"),

        # Deadman / enable controls
        DeclareLaunchArgument("enable_button", default_value="4"),
        DeclareLaunchArgument("enable_axis", default_value="-1"),
        DeclareLaunchArgument("enable_axis_threshold", default_value="0.5"),
        DeclareLaunchArgument("require_enable", default_value="true"),

        # Speed-mode controls
        DeclareLaunchArgument("fast_mode_button", default_value="5"),
        DeclareLaunchArgument("low_speed_scale", default_value="0.50"),
        DeclareLaunchArgument("slow_mode_scale", default_value="0.50"),
        DeclareLaunchArgument(
            "single_stick_low_speed_scale",
            default_value="0.50",
        ),

        # PS4 joystick driver
        Node(
            package="joy",
            executable="joy_node",
            name="joy_node",
            output="screen",
            parameters=[{
                "dev": joy_dev,
                "deadzone": 0.0,
                "autorepeat_rate": 20.0,
            }],
            remappings=[
                ("/joy", joy_topic),
            ],
        ),

        # Existing robot-independent SprayBot tank mixer.
        # Publishes geometry_msgs/msg/TwistStamped on /cmd_vel_teleop.
        Node(
            package="obot",
            executable="tank_mixer_node",
            name="tank_mixer_node",
            output="screen",
            parameters=[{
                "joy_topic": joy_topic,
                "cmd_vel_out": "/cmd_vel_teleop",
                "left_axis": left_axis,
                "right_axis": right_axis,
                "invert_left": invert_left,
                "invert_right": invert_right,
                "deadband": deadband,
                "track_width": track_width,
                "max_linear": max_linear,
                "joy_timeout_sec": 0.5,

                "enable_button": enable_button,
                "enable_axis": enable_axis,
                "enable_axis_threshold": enable_axis_threshold,
                "require_enable": require_enable,

                "fast_mode_button": fast_mode_button,
                "low_speed_scale": low_speed_scale,
                "slow_mode_scale": slow_mode_scale,
                "single_stick_low_speed_scale": single_stick_low_speed_scale,
            }],
        ),

    ])
