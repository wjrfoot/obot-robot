from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    common_parameters = {
        "image_width": 320,
        "image_height": 240,
        "framerate": 15.0,
        "pixel_format": "mjpeg2rgb",
        "io_method": "mmap",
    }

    rear_camera = Node(
        package="usb_cam",
        executable="usb_cam_node_exe",
        name="usb_cam",
        namespace="rear_camera",
        output="screen",
        parameters=[{
            **common_parameters,
            "video_device": "/dev/video0",
            "camera_name": "rear_camera",
        }],
    )

    front_camera = Node(
        package="usb_cam",
        executable="usb_cam_node_exe",
        name="usb_cam",
        namespace="front_camera",
        output="screen",
        parameters=[{
            **common_parameters,
            "video_device": "/dev/video2",
            "camera_name": "front_camera",
        }],
    )

    return LaunchDescription([
        rear_camera,
        front_camera,
    ])
