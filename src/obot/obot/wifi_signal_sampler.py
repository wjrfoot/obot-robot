#!/usr/bin/env python3
import math
import re
import subprocess
from typing import Optional, Tuple

from obot_msgs.msg import WifiSample
import rclpy
from rclpy.node import Node
from tf2_ros import Buffer, TransformException, TransformListener


class WifiSignalSampler(Node):
    def __init__(self) -> None:
        super().__init__('wifi_signal_sampler')

        self.declare_parameter('sample_rate_hz', 2.0)
        self.declare_parameter('wifi_interface', 'wlan0')
        self.declare_parameter('robot_base_frame', 'base_link')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('topic_sample', '/wifi/sample')

        sample_rate_hz = float(self.get_parameter('sample_rate_hz').value)
        self.wifi_interface = str(self.get_parameter('wifi_interface').value)
        self.robot_base_frame = str(self.get_parameter('robot_base_frame').value)
        self.map_frame = str(self.get_parameter('map_frame').value)
        topic_sample = str(self.get_parameter('topic_sample').value)

        self.sample_pub = self.create_publisher(WifiSample, topic_sample, 20)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        timer_period = 1.0 / max(0.1, sample_rate_hz)
        self.timer = self.create_timer(timer_period, self._sample_once)

        self.get_logger().info(
            f'Started Wi-Fi sampler on interface {self.wifi_interface} at {sample_rate_hz:.2f} Hz'
        )

    def _sample_once(self) -> None:
        reading = self._read_wifi_link(self.wifi_interface)
        if reading is None:
            self.get_logger().warn('Could not read Wi-Fi RSSI. Is the interface connected?')
            return
        rssi, ssid = reading

        try:
            transform = self.tf_buffer.lookup_transform(
                self.map_frame,
                self.robot_base_frame,
                rclpy.time.Time(),
            )
        except TransformException as exc:
            self.get_logger().warn(f'No TF {self.map_frame}<-{self.robot_base_frame}: {exc}')
            return

        stamp = self.get_clock().now().to_msg()
        rotation = transform.transform.rotation
        yaw = self._quat_to_yaw(rotation.x, rotation.y, rotation.z, rotation.w)

        sample_msg = WifiSample()
        sample_msg.header.stamp = stamp
        sample_msg.header.frame_id = self.map_frame
        sample_msg.position.x = transform.transform.translation.x
        sample_msg.position.y = transform.transform.translation.y
        sample_msg.position.z = transform.transform.translation.z
        sample_msg.yaw = float(yaw)
        sample_msg.rssi_dbm = int(rssi)
        sample_msg.interface = self.wifi_interface
        sample_msg.ssid = ssid

        self.sample_pub.publish(sample_msg)

    @staticmethod
    def _read_wifi_link(interface_name: str) -> Optional[Tuple[int, str]]:
        try:
            cmd = ['iw', 'dev', interface_name, 'link']
            result = subprocess.run(cmd, capture_output=True, text=True, check=False)
            if result.returncode != 0:
                return None

            signal_match = re.search(r'signal:\s*(-?\d+)\s*dBm', result.stdout)
            if not signal_match:
                return None

            ssid_match = re.search(r'SSID:\s*(.+)$', result.stdout, re.MULTILINE)
            ssid = ssid_match.group(1).strip() if ssid_match else ''
            return int(signal_match.group(1)), ssid
        except FileNotFoundError:
            return None

    @staticmethod
    def _quat_to_yaw(x: float, y: float, z: float, w: float) -> float:
        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        return math.atan2(siny_cosp, cosy_cosp)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = WifiSignalSampler()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
