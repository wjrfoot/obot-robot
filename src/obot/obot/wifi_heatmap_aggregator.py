#!/usr/bin/env python3
from dataclasses import dataclass
from typing import Dict, Tuple

from obot_msgs.msg import WifiSample
import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray


@dataclass
class CellStats:
    gx: int
    gy: int
    x: float
    y: float
    count: int
    avg_rssi_dbm: float


class WifiHeatmapAggregator(Node):
    def __init__(self) -> None:
        super().__init__('wifi_heatmap_aggregator')

        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('topic_sample', '/wifi/sample')
        self.declare_parameter('topic_markers', '/wifi/heatmap_markers')
        self.declare_parameter('cell_size_m', 0.5)
        self.declare_parameter('min_samples_per_cell', 1)
        self.declare_parameter('publish_rate_hz', 1.0)

        # Thresholds based on practical Wi-Fi reliability ranges.
        # Better than -60 dBm is typically very stable (green).
        # Around -75 dBm starts to become marginal (orange/red).
        self.declare_parameter('good_rssi_dbm', -60.0)
        self.declare_parameter('marginal_rssi_dbm', -75.0)

        self.map_frame = str(self.get_parameter('map_frame').value)
        self.topic_sample = str(self.get_parameter('topic_sample').value)
        self.topic_markers = str(self.get_parameter('topic_markers').value)
        self.cell_size_m = float(self.get_parameter('cell_size_m').value)
        self.min_samples_per_cell = int(self.get_parameter('min_samples_per_cell').value)
        publish_rate_hz = float(self.get_parameter('publish_rate_hz').value)
        self.good_rssi = float(self.get_parameter('good_rssi_dbm').value)
        self.marginal_rssi = float(self.get_parameter('marginal_rssi_dbm').value)

        if self.good_rssi <= self.marginal_rssi:
            self.get_logger().warn(
                'good_rssi_dbm should be greater than marginal_rssi_dbm. Resetting defaults.'
            )
            self.good_rssi = -60.0
            self.marginal_rssi = -75.0

        self.grid: Dict[Tuple[int, int], CellStats] = {}

        self.create_subscription(WifiSample, self.topic_sample, self._sample_cb, 50)
        self.marker_pub = self.create_publisher(MarkerArray, self.topic_markers, 10)

        timer_period = 1.0 / max(0.1, publish_rate_hz)
        self.create_timer(timer_period, self._publish_markers)

        self.get_logger().info(
            f'Wi-Fi heatmap aggregator running: cell_size={self.cell_size_m:.2f}m '
            f'min_samples={self.min_samples_per_cell} '
            f'good={self.good_rssi:.1f}dBm marginal={self.marginal_rssi:.1f}dBm'
        )

    def _sample_cb(self, msg: WifiSample) -> None:
        x = msg.position.x
        y = msg.position.y
        rssi = float(msg.rssi_dbm)

        gx = int(round(x / self.cell_size_m))
        gy = int(round(y / self.cell_size_m))
        key = (gx, gy)

        if key not in self.grid:
            self.grid[key] = CellStats(
                gx=gx,
                gy=gy,
                x=gx * self.cell_size_m,
                y=gy * self.cell_size_m,
                count=1,
                avg_rssi_dbm=rssi,
            )
            return

        cell = self.grid[key]
        new_count = cell.count + 1
        new_avg = (cell.avg_rssi_dbm * cell.count + rssi) / new_count
        cell.count = new_count
        cell.avg_rssi_dbm = new_avg

    def _normalize(self, rssi_dbm: float) -> float:
        # 1.0 = good (green), 0.0 = marginal/poor (red)
        if rssi_dbm >= self.good_rssi:
            return 1.0
        if rssi_dbm <= self.marginal_rssi:
            return 0.0
        return (rssi_dbm - self.marginal_rssi) / (self.good_rssi - self.marginal_rssi)

    def _publish_markers(self) -> None:
        marker_array = MarkerArray()

        clear_marker = Marker()
        clear_marker.header.frame_id = self.map_frame
        clear_marker.header.stamp = self.get_clock().now().to_msg()
        clear_marker.ns = 'wifi_heatmap'
        clear_marker.id = 0
        clear_marker.action = Marker.DELETEALL
        marker_array.markers.append(clear_marker)

        for cell in self.grid.values():
            if cell.count < max(1, self.min_samples_per_cell):
                continue

            score = self._normalize(cell.avg_rssi_dbm)

            # Interpolate red->green with fixed blue=0.
            red = 1.0 - score
            green = score

            marker = Marker()
            marker.header.frame_id = self.map_frame
            marker.header.stamp = clear_marker.header.stamp
            marker.ns = 'wifi_heatmap'
            marker.id = self._marker_id_for_cell(cell.gx, cell.gy)
            marker.type = Marker.CUBE
            marker.action = Marker.ADD

            marker.pose.position.x = cell.x
            marker.pose.position.y = cell.y
            marker.pose.position.z = 0.05
            marker.pose.orientation.w = 1.0

            marker.scale.x = self.cell_size_m
            marker.scale.y = self.cell_size_m
            marker.scale.z = 0.1

            marker.color.a = 0.65
            marker.color.r = red
            marker.color.g = green
            marker.color.b = 0.0

            marker_array.markers.append(marker)

        self.marker_pub.publish(marker_array)

    @staticmethod
    def _zigzag(n: int) -> int:
        return 2 * n if n >= 0 else -2 * n - 1

    def _marker_id_for_cell(self, gx: int, gy: int) -> int:
        # Cantor pairing over zigzag-mapped coordinates gives deterministic IDs by grid cell.
        a = self._zigzag(gx)
        b = self._zigzag(gy)
        pair = (a + b) * (a + b + 1) // 2 + b
        return int(pair % 2147483646) + 1


def main(args=None) -> None:
    rclpy.init(args=args)
    node = WifiHeatmapAggregator()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
