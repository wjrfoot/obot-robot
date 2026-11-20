"""Record bounded wheel-odometry poses for RViz visualization."""

from collections import deque
from copy import deepcopy
import math

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy,
)
from std_srvs.srv import Trigger


BACKWARD_JUMP_NS = 1_000_000_000


def pose_yaw(pose):
    q = pose.orientation
    norm = math.sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w)
    if not math.isfinite(norm) or norm < 1e-9:
        raise ValueError('invalid orientation')
    x, y, z, w = q.x / norm, q.y / norm, q.z / norm, q.w / norm
    return math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))


class PathHistory:
    def __init__(self, distance, angle, max_points):
        if (not math.isfinite(distance) or distance < 0
                or not math.isfinite(angle) or angle < 0
                or isinstance(max_points, bool) or not isinstance(max_points, int)
                or max_points < 1):
            raise ValueError('Invalid path sampling parameters')
        self.distance = distance
        self.angle = angle
        self.points = deque(maxlen=max_points)
        self.frame = None
        self.last_stamp = None
        self.last_yaw = None

    def clear(self):
        self.points.clear()
        self.last_stamp = None
        self.last_yaw = None

    def add(self, msg):
        frame = msg.header.frame_id.strip()
        p = msg.pose.pose.position
        if not frame or not all(math.isfinite(v) for v in (p.x, p.y, p.z)):
            return False
        try:
            yaw = pose_yaw(msg.pose.pose)
        except ValueError:
            return False
        stamp = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
        if self.frame != frame or (
                self.last_stamp is not None
                and stamp < self.last_stamp - BACKWARD_JUMP_NS):
            self.clear()
        self.frame = frame
        self.last_stamp = stamp
        if self.points:
            previous = self.points[-1].pose.position
            distance = math.hypot(p.x - previous.x, p.y - previous.y)
            delta = yaw - self.last_yaw
            angle = abs(math.atan2(math.sin(delta), math.cos(delta)))
            if distance <= self.distance and angle <= self.angle:
                return False
        point = PoseStamped()
        point.header = deepcopy(msg.header)
        point.header.frame_id = frame
        point.pose = deepcopy(msg.pose.pose)
        self.points.append(point)
        self.last_yaw = yaw
        return True


class PathRecorder(Node):
    def __init__(self, **kwargs):
        super().__init__('obot_path_recorder', **kwargs)
        defaults = {
            'odom_topic': '/diff_drive_controller/odom',
            'path_topic': '/obot/path',
            'minimum_distance_m': 0.05,
            'minimum_angle_rad': 0.15,
            'max_points': 10000,
            'publish_rate_hz': 2.0,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        p = {name: self.get_parameter(name).value for name in defaults}
        rate = p['publish_rate_hz']
        if not math.isfinite(rate) or rate <= 0:
            raise ValueError('publish_rate_hz must be positive and finite')
        self.history = PathHistory(
            p['minimum_distance_m'], p['minimum_angle_rad'], p['max_points'])
        path_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST, depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL)
        odom_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST, depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT)
        self.publisher = self.create_publisher(Path, p['path_topic'], path_qos)
        self.subscription = self.create_subscription(
            Odometry, p['odom_topic'], self._on_odom, odom_qos)
        self.service = self.create_service(Trigger, '/obot/clear_path', self._clear)
        self.timer = self.create_timer(1 / rate, self._publish)
        self._publish()

    def _on_odom(self, msg):
        self.history.add(msg)

    def _publish(self):
        path = Path()
        path.header.frame_id = self.history.frame or 'odom'
        path.header.stamp = self.get_clock().now().to_msg()
        path.poses = list(self.history.points)
        self.publisher.publish(path)

    def _clear(self, request, response):
        self.history.clear()
        self._publish()
        response.success = True
        response.message = 'Recorded path cleared'
        return response


def main(args=None):
    rclpy.init(args=args)
    node = PathRecorder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
