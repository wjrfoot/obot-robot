#!/usr/bin/env python3
"""Compare commanded motion with OBot wheel odometry using a terminal CLI."""
import codecs
from collections import deque
import errno
import math
import os
import queue
import select
import signal
import sys
import time

from geometry_msgs.msg import PoseStamped, TwistStamped

from nav_msgs.msg import Odometry, Path
from obot_msgs.srv import ClickDrive

import rclpy
from rclpy._rclpy_pybind11 import RCLError
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.signals import SignalHandlerOptions

FEET_TO_METERS = 0.3048
COMMAND_RATE_HZ = 30.0
PATH_PUBLISH_RATE_HZ = 2.0
PATH_DISPLAY_LIMIT = 500
HELP = """OBot motion test commands
  f N   forward N feet          F N   forward N meters
  b N   backward N feet         B N   backward N meters
  r N   clockwise N degrees     l N   counterclockwise N degrees
  s     stop and discard queued movements
  c/z   clear paths and reset test origin (only when stopped)
  p     print commanded/odom positions and errors
  ?     show this help          q     stop and quit
"""


def wrap_angle(angle):
    """Wrap an angle in radians to [-pi, pi]."""
    return math.atan2(math.sin(angle), math.cos(angle))


def command_target(command, magnitude):
    """Convert a positive CLI magnitude to meters or radians."""
    if not math.isfinite(magnitude) or magnitude <= 0:
        raise ValueError('Command magnitude must be positive and finite')
    if command in ('f', 'b'):
        return magnitude * FEET_TO_METERS
    if command in ('F', 'B'):
        return magnitude
    if command in ('r', 'l'):
        return math.radians(magnitude)
    raise ValueError(
        f'Unsupported motion command: {command}')


def rotation_sign(command):
    """Return yaw sign: clockwise is negative, counterclockwise positive."""
    if command == 'r':
        return -1
    if command == 'l':
        return 1
    raise ValueError(f'Not a rotation command: {command}')


def relative_pose(origin, pose):
    """Express an odometry pose in the coordinate system of its test origin."""
    dx, dy = pose[0] - origin[0], pose[1] - origin[1]
    origin_yaw = origin[2]
    return (
        math.cos(origin_yaw) * dx + math.sin(origin_yaw) * dy,
        -math.sin(origin_yaw) * dx + math.cos(origin_yaw) * dy,
        wrap_angle(pose[2] - origin_yaw),
    )


def world_pose(origin, pose):
    """Transform a relative pose into the odometry frame."""
    ox, oy, origin_yaw = origin
    x, y, yaw = pose
    return (
        ox + math.cos(origin_yaw) * x - math.sin(origin_yaw) * y,
        oy + math.sin(origin_yaw) * x + math.cos(origin_yaw) * y,
        wrap_angle(origin_yaw + yaw),
    )


def integrate_pose(pose, linear, angular, duration):
    """Integrate a constant unicycle velocity for duration seconds."""
    if not all(math.isfinite(value) for value in (linear, angular, duration)):
        raise ValueError('Velocity and duration must be finite')
    if duration < 0:
        raise ValueError('Duration cannot be negative')
    x, y, yaw = pose
    delta_yaw = angular * duration
    if abs(angular) < 1e-12:
        x += linear * math.cos(yaw) * duration
        y += linear * math.sin(yaw) * duration
    else:
        radius = linear / angular
        x += radius * (math.sin(yaw + delta_yaw) - math.sin(yaw))
        y -= radius * (math.cos(yaw + delta_yaw) - math.cos(yaw))
    return x, y, wrap_angle(yaw + delta_yaw)


def twist_values(message):
    """Return all components of a stamped velocity command."""
    twist = message.twist
    return (
        twist.linear.x, twist.linear.y, twist.linear.z,
        twist.angular.x, twist.angular.y, twist.angular.z,
    )


class MuxOutputTracker:
    """Track observed mux velocities and feedback freshness."""

    def __init__(self, timeout):
        """Create a tracker with a maximum feedback gap."""
        self.timeout = timeout
        self.last_observation = None
        self.started_at = None
        self.running = False

    def start(self, now):
        """Start tracking a command without counting earlier idle output."""
        self.last_observation = None
        self.started_at = now
        self.running = True

    def stop(self):
        """Stop tracking output for the current command."""
        self.running = False
        self.last_observation = None

    def observe(self, message, now):
        """Return the preceding observed output interval."""
        if not self.running:
            return None
        self.check_timeout(now)
        values = twist_values(message)
        if not all(math.isfinite(value) for value in values):
            raise ValueError('Mux output contains non-finite velocities')
        interval = None
        if self.last_observation is not None:
            previous_time, previous_values = self.last_observation
            duration = now - previous_time
            interval = (previous_values[0], previous_values[5], duration)
        self.last_observation = (now, values)
        return interval

    def check_timeout(self, now):
        """Reject missing or interrupted mux feedback."""
        since = (
            self.last_observation[0]
            if self.last_observation is not None else self.started_at
        )
        if self.running and since is not None and now - since > self.timeout:
            raise ValueError(
                'Mux output feedback timed out: '
                f'elapsed={now - since:.3f}s, '
                f'limit={self.timeout:.3f}s')


class TimingDiagnostics:
    """Collect per-command callback timing using the local monotonic clock."""

    def __init__(self):
        """Initialize timing maxima for one command."""
        self.last_tick = None
        self.last_send = None
        self.max_tick_gap = 0.0
        self.max_send_gap = 0.0
        self.max_mux_gap = 0.0
        self.max_path_duration = 0.0
        self.max_path_points = (0, 0)

    def tick(self, now):
        """Record the time between timer callback starts."""
        if self.last_tick is not None:
            self.max_tick_gap = max(self.max_tick_gap, now - self.last_tick)
        self.last_tick = now

    def mux(self, interval):
        """Record the gap between output callbacks."""
        if interval is not None:
            self.max_mux_gap = max(self.max_mux_gap, interval)

    def send(self, now):
        """Record the gap between test command publications."""
        if self.last_send is not None:
            self.max_send_gap = max(self.max_send_gap, now - self.last_send)
        self.last_send = now

    def path(self, duration, points):
        """Record the most expensive sampled path publication."""
        if duration > self.max_path_duration:
            self.max_path_duration = duration
            self.max_path_points = points

    def summary(self):
        """Return callback timing evidence for diagnostic logs."""
        return (
            f'max timer gap={self.max_tick_gap:.3f}s, '
            f'max mux callback gap={self.max_mux_gap:.3f}s, '
            f'max send gap={self.max_send_gap:.3f}s, '
            f'max path publish={self.max_path_duration:.3f}s '
            f'(cmd,actual history points={self.max_path_points})')


class MotionHistory:
    """Track full-resolution commanded and measured paths from one origin."""

    def __init__(self):
        """Create empty histories."""
        self.origin = None
        self.commanded_pose = (0.0, 0.0, 0.0)
        self.commanded_path = []
        self.actual_path = []

    def clear(self, origin):
        """Clear both paths and establish a new odometry origin."""
        self.origin = tuple(origin)
        self.commanded_pose = (0.0, 0.0, 0.0)
        self.commanded_path.clear()
        self.actual_path.clear()
        self.commanded_path.append(self.commanded_pose)
        self.actual_path.append(self.commanded_pose)

    def add_actual(self, pose):
        """Append a measured odometry pose relative to the current origin."""
        if self.origin is None:
            self.clear(pose)
            return
        self.actual_path.append(relative_pose(self.origin, pose))

    def integrate_command(self, linear, angular, duration):
        """Integrate a mux-accepted command and append its resulting pose."""
        if self.origin is None:
            raise RuntimeError('Motion history has no test origin')
        self.commanded_pose = integrate_pose(
            self.commanded_pose, linear, angular, duration)
        self.commanded_path.append(self.commanded_pose)


def display_points(points, limit):
    """Select evenly spaced history points, including both endpoints."""
    if limit < 2:
        raise ValueError('Display point limit must be at least 2')
    count = len(points)
    if count <= limit:
        return points
    return [points[i * (count - 1) // (limit - 1)] for i in range(limit)]


def yaw_of(quaternion):
    """Extract yaw from a finite, non-zero orientation quaternion."""
    norm = math.sqrt(
        quaternion.x ** 2 + quaternion.y ** 2
        + quaternion.z ** 2 + quaternion.w ** 2)
    if not math.isfinite(norm) or norm < 1e-12:
        raise ValueError('Odometry has an invalid orientation quaternion')
    x, y, z, w = (
        quaternion.x / norm,
        quaternion.y / norm,
        quaternion.z / norm,
        quaternion.w / norm,
    )
    return math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))


class MotionTest(Node):
    """Publish test motion and compare its path against wheel odometry."""

    def __init__(self):
        """Initialize ROS interfaces, parameters, and command state."""
        super().__init__('obot_cmd_motion')
        defaults = (
            ('cmd_topic', '/cmd_vel_test'),
            ('actual_odom_topic', '/diff_drive_controller/odom'),
            ('mux_output_topic', '/diff_drive_controller/cmd_vel'),
            ('linear_speed', 0.20),
            ('angular_speed', 0.35),
            ('odom_timeout_s', 1.0),
            ('mux_output_timeout_s', 0.5),
            ('stop_linear_tolerance', 0.015),
            ('stop_angular_tolerance', 0.025),
            ('stop_hold_s', 0.5),
            ('settle_timeout_s', 8.0),
        )
        for key, value in defaults:
            self.declare_parameter(key, value)

        def param(key):
            return self.get_parameter(key).value

        self.linear_speed = float(param('linear_speed'))
        self.angular_speed = float(param('angular_speed'))
        self.odom_timeout = float(param('odom_timeout_s'))
        self.mux_output_timeout = float(param('mux_output_timeout_s'))
        self.stop_v = float(param('stop_linear_tolerance'))
        self.stop_w = float(param('stop_angular_tolerance'))
        self.stop_hold = float(param('stop_hold_s'))
        self.settle_timeout = float(param('settle_timeout_s'))
        positive = (
            self.linear_speed, self.angular_speed, self.odom_timeout,
            self.mux_output_timeout,
            self.stop_v, self.stop_w, self.stop_hold, self.settle_timeout,
        )
        if not all(math.isfinite(value) and value > 0 for value in positive):
            raise ValueError(
                'Speeds, tolerances, and timeouts must be positive and finite')
        self.cmd_pub = self.create_publisher(
            TwistStamped, str(param('cmd_topic')), 10)
        path_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.cmd_odom_pub = self.create_publisher(Odometry, '/cmd_odom', 10)
        self.cmd_path_pub = self.create_publisher(Path, '/cmd_path', path_qos)
        self.actual_path_pub = self.create_publisher(
            Path, '/actual_path', path_qos)
        self.create_subscription(
            Odometry, str(param('actual_odom_topic')), self._on_odom, 20)
        self.create_subscription(
            TwistStamped, str(param('mux_output_topic')), self._on_mux, 20)

        self.create_subscription(
            TwistStamped, '/cmd_vel_teleop', self._on_manual_override, 20)
        self.last_manual_override = None

        self.history = MotionHistory()
        self.odom_frame = None
        self.odom_pose = None
        self.odom_v = 0.0
        self.odom_w = 0.0
        self.last_odom = None
        self.mux_tracker = MuxOutputTracker(self.mux_output_timeout)
        self.observed_velocity = (0.0, 0.0)
        self.pending = deque()
        self.inputs = queue.Queue()
        self.active = None
        self.progress = 0.0
        self.timing = TimingDiagnostics()
        self.target_reached_at = None
        self.target_reached_progress = None
        self.target_odom_yaw = None
        self.zero_odom_yaw = None
        self.command_success = True
        self.failure_reason = None
        self.state = 'idle'
        self.click_drive_srv = self.create_service(
            ClickDrive, '/click_drive', self._on_click_drive)
        self.settle_start = None
        self.still_since = None
        self.quit_requested = False
        self.cmd_frame = 'base_link'
        self.last_path_publish = 0.0
        self.input_fd = sys.stdin.fileno()
        self.input_owns_fd = False
        if not sys.stdin.isatty() and sys.stdout.isatty():
            try:
                self.input_fd = os.open(
                    '/dev/tty', os.O_RDONLY | os.O_NONBLOCK)
                self.input_owns_fd = True
            except OSError as exc:
                if exc.errno not in (errno.ENXIO, errno.ENODEV, errno.ENOENT):
                    raise
        self.input_decoder = codecs.getincrementaldecoder(
            sys.stdin.encoding or 'utf-8')(errors='replace')
        self.input_buffer = ''
        self.input_eof = False
        self.timer = self.create_timer(1.0 / COMMAND_RATE_HZ, self._tick)

        print(HELP, flush=True)
        print('motion> ', end='', flush=True)
        self.get_logger().info(
            'Waiting for odometry; motion counts observed mux output')

    def _read_input(self):
        if self.input_eof or not select.select([self.input_fd], [], [], 0)[0]:
            return
        data = os.read(self.input_fd, 4096)
        self.input_buffer += self.input_decoder.decode(data, final=not data)
        while '\n' in self.input_buffer:
            line, self.input_buffer = self.input_buffer.split('\n', 1)
            self._parse_input(line)
            print('motion> ', end='', flush=True)
        if not data:
            self.input_eof = True
            if self.input_buffer:
                self._parse_input(self.input_buffer)
                self.input_buffer = ''
            # No controlling terminal (e.g. systemd).
            # Keep ROS services running after stdin closes.

    def _parse_input(self, line):
        parts = line.strip().split()
        if not parts:
            return
        command = parts[0]
        if len(parts) == 1 and command in ('s', 'c', 'z', 'p', '?', 'q'):
            self.inputs.put((command, 0.0))
        elif len(parts) == 2 and command in ('f', 'F', 'b', 'B', 'r', 'l'):
            try:
                magnitude = float(parts[1])
                command_target(command, magnitude)
                self.inputs.put((command, magnitude))
            except ValueError:
                print('Enter a positive finite number.', flush=True)
        else:
            print(HELP, flush=True)

    def close_input(self):
        """Close the controlling terminal if this node opened it."""
        if self.input_owns_fd:
            os.close(self.input_fd)
            self.input_owns_fd = False

    def _on_odom(self, msg):
        frame = msg.header.frame_id
        if not frame:
            self.get_logger().warning('Odometry has no frame_id; ignoring')
            return
        p = msg.pose.pose.position
        try:
            pose = (p.x, p.y, yaw_of(msg.pose.pose.orientation))
        except ValueError as exc:
            self.get_logger().warning(str(exc))
            return
        velocities = (msg.twist.twist.linear.x, msg.twist.twist.angular.z)
        if not all(math.isfinite(value) for value in (*pose, *velocities)):
            self.get_logger().warning(
                'Odometry contains non-finite values; ignoring')
            return

        frame_changed = (
            self.odom_frame is not None
            and frame != self.odom_frame
        )
        if frame_changed:
            self.get_logger().error(
                'Odometry frame changed; stopping test')
            self._stop()
        self.odom_frame = frame
        self.odom_pose = pose
        self.odom_v, self.odom_w = velocities
        self.last_odom = time.monotonic()
        if frame_changed or self.history.origin is None:
            self._reset_origin()
        else:
            self.history.add_actual(pose)

    def _on_mux(self, msg):
        if self.state not in ('moving', 'stopping'):
            return
        now = time.monotonic()
        try:
            interval = self.mux_tracker.observe(msg, now)
        except ValueError as exc:
            self._fail_motion(str(exc))
            return
        self.timing.mux(interval[2] if interval else None)
        previous_progress = self.progress
        if interval is not None:
            linear, angular, duration = interval
            self.history.integrate_command(linear, angular, duration)
            speed = (
                abs(angular) if self.active[0] in ('r', 'l') else abs(linear))
            self.progress += speed * duration
        values = twist_values(msg)
        self.observed_velocity = (values[0], values[5])
        if self.state == 'stopping' and not any(values):
            elapsed = now - self.target_reached_at
            extra = self.progress - self.target_reached_progress
            self.zero_odom_yaw = (
                self.odom_pose[2] if self.odom_pose is not None else None)
            self.get_logger().info(
                f'Target-to-zero output: {elapsed:.3f}s; additional observed '
                f'commanded progress={extra:.5f} SI; '
                f'total excess over target='
                f'{self.progress - self.active[2]:+.5f} SI; '
                f'latest odometry age={now - self.last_odom:.3f}s')
            self.get_logger().info(
                f'Command timing (local): {self.timing.summary()}')
            self.mux_tracker.stop()
            self.state = 'settling'
            self.settle_start = now
            self.still_since = None
            self.get_logger().info(
                'Zero mux output observed; waiting for odometry to settle')
        elif self.state == 'moving' and self.progress >= self.active[2]:
            self.state = 'stopping'
            self.target_reached_at = now
            self.target_reached_progress = self.progress
            self.target_odom_yaw = (
                self.odom_pose[2] if self.odom_pose is not None else None)
            self.get_logger().info(
                f'Target reached at observed progress={self.progress:.5f} '
                f'SI; target={self.active[2]:.5f} SI; '
                f'latest odometry age={now - self.last_odom:.3f}s; '
                'publishing zero')
            self._publish_cmd(0.0, 0.0)
            self.get_logger().info(
                f'Target crossing: previous={previous_progress:.5f} SI, '
                f'interval={interval[2]:.3f}s, '
                f'callback-to-zero publish={time.monotonic() - now:.3f}s')

    def _reset_origin(self):
        if self.odom_pose is None:
            return
        self.history.clear(self.odom_pose)
        self._publish_paths()
        self.get_logger().info('Both paths reset to the current odometry pose')

    def _pose_stamped(self, relative, stamp):
        pose = world_pose(self.history.origin, relative)
        message = PoseStamped()
        message.header.stamp = stamp
        message.header.frame_id = self.odom_frame or 'odom'
        message.pose.position.x, message.pose.position.y = pose[:2]
        message.pose.orientation.z = math.sin(pose[2] / 2)
        message.pose.orientation.w = math.cos(pose[2] / 2)
        return message

    def _publish_paths(self):
        start = time.monotonic()
        points_count = (
            len(self.history.commanded_path), len(self.history.actual_path))
        for publisher, points in (
                (self.cmd_path_pub, self.history.commanded_path),
                (self.actual_path_pub, self.history.actual_path)):
            message = Path()
            message.header.stamp = self.get_clock().now().to_msg()
            message.header.frame_id = self.odom_frame or 'odom'
            message.poses = [
                self._pose_stamped(point, message.header.stamp)
                for point in display_points(points, PATH_DISPLAY_LIMIT)]
            publisher.publish(message)
        self.last_path_publish = time.monotonic()
        if self.state in ('moving', 'stopping'):
            self.timing.path(self.last_path_publish - start, points_count)

    def _publish_cmd(self, linear, angular):
        message = TwistStamped()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = self.cmd_frame
        message.twist.linear.x = linear
        message.twist.angular.z = angular
        if self.state in ('moving', 'stopping'):
            self.timing.send(time.monotonic())
        self.cmd_pub.publish(message)

    def _publish_state(self, linear, angular):
        if self.history.origin is None:
            return
        message = Odometry()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = self.odom_frame or 'odom'
        message.child_frame_id = 'cmd_base_link'
        pose = world_pose(self.history.origin, self.history.commanded_pose)
        message.pose.pose.position.x, message.pose.pose.position.y = pose[:2]
        message.pose.pose.orientation.z = math.sin(pose[2] / 2)
        message.pose.pose.orientation.w = math.cos(pose[2] / 2)
        message.twist.twist.linear.x = linear
        message.twist.twist.angular.z = angular
        self.cmd_odom_pub.publish(message)

    def _fresh_odom(self):
        return (
            self.last_odom is not None
            and time.monotonic() - self.last_odom <= self.odom_timeout
        )

    def _stopped(self):
        return (
            abs(self.odom_v) <= self.stop_v
            and abs(self.odom_w) <= self.stop_w
        )

    def _stop(self):
        if self.active is not None and self.command_success:
            self.command_success = False
            self.failure_reason = 'Command stopped before completion'
        self.active = None
        self.pending.clear()
        self.mux_tracker.stop()
        self.observed_velocity = (0.0, 0.0)
        self.state = 'idle'
        self.progress = 0.0
        self.timing = TimingDiagnostics()
        self.still_since = None
        self._publish_cmd(0.0, 0.0)

    def _report(self, final=False):
        if self.odom_pose is None or self.history.origin is None:
            self.get_logger().warning('No odometry received yet')
            return
        actual = relative_pose(self.history.origin, self.odom_pose)
        commanded = self.history.commanded_pose
        position_error = math.hypot(
            actual[0] - commanded[0], actual[1] - commanded[1])
        heading_error = math.degrees(
            wrap_angle(actual[2] - commanded[2]))
        if self.active is None:
            result = 'idle'
        elif final:
            result = 'completed' if self.command_success else 'not completed'
        elif self.state == 'moving':
            result = 'in progress'
        elif self.command_success:
            result = 'settling'
        else:
            result = 'failed; waiting for stop'
        self.get_logger().info(
            f'Command {result}: '
            f'commanded x={commanded[0]:.3f} y={commanded[1]:.3f} '
            f'yaw={math.degrees(commanded[2]):.2f} deg | '
            f'odom x={actual[0]:.3f} y={actual[1]:.3f} '
            f'yaw={math.degrees(actual[2]):.2f} deg | '
            f'position error={position_error:.3f} m, '
            f'heading error={heading_error:+.2f} deg')
        if self.failure_reason:
            self.get_logger().warning(self.failure_reason)

    def _manual_active(self):
        return (
            self.last_manual_override is not None
            and time.monotonic() - self.last_manual_override < 0.7
        )

    def _on_manual_override(self, msg):
        if (
            abs(msg.twist.linear.x) <= 1e-9
            and abs(msg.twist.angular.z) <= 1e-9
        ):
            return

        self.last_manual_override = time.monotonic()

        if self.active is not None or self.pending:
            self.get_logger().warning(
                'Joystick override: canceling motion')
            self._stop()

    def _on_click_drive(self, request, response):
        """Handle camera-directed rotation and cancellation."""
        command = request.command.strip().upper()

        if command == 'STATUS':
            response.accepted = True
            response.state = self.state
            response.message = (
                f'active={self.active}, pending={len(self.pending)}')
            return response

        if command == 'STOP':
            self._stop()
            response.accepted = True
            response.state = self.state
            response.message = 'Motion stopped and queue cleared'
            return response

        if self._manual_active():
            response.accepted = False
            response.state = self.state
            response.message = 'Joystick currently controlling robot'
            return response

        if command not in ('TURN', 'DRIVE'):
            response.accepted = False
            response.state = self.state
            response.message = f'Unsupported command: {command}'
            return response

        if command == 'TURN':
            magnitude = float(request.angle_deg)
            if (not math.isfinite(magnitude)
                    or abs(magnitude) < 0.1
                    or abs(magnitude) > 180):
                response.accepted = False
                response.state = self.state
                response.message = 'Angle must be between 0.1 and 180 degrees'
                return response
        else:
            magnitude = float(request.distance_m)
            if not math.isfinite(magnitude) or not 0.1 <= magnitude <= 5.0:
                response.accepted = False
                response.state = self.state
                response.message = 'Distance must be between 0.1 and 5.0 meters'
                return response

        if (
            self.state != 'idle'
            or self.active is not None
            or self.pending
            or not self.inputs.empty()
        ):
            response.accepted = False
            response.state = self.state
            response.message = 'Motion controller busy'
            return response

        if not self._fresh_odom() or not self._stopped():
            response.accepted = False
            response.state = self.state
            response.message = 'Odometry unavailable or robot moving'
            return response

        if command == 'TURN':
            direction = 'r' if magnitude > 0 else 'l'
            magnitude = abs(magnitude)
            self.pending.append((direction, magnitude))
            message = f'Turn {direction} {magnitude:.2f} degrees'
        else:
            self.pending.append(('F', magnitude))
            message = f'Drive forward {magnitude:.2f} meters'

        response.accepted = True
        response.state = 'queued'
        response.message = message
        self.get_logger().info(
            f'ClickDrive: {response.message}')
        return response

    def _handle_inputs(self):
        while True:
            try:
                command, magnitude = self.inputs.get_nowait()
            except queue.Empty:
                return
            if command == '?':
                print(HELP, flush=True)
            elif command == 'p':
                self._report()
            elif command == 's':
                self._stop()
                self.get_logger().info('STOP: queued movements discarded')
            elif command == 'q':
                self._stop()
                self.quit_requested = True
                return
            elif command in ('c', 'z'):
                if (
                    self.state != 'idle'
                    or self.pending
                    or not self._fresh_odom()
                    or not self._stopped()
                ):
                    self.get_logger().warning(
                        'Clear requires fresh odometry and an idle, '
                        'stopped robot')
                else:
                    self._reset_origin()
            else:
                self.pending.append((command, magnitude))

    def _start_next(self):
        if not self.pending or not self._fresh_odom() or not self._stopped():
            return
        command, magnitude = self.pending.popleft()
        target = command_target(command, magnitude)
        self.active = (command, magnitude, target)
        self.progress = 0.0
        self.timing = TimingDiagnostics()
        self.target_reached_at = None
        self.target_reached_progress = None
        self.target_odom_yaw = None
        self.zero_odom_yaw = None
        self.command_success = True
        self.failure_reason = None
        self.mux_tracker.start(time.monotonic())
        self.observed_velocity = (0.0, 0.0)
        self.state = 'moving'
        self.get_logger().info(
            f'Start {command} {magnitude:g}; target {target:.4f} SI units')

    def _fail_motion(self, reason):
        self.command_success = False
        self.failure_reason = reason
        self.pending.clear()
        self.mux_tracker.stop()
        self.observed_velocity = (0.0, 0.0)
        self.state = 'settling'
        self.settle_start = time.monotonic()
        self.still_since = None
        self.get_logger().error(reason)
        self._publish_cmd(0.0, 0.0)
        self.get_logger().info(
            f'Command timing at failure (local): {self.timing.summary()}')

    def _finish_settling(self, now):
        if (
            self.last_odom is not None
            and self.last_odom >= self.settle_start
            and self._stopped()
        ):
            if self.still_since is None:
                self.still_since = now
            if now - self.still_since >= self.stop_hold:
                if (
                    self.command_success and self.active[0] in ('r', 'l')
                    and self.target_odom_yaw is not None
                    and self.zero_odom_yaw is not None
                    and self.odom_pose is not None
                ):
                    before_zero = math.degrees(wrap_angle(
                        self.zero_odom_yaw - self.target_odom_yaw))
                    after_zero = math.degrees(wrap_angle(
                        self.odom_pose[2] - self.zero_odom_yaw))
                    self.get_logger().info(
                        'Measured yaw change: target to zero output '
                        f'{before_zero:+.2f} deg; zero output to settled '
                        f'{after_zero:+.2f} deg (local receipt snapshots; '
                        f'latest odometry age={now - self.last_odom:.3f}s)')
                self._report(final=True)
                self.state = 'idle'
                self.active = None
        else:
            self.still_since = None
        if (
                self.state == 'settling'
                and now - self.settle_start > self.settle_timeout):
            self.get_logger().error(
                'Robot did not settle before timeout; '
                'final errors were not reported')
            self._stop()

    def _tick(self):
        now = time.monotonic()
        if self.state in ('moving', 'stopping'):
            self.timing.tick(now)
        self._read_input()
        self._handle_inputs()
        if self.quit_requested:
            return
        if self.state != 'idle' and not self._fresh_odom():
            self.get_logger().error(
                'Odometry stale: stopping and discarding queued commands')
            self._stop()
        if self.state == 'idle':
            self._start_next()

        linear = angular = 0.0
        if self.state in ('moving', 'stopping'):
            try:
                self.mux_tracker.check_timeout(now)
            except ValueError as exc:
                self._fail_motion(str(exc))
        if self.state == 'moving':
            command, _, _ = self.active
            if command in ('r', 'l'):
                angular = rotation_sign(command) * self.angular_speed
            elif command in ('b', 'B'):
                linear = -self.linear_speed
            else:
                linear = self.linear_speed
        elif self.state == 'settling':
            self._finish_settling(now)

        self._publish_cmd(linear, angular)
        self._publish_state(*self.observed_velocity)
        if (now - self.last_path_publish
                >= 1.0 / PATH_PUBLISH_RATE_HZ):
            self._publish_paths()


def main():
    """Run the interactive ROS 2 motion test node."""
    shutdown_requested = False

    def request_shutdown(signum, frame):
        nonlocal shutdown_requested
        shutdown_requested = True

    previous_handlers = {
        signum: signal.signal(signum, request_shutdown)
        for signum in (signal.SIGINT, signal.SIGTERM)
    }
    node = None
    initialized = False
    try:
        rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
        initialized = True
        node = MotionTest()
        while (rclpy.ok() and not node.quit_requested
               and not shutdown_requested):
            try:
                rclpy.spin_once(node, timeout_sec=0.1)
            except ExternalShutdownException:
                if rclpy.ok():
                    raise
                break
    except KeyboardInterrupt:
        pass
    finally:
        try:
            if node is not None:
                try:
                    if rclpy.ok(context=node.context):
                        try:
                            node._stop()
                            for _ in range(10):
                                if not rclpy.ok(context=node.context):
                                    break
                                node._publish_cmd(0.0, 0.0)
                                time.sleep(0.03)
                        except RCLError:
                            if rclpy.ok(context=node.context):
                                raise
                            print('ROS context shut down before final stop',
                                  file=sys.stderr, flush=True)
                    else:
                        print('ROS context shut down before final stop',
                              file=sys.stderr, flush=True)
                finally:
                    node.close_input()
                    node.destroy_node()
        finally:
            if initialized:
                rclpy.try_shutdown()
            for signum, handler in previous_handlers.items():
                signal.signal(signum, handler)


if __name__ == '__main__':
    main()
