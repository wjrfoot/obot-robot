"""Unit tests for the terminal motion-test calculations and path history."""

from collections import deque
import math
import os
from pathlib import Path
import queue
import signal
import subprocess
import sys
import time
from types import SimpleNamespace
from unittest.mock import Mock

from geometry_msgs.msg import PoseStamped, TwistStamped

from nav_msgs.msg import Odometry

from obot import obot_cmd_motion as motion
from obot.obot_cmd_motion import (
    command_target,
    integrate_pose,
    MotionHistory,
    MuxOutputTracker,
    relative_pose,
    rotation_sign,
    world_pose,
)

import pytest

import rclpy
from rclpy.executors import SingleThreadedExecutor

import yaml


@pytest.mark.parametrize(('command', 'magnitude', 'expected'), [
    ('f', 1.0, 0.3048),
    ('b', 2.0, 0.6096),
    ('F', 1.5, 1.5),
    ('B', 0.25, 0.25),
    ('r', 90.0, math.pi / 2),
    ('l', 180.0, math.pi),
])
def test_command_target_converts_distance_and_rotation(
        command, magnitude, expected):
    """Convert all supported CLI units into meters or radians."""
    assert math.isclose(command_target(command, magnitude), expected)


@pytest.mark.parametrize('magnitude', [0.0, -1.0, float('nan'), float('inf')])
def test_command_target_rejects_invalid_magnitudes(magnitude):
    """Reject non-positive and non-finite motion magnitudes."""
    with pytest.raises(ValueError):
        command_target('f', magnitude)


def test_rotation_direction():
    """Use negative yaw for clockwise and positive yaw for counterclockwise."""
    assert rotation_sign('r') == -1
    assert rotation_sign('l') == 1
    with pytest.raises(ValueError):
        rotation_sign('f')


def test_commanded_pose_integration():
    """Integrate both straight-line and constant-curvature motion."""
    pose = integrate_pose((0.0, 0.0, 0.0), 0.5, 0.0, 2.0)
    assert pose == pytest.approx((1.0, 0.0, 0.0))

    pose = integrate_pose((0.0, 0.0, 0.0), 1.0, 1.0, math.pi / 2)
    assert pose == pytest.approx((1.0, 1.0, math.pi / 2))


def test_odometry_relative_pose_and_world_transform():
    """Convert between odometry-frame poses and the relative test frame."""
    origin = (3.0, 4.0, math.pi / 2)
    actual = (3.0, 5.0, math.pi)
    relative = relative_pose(origin, actual)
    assert relative == pytest.approx((1.0, 0.0, math.pi / 2))
    assert world_pose(origin, relative) == pytest.approx(actual)


def test_path_clear_establishes_a_shared_new_origin():
    """Reset both paths to a matching pose at a new origin."""
    history = MotionHistory()
    history.clear((2.0, -1.0, math.pi / 4))
    history.integrate_command(0.5, 0.0, 2.0)
    history.add_actual((3.0, -1.0, math.pi / 4))
    assert len(history.commanded_path) == 2
    assert len(history.actual_path) == 2

    new_origin = (10.0, 8.0, -math.pi / 2)
    history.clear(new_origin)
    assert history.origin == new_origin
    assert history.commanded_pose == (0.0, 0.0, 0.0)
    assert list(history.commanded_path) == [(0.0, 0.0, 0.0)]
    assert list(history.actual_path) == [(0.0, 0.0, 0.0)]
    assert world_pose(
        history.origin, history.commanded_path[0]) == pytest.approx(new_origin)
    assert world_pose(
        history.origin, history.actual_path[0]) == pytest.approx(new_origin)


@pytest.mark.parametrize('count', [0, 1, 2, 499, 500, 501, 10000, 100000])
def test_display_points_are_bounded_and_preserve_endpoints(count):
    """Keep ordered endpoints without mutating full-resolution history."""
    points = list(range(count))
    selected = motion.display_points(points, motion.PATH_DISPLAY_LIMIT)
    assert len(selected) == min(count, 500)
    assert selected == sorted(set(selected))
    assert points == list(range(count))
    if count:
        assert selected[0] == points[0]
        assert selected[-1] == points[-1]


def test_full_history_preserves_samples_beyond_previous_limit():
    """Keep historic points and integrate independently of display sampling."""
    history = MotionHistory()
    history.clear((0.0, 0.0, 0.0))
    for index in range(10001):
        history.integrate_command(0.2, 0.0, 0.01)
        history.add_actual((index * 0.002, 0.0, 0.0))
    assert len(history.commanded_path) == 10002
    assert len(history.actual_path) == 10002
    assert history.commanded_path[0] == (0.0, 0.0, 0.0)
    assert history.actual_path[0] == (0.0, 0.0, 0.0)
    assert history.commanded_pose[0] == pytest.approx(20.002)
    assert history.commanded_path[-1] == history.commanded_pose


def mux_message(sequence, linear=0.2, angular=0.0):
    """Create an identifiable stamped velocity message."""
    message = TwistStamped()
    message.header.stamp.sec = 1
    message.header.stamp.nanosec = sequence
    message.header.frame_id = 'base_link'
    message.twist.linear.x = linear
    message.twist.angular.z = angular
    return message


def test_delayed_forwarding_counts_only_observed_intervals():
    """Do not count waiting before the first output or a requested velocity."""
    tracker = MuxOutputTracker(timeout=1.0)
    history = MotionHistory()
    history.clear((0.0, 0.0, 0.0))
    tracker.start(0.0)
    first = mux_message(1, linear=0.15)
    second = mux_message(2, linear=0.25)
    assert tracker.observe(first, 0.4) is None
    interval = tracker.observe(second, 0.5)
    assert interval == pytest.approx((0.15, 0.0, 0.1))
    history.integrate_command(*interval)
    assert history.commanded_pose == pytest.approx((0.015, 0.0, 0.0))


def test_blocked_command_times_out_without_integration():
    """An unobserved command has no motion interval."""
    tracker = MuxOutputTracker(timeout=0.5)
    tracker.start(1.0)
    tracker.check_timeout(1.5)
    with pytest.raises(ValueError, match='timed out'):
        tracker.check_timeout(1.51)
    assert tracker.last_observation is None


def test_zero_output_closes_previous_interval():
    """Observed zero closes the preceding motion interval."""
    tracker = MuxOutputTracker(timeout=0.5)
    tracker.start(0.1)
    moving = mux_message(2, angular=-0.3, linear=0.0)
    zero = mux_message(3, linear=0.0)
    assert tracker.observe(moving, 0.12) is None
    interval = tracker.observe(zero, 0.22)
    assert interval == pytest.approx((0.0, -0.3, 0.10))


def test_stale_mux_observation_gap_fails():
    """Reject output after an interrupted stream, even before the next tick."""
    tracker = MuxOutputTracker(timeout=0.5)
    tracker.start(0.0)
    first, second = mux_message(1), mux_message(2)
    assert tracker.observe(first, 0.1) is None
    with pytest.raises(ValueError, match='feedback timed out'):
        tracker.observe(second, 0.7)


def test_nonfinite_mux_velocity_fails():
    """Do not integrate invalid output or treat it as fresh feedback."""
    tracker = MuxOutputTracker(timeout=0.5)
    tracker.start(0.0)
    with pytest.raises(ValueError, match='non-finite'):
        tracker.observe(mux_message(1, linear=float('nan')), 0.1)
    assert tracker.last_observation is None


def fake_motion_node():
    """Create the state needed to exercise actual motion-node callbacks."""
    history = MotionHistory()
    history.clear((0.0, 0.0, 0.0))
    node = SimpleNamespace(
        state='moving',
        active=('F', 0.015, 0.015),
        progress=0.0,
        timing=motion.TimingDiagnostics(),
        command_success=True,
        failure_reason=None,
        pending=deque([('f', 1.0)]),
        mux_tracker=MuxOutputTracker(0.5),
        observed_velocity=(0.0, 0.0),
        history=history,
        last_odom=0.2,
        odom_pose=(0.0, 0.0, 0.0),
        settle_start=None,
        still_since=None,
        stop_hold=0.5,
        settle_timeout=8.0,
        get_logger=Mock(return_value=Mock()),
        _publish_cmd=Mock(),
        _report=Mock(),
        _stopped=Mock(return_value=True),
    )
    node._fail_motion = lambda reason: motion.MotionTest._fail_motion(
        node, reason)
    node.mux_tracker.start(0.0)
    return node


def test_node_stops_at_target_and_waits_for_forwarded_zero(monkeypatch):
    """Integrate observed output through zero and settle only afterwards."""
    now = [0.0]
    monkeypatch.setattr(motion, 'time', SimpleNamespace(
        monotonic=lambda: now[0]))
    node = fake_motion_node()
    first, second = mux_message(1), mux_message(2)
    now[0] = 0.1
    motion.MotionTest._on_mux(node, first)
    assert node.progress == 0.0
    now[0] = 0.2
    motion.MotionTest._on_mux(node, second)
    assert node.progress == pytest.approx(0.02)
    assert node.active[2] == 0.015
    assert node.state == 'stopping'
    node._publish_cmd.assert_called_once_with(0.0, 0.0)
    zero = mux_message(3, linear=0.0)
    now[0] = 0.3
    motion.MotionTest._on_mux(node, zero)
    assert node.progress == pytest.approx(0.04)
    assert node.history.commanded_pose[0] == pytest.approx(0.04)
    assert node.state == 'settling'
    motion.MotionTest._finish_settling(node, 0.3)
    node._report.assert_not_called()
    assert node.still_since is None
    node.last_odom = 0.31
    motion.MotionTest._finish_settling(node, 0.31)
    motion.MotionTest._finish_settling(node, 0.81)
    node._report.assert_called_once_with(final=True)
    assert node.state == 'idle'


def test_delayed_callback_causes_large_target_crossing(monkeypatch):
    """A delayed callback makes progress jump despite regular send timing."""
    now = [0.01]
    monkeypatch.setattr(motion, 'time', SimpleNamespace(
        monotonic=lambda: now[0]))
    node = fake_motion_node()
    node.active = ('F', 0.04, 0.04)
    first, second = mux_message(1), mux_message(2)
    motion.MotionTest._on_mux(node, first)
    now[0] = 0.26
    motion.MotionTest._on_mux(node, second)
    assert node.progress == pytest.approx(0.05)
    assert node.history.commanded_pose[0] == pytest.approx(0.05)
    assert node.state == 'stopping'
    assert node.timing.max_mux_gap == pytest.approx(0.25)
    logs = ' '.join(
        str(call.args[0]) for call in node.get_logger().info.call_args_list)
    assert 'interval=0.250s' in logs
    node._publish_cmd.assert_called_once_with(0.0, 0.0)


def test_batched_callbacks_use_callback_time_not_header_time(monkeypatch):
    """A delayed echo followed by a batch biases the apparent intervals."""
    now = [0.0]
    monkeypatch.setattr(motion, 'time', SimpleNamespace(
        monotonic=lambda: now[0]))
    node = fake_motion_node()
    node.active = ('F', 1.0, 1.0)
    messages = [mux_message(n) for n in (1, 2, 3)]
    for callback_time, message in zip((0.01, 0.20, 0.201), messages):
        now[0] = callback_time
        motion.MotionTest._on_mux(node, message)
    assert node.progress == pytest.approx(0.2 * (0.201 - 0.01))
    assert node.timing.max_mux_gap == pytest.approx(0.19)
    assert node.state == 'moving'


def test_timing_diagnostics_capture_path_cost_and_reset():
    """Track maxima per command without changing trajectory state."""
    timing = motion.TimingDiagnostics()
    timing.tick(1.0)
    timing.tick(1.2)
    timing.tick(1.23)
    timing.send(1.0)
    timing.send(1.04)
    timing.mux(0.1)
    timing.mux(0.04)
    timing.path(0.26, (5000, 4900))
    timing.path(0.02, (100, 100))
    assert timing.max_tick_gap == pytest.approx(0.2)
    assert timing.max_send_gap == pytest.approx(0.04)
    assert timing.max_mux_gap == pytest.approx(0.1)
    assert 'max path publish=0.260s (cmd,actual history points=(5000, 4900))' \
        in timing.summary()

    node = fake_motion_node()
    node.pending = deque([('F', 1.0)])
    node._fresh_odom = Mock(return_value=True)
    node.timing = timing
    motion.MotionTest._start_next(node)
    assert node.timing.max_path_duration == 0.0
    assert node.timing.last_tick is None


def test_path_publish_records_full_build_and_publish_time(monkeypatch):
    """Measure both complete path builds and their publication."""
    times = iter((1.0, 1.25))
    monkeypatch.setattr(motion, 'time', SimpleNamespace(
        monotonic=lambda: next(times)))
    node = fake_motion_node()
    node.history.commanded_path.append((1.0, 0.0, 0.0))
    node.history.actual_path.append((0.5, 0.0, 0.0))
    node.history.actual_path.append((1.0, 0.0, 0.0))
    node.odom_frame = 'odom'
    node.cmd_path_pub = Mock()
    node.actual_path_pub = Mock()
    node._pose_stamped = Mock(side_effect=lambda point, stamp: PoseStamped())
    node.get_clock = Mock(return_value=SimpleNamespace(
        now=lambda: SimpleNamespace(
            to_msg=lambda: mux_message(1).header.stamp)))
    motion.MotionTest._publish_paths(node)
    assert node.timing.max_path_duration == pytest.approx(0.25)
    assert node.timing.max_path_points == (2, 3)
    assert len(node.cmd_path_pub.publish.call_args.args[0].poses) == 2
    assert len(node.actual_path_pub.publish.call_args.args[0].poses) == 3


def test_path_publish_caps_both_paths_without_changing_history(monkeypatch):
    """Limit message work to 500 samples while keeping both endpoints."""
    node = fake_motion_node()
    node.history.commanded_path = [(float(n), 0.0, 0.0) for n in range(20000)]
    node.history.actual_path = [(float(n), 0.0, 0.0) for n in range(30000)]
    node.odom_frame = 'odom'
    node.cmd_path_pub = Mock()
    node.actual_path_pub = Mock()
    node.get_clock = Mock(return_value=SimpleNamespace(
        now=lambda: SimpleNamespace(
            to_msg=lambda: mux_message(1).header.stamp)))

    def pose(point, stamp):
        msg = PoseStamped()
        msg.pose.position.x = point[0]
        return msg

    node._pose_stamped = Mock(side_effect=pose)
    motion.MotionTest._publish_paths(node)
    assert node._pose_stamped.call_count == 1000
    for publisher, count in ((node.cmd_path_pub, 20000),
                             (node.actual_path_pub, 30000)):
        poses = publisher.publish.call_args.args[0].poses
        assert len(poses) == 500
        assert poses[0].pose.position.x == 0.0
        assert poses[-1].pose.position.x == count - 1
    assert len(node.history.commanded_path) == 20000
    assert len(node.history.actual_path) == 30000
    assert node.timing.max_path_points == (20000, 30000)


def test_rotation_reports_crossing_echo_delay_and_odom_coast(monkeypatch):
    """Separate sampled target excess, zero output delay and measured yaw."""
    now = [0.0]
    monkeypatch.setattr(motion, 'time', SimpleNamespace(
        monotonic=lambda: now[0]))
    node = fake_motion_node()
    node.active = ('r', 90.0, math.pi / 2)
    for index in range(136):
        sent_at = index / 30
        message = mux_message(index + 1, linear=0.0, angular=-0.35)
        now[0] = sent_at
        node.last_odom = sent_at
        node.odom_pose = (0.0, 0.0, -sent_at * 0.35)
        motion.MotionTest._on_mux(node, message)
    assert node.state == 'stopping'
    assert node.target_reached_progress == pytest.approx(1.575)
    node.last_odom = 4.65
    node.odom_pose = (0.0, 0.0, -1.60)
    zero = mux_message(137, linear=0.0)
    now[0] = 4.65
    motion.MotionTest._on_mux(node, zero)
    assert node.progress == pytest.approx(1.6275)
    assert math.degrees(node.progress) == pytest.approx(93.25, abs=0.01)
    logs = ' '.join(
        str(call.args[0]) for call in node.get_logger().info.call_args_list)
    assert 'Target reached at observed progress=1.57500' in logs
    assert 'Target-to-zero output: 0.150s' in logs
    assert 'additional observed commanded progress=0.05250' in logs
    node.odom_pose = (0.0, 0.0, -1.62)
    node.last_odom = 4.7
    motion.MotionTest._finish_settling(node, 4.7)
    motion.MotionTest._finish_settling(node, 5.2)
    logs = ' '.join(
        str(call.args[0]) for call in node.get_logger().info.call_args_list)
    assert 'target to zero output -1.43 deg' in logs
    assert 'zero output to settled -1.15 deg' in logs


def test_rviz_motion_displays_and_fixed_frame():
    """Load the launched RViz config and verify all requested displays."""
    config_path = Path(__file__).resolve().parents[1] / 'rviz'
    with (config_path / 'obot_cmd_motion.rviz').open() as config_file:
        config = yaml.safe_load(config_file)
    manager = config['Visualization Manager']
    assert manager['Global Options']['Fixed Frame'] == 'odom'
    displays = {display['Name']: display for display in manager['Displays']}
    for name in ('Robot Model', 'Commanded Path', 'Actual Path',
                 'Commanded Odometry'):
        assert displays[name]['Enabled']
    assert displays['Robot Model']['Description Source'] == 'Topic'
    assert (displays['Robot Model']['Description Topic']['Value']
            == '/robot_description')
    assert displays['Commanded Path']['Topic']['Value'] == '/cmd_path'
    assert displays['Actual Path']['Topic']['Value'] == '/actual_path'
    assert displays['Commanded Odometry']['Topic']['Value'] == '/cmd_odom'
    assert (displays['Commanded Path']['Color']
            != displays['Actual Path']['Color'])


def test_node_integrates_observed_velocities_without_attribution(monkeypatch):
    """Track actual mux output, including a different publisher's velocity."""
    now = [0.0]
    monkeypatch.setattr(motion, 'time', SimpleNamespace(
        monotonic=lambda: now[0]))
    node = fake_motion_node()
    first = mux_message(1)
    node.active = ('F', 1.0, 1.0)
    now[0] = 0.1
    motion.MotionTest._on_mux(node, first)
    now[0] = 0.2
    motion.MotionTest._on_mux(node, mux_message(999, linear=0.1))
    now[0] = 0.3
    motion.MotionTest._on_mux(node, mux_message(500, linear=0.0))
    assert node.progress == pytest.approx(0.03)
    assert node.history.commanded_pose[0] == pytest.approx(0.03)
    assert node.command_success
    assert node.state == 'moving'
    assert node.pending
    node._publish_cmd.assert_not_called()
    node.get_logger().error.assert_not_called()


@pytest.mark.parametrize('stamps', [(1, 3, 4), (3, 1, 2), (1, 1, 2)])
def test_node_continues_across_missing_reordered_or_duplicate_stamps(
        monkeypatch, stamps):
    """Integrate observed callbacks despite skipped or reordered stamps."""
    now = [0.1]
    monkeypatch.setattr(motion, 'time', SimpleNamespace(
        monotonic=lambda: now[0]))
    node = fake_motion_node()
    node.active = ('F', 0.04, 0.04)
    messages = {n: mux_message(n) for n in (1, 2, 3, 4)}
    for callback_time, stamp in zip((0.1, 0.2, 0.3), stamps):
        now[0] = callback_time
        motion.MotionTest._on_mux(node, messages[stamp])
    assert node.progress == pytest.approx(0.04)
    assert node.history.commanded_pose[0] == pytest.approx(0.04)
    assert node.command_success
    assert node.state == 'stopping'
    assert node.pending
    node._publish_cmd.assert_called_once_with(0.0, 0.0)


def test_node_timeout_stops_and_discards_queued_commands(monkeypatch):
    """A missing mux echo produces an error and a zero test command."""
    now = [1.0]
    monkeypatch.setattr(motion, 'time', SimpleNamespace(
        monotonic=lambda: now[0]))
    node = fake_motion_node()
    node.quit_requested = False
    node._handle_inputs = Mock()
    node._read_input = Mock()
    node._fresh_odom = Mock(return_value=True)
    node._finish_settling = Mock()
    node._publish_state = Mock()
    node._publish_paths = Mock()
    node.last_path_publish = 1.0
    motion.MotionTest._tick(node)
    assert not node.command_success
    assert node.state == 'settling'
    assert not node.pending
    assert node.progress == 0.0
    assert node.history.commanded_pose == (0.0, 0.0, 0.0)
    assert node._publish_cmd.call_args_list == [
        ((0.0, 0.0),), ((0.0, 0.0),),
    ]


def test_stale_odom_stops_and_discards_queued_commands(monkeypatch):
    """Stale wheel odometry stops active movement without starting the queue."""
    monkeypatch.setattr(motion, 'time', SimpleNamespace(monotonic=lambda: 1.0))
    node = fake_motion_node()
    node.quit_requested = False
    node._handle_inputs = Mock()
    node._read_input = Mock()
    node._fresh_odom = Mock(return_value=False)
    node._stop = lambda: motion.MotionTest._stop(node)
    node._start_next = lambda: motion.MotionTest._start_next(node)
    node._publish_state = Mock()
    node._publish_paths = Mock()
    node.last_path_publish = 1.0
    motion.MotionTest._tick(node)
    assert node.state == 'idle'
    assert node.active is None
    assert not node.pending
    assert node._publish_cmd.call_args_list == [
        ((0.0, 0.0),), ((0.0, 0.0),),
    ]


def test_second_forward_command_survives_missing_mux_sample(monkeypatch):
    """Consecutive commands keep history when a sample is skipped on the second."""
    now = [0.0]
    monkeypatch.setattr(motion, 'time', SimpleNamespace(monotonic=lambda: now[0]))
    node = fake_motion_node()
    node.pending = deque([('f', 3.0), ('f', 3.0)])
    node.state = 'idle'
    node.active = None
    node._fresh_odom = Mock(return_value=True)
    for command_index in range(2):
        motion.MotionTest._start_next(node)
        assert node.active == ('f', 3.0, pytest.approx(0.9144))
        for sample in range(150):
            now[0] += 0.033
            node.last_odom = now[0]
            if command_index == 1 and sample == 50:
                continue
            motion.MotionTest._on_mux(
                node, mux_message(sample + 1, linear=0.2))
            if node.state == 'stopping':
                break
        assert node.state == 'stopping'
        now[0] += 0.033
        node.last_odom = now[0]
        motion.MotionTest._on_mux(node, mux_message(151, linear=0.0))
        assert node.state == 'settling'
        now[0] += node.stop_hold + 0.01
        node.last_odom = now[0]
        motion.MotionTest._finish_settling(node, now[0])
        now[0] += node.stop_hold + 0.01
        node.last_odom = now[0]
        motion.MotionTest._finish_settling(node, now[0])
        assert node.state == 'idle'
        assert node.command_success
    assert not node.pending
    assert node.history.commanded_pose[0] >= 2 * 0.9144
    assert len(node.history.commanded_path) > 250


def test_missing_forwarded_zero_fails_the_command(monkeypatch):
    """Do not mark a command successful when its zero was never observed."""
    now = [0.1]
    monkeypatch.setattr(motion, 'time', SimpleNamespace(
        monotonic=lambda: now[0]))
    node = fake_motion_node()
    moving = mux_message(1)
    motion.MotionTest._on_mux(node, moving)
    node.state = 'stopping'
    node.quit_requested = False
    node._handle_inputs = Mock()
    node._read_input = Mock()
    node._fresh_odom = Mock(return_value=True)
    node._finish_settling = Mock()
    node._publish_state = Mock()
    node._publish_paths = Mock()
    node.last_path_publish = 0.7
    now[0] = 0.7
    motion.MotionTest._tick(node)
    assert not node.command_success
    assert node.state == 'settling'
    assert node.progress == 0.0
    assert not node.pending
    node.get_logger().error.assert_called_once()


@pytest.mark.parametrize(
    ('command', 'quit_requested'), [('s', False), ('q', True)])
def test_terminal_stop_discards_motion_and_sends_zero(command, quit_requested):
    """Stopping or quitting cancels tracking and immediately publishes zero."""
    node = fake_motion_node()
    node.inputs = queue.Queue()
    node.inputs.put((command, 0.0))
    node.quit_requested = False
    node._stop = lambda: motion.MotionTest._stop(node)
    motion.MotionTest._handle_inputs(node)
    assert node.quit_requested == quit_requested
    assert node.state == 'idle'
    assert node.active is None
    assert not node.pending
    assert not node.mux_tracker.running
    node._publish_cmd.assert_called_once_with(0.0, 0.0)


def test_stamped_echo_over_isolated_ros_transport(monkeypatch):
    """Observe serialized mux-style output without contacting a robot."""
    monkeypatch.setenv('ROS_DOMAIN_ID', '173')
    monkeypatch.setenv('ROS_LOCALHOST_ONLY', '1')
    monkeypatch.setattr(motion.MotionTest, '_read_input', lambda self: None)
    monkeypatch.setattr(sys, 'stdin', sys.__stdin__)
    rclpy.init()
    node = motion.MotionTest()
    node.timer.cancel()
    observer = rclpy.create_node('motion_mux_test')
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    executor.add_node(observer)
    echo = observer.create_publisher(
        TwistStamped, '/diff_drive_controller/cmd_vel', 10)
    subscription = observer.create_subscription(
        TwistStamped, '/cmd_vel_test', echo.publish, 10)
    try:
        deadline = time.monotonic() + 5.0
        while (
            (node.cmd_pub.get_subscription_count() == 0
             or echo.get_subscription_count() == 0)
            and time.monotonic() < deadline
        ):
            executor.spin_once(timeout_sec=0.05)
        assert node.cmd_pub.get_subscription_count() > 0
        assert echo.get_subscription_count() > 0
        node.odom_frame = 'odom'
        node.history.clear((0.0, 0.0, 0.0))
        node.active = ('F', 1.0, 1.0)
        node.state = 'moving'
        node.mux_tracker.start(time.monotonic())
        for expected in (1, 2):
            node._publish_cmd(0.2, 0.0)
            deadline = time.monotonic() + 1.0
            while (
                len(node.history.commanded_path) < expected
                and time.monotonic() < deadline
            ):
                executor.spin_once(timeout_sec=0.02)
            if expected == 1:
                while (node.mux_tracker.last_observation is None
                       and time.monotonic() < deadline):
                    executor.spin_once(timeout_sec=0.02)
                assert node.mux_tracker.last_observation is not None
            else:
                assert len(node.history.commanded_path) >= expected
            time.sleep(0.03)
        assert node.progress > 0.0
        assert node.history.commanded_pose[0] == pytest.approx(node.progress)
        assert node.state == 'moving'
    finally:
        observer.destroy_subscription(subscription)
        executor.shutdown()
        node.destroy_node()
        observer.destroy_node()
        rclpy.shutdown()


def test_main_does_not_publish_after_external_shutdown(monkeypatch, capsys):
    """Skip stop publishing when an external caller invalidated the context."""
    node = Mock()
    node.context = object()
    node.quit_requested = False
    monkeypatch.setattr(motion, 'MotionTest', lambda: node)
    monkeypatch.setattr(motion.rclpy, 'init', Mock())
    monkeypatch.setattr(motion.rclpy, 'ok', Mock(return_value=False))
    monkeypatch.setattr(motion.rclpy, 'try_shutdown', Mock())
    motion.main()
    node._stop.assert_not_called()
    node._publish_cmd.assert_not_called()
    node.close_input.assert_called_once()
    node.destroy_node.assert_called_once()
    assert 'context shut down before final stop' in capsys.readouterr().err


@pytest.fixture
def isolated_ros_env(monkeypatch):
    """Keep ROS test traffic on loopback in an isolated domain."""
    monkeypatch.setenv('ROS_DOMAIN_ID', '173')
    monkeypatch.setenv('RMW_IMPLEMENTATION', 'rmw_cyclonedds_cpp')
    monkeypatch.setenv('CYCLONEDDS_URI', (
        '<CycloneDDS><Domain><General><Interfaces>'
        '<NetworkInterface name="lo"/></Interfaces>'
        '<AllowMulticast>false</AllowMulticast></General>'
        '<Discovery><ParticipantIndex>auto</ParticipantIndex>'
        '<Peers><Peer address="127.0.0.1"/></Peers></Discovery>'
        '</Domain></CycloneDDS>'
    ))


def test_motion_executor_with_long_paths(isolated_ros_env, monkeypatch):
    """Run real timers and mux echoes with long paths in an isolated domain."""
    monkeypatch.setattr(motion.MotionTest, '_read_input', lambda self: None)
    monkeypatch.setattr(sys, 'stdin', sys.__stdin__)
    rclpy.init()
    node = motion.MotionTest()
    observer = rclpy.create_node('long_path_motion_test')
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    executor.add_node(observer)
    echo = observer.create_publisher(
        TwistStamped, '/diff_drive_controller/cmd_vel', 10)
    command_times = []
    path_times = []
    path_sizes = []

    def relay(message):
        command_times.append(time.monotonic())
        echo.publish(message)

    command_sub = observer.create_subscription(
        TwistStamped, '/cmd_vel_test', relay, 10)

    def receive_path(message):
        if len(message.poses) > 1:
            path_times.append(time.monotonic())
            path_sizes.append(len(message.poses))

    from nav_msgs.msg import Path as PathMessage
    path_sub = observer.create_subscription(
        PathMessage, '/cmd_path', receive_path, 10)
    odom_pub = observer.create_publisher(
        Odometry, '/diff_drive_controller/odom', 10)
    odom = Odometry()
    odom.header.frame_id = 'odom'
    odom.pose.pose.orientation.w = 1.0
    odom_timer = observer.create_timer(
        0.03, lambda: odom_pub.publish(odom))
    try:
        deadline = time.monotonic() + 5.0
        while (not node._fresh_odom() or not command_times
               or echo.get_subscription_count() == 0):
            assert time.monotonic() < deadline
            executor.spin_once(timeout_sec=0.02)
        node.history.commanded_path = [
            (i * 0.001, 0.0, 0.0) for i in range(100000)]
        node.history.actual_path = [
            (i * 0.001, 0.0, 0.0) for i in range(100000)]
        node.pending.append(('F', 0.25))
        deadline = time.monotonic() + 5.0
        while node.state != 'settling':
            assert time.monotonic() < deadline
            executor.spin_once(timeout_sec=0.02)
        assert node.command_success, node.failure_reason
        assert node.progress >= 0.25
        assert node.timing.max_path_points[0] >= 100000
        assert node.timing.max_path_duration < 0.2
        assert node.timing.max_tick_gap < 0.2
        assert node.timing.max_mux_gap < 0.2
        assert len(path_sizes) >= 2
        assert all(size <= 500 for size in path_sizes)
        assert all(
            later - earlier > 0.35
            for earlier, later in zip(path_times, path_times[1:]))
        assert len(command_times) >= 30
        print(
            f'long path: path max={node.timing.max_path_duration:.4f}s, '
            f'tick gap max={node.timing.max_tick_gap:.4f}s, '
            f'mux gap max={node.timing.max_mux_gap:.4f}s')
    finally:
        observer.destroy_timer(odom_timer)
        observer.destroy_subscription(command_sub)
        observer.destroy_subscription(path_sub)
        executor.shutdown()
        node.close_input()
        node.destroy_node()
        observer.destroy_node()
        rclpy.shutdown()


@pytest.mark.parametrize('shutdown', ['sigint', 'sigterm', 'q'])
def test_shutdown_sends_zero_with_open_stdin(isolated_ros_env, shutdown):
    """Exercise signals and q during motion over loopback-only ROS."""
    rclpy.init()
    observer = rclpy.create_node('motion_shutdown_test')
    executor = SingleThreadedExecutor()
    executor.add_node(observer)
    received = []
    echo = observer.create_publisher(
        TwistStamped, '/diff_drive_controller/cmd_vel', 10)

    def on_command(message):
        received.append(message.twist.linear.x)
        echo.publish(message)

    subscription = observer.create_subscription(
        TwistStamped, '/cmd_vel_test', on_command, 10)
    odom = observer.create_publisher(
        Odometry, '/diff_drive_controller/odom', 10)
    child = subprocess.Popen(
        [sys.executable, '-m', 'obot.obot_cmd_motion'],
        stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.PIPE,
        env=os.environ.copy(),
    )

    def spin_until(predicate, timeout):
        deadline = time.monotonic() + timeout
        while not predicate() and time.monotonic() < deadline:
            message = Odometry()
            message.header.frame_id = 'odom'
            message.header.stamp = observer.get_clock().now().to_msg()
            message.pose.pose.orientation.w = 1.0
            odom.publish(message)
            executor.spin_once(timeout_sec=0.03)
        return predicate()

    try:
        assert spin_until(
            lambda: odom.get_subscription_count() > 0 and bool(received), 8)
        child.stdin.write(b'F ')
        child.stdin.flush()
        count = len(received)
        assert spin_until(lambda: len(received) >= count + 3, 2)
        assert all(value == 0.0 for value in received[count:])
        child.stdin.write(b'1\n')
        child.stdin.flush()
        assert spin_until(lambda: any(value > 0 for value in received), 5)
        before_shutdown = len(received)
        if shutdown == 'q':
            child.stdin.write(b'q\n')
            child.stdin.flush()
        else:
            child.send_signal(
                signal.SIGINT if shutdown == 'sigint' else signal.SIGTERM)
        assert spin_until(
            lambda: any(value == 0.0 for value in received[before_shutdown:]),
            5)
        assert spin_until(lambda: child.poll() is not None, 5)
        stdout, stderr = child.communicate(timeout=5)
        assert child.returncode == 0, (stdout, stderr)
        assert b'context is invalid' not in stderr
        assert b'Fatal Python error' not in stderr
    finally:
        if child.poll() is None:
            child.kill()
            child.communicate(timeout=5)
        observer.destroy_subscription(subscription)
        executor.shutdown()
        observer.destroy_node()
        rclpy.shutdown()


def test_launch_style_input_uses_controlling_terminal(isolated_ros_env):
    """A piped stdin and PTY output still accept commands from the terminal."""
    script = r"""
import os
import pty
import select
import signal
import sys
import time

read_fd, write_fd = os.pipe()
pid, terminal = pty.fork()
if pid == 0:
    os.dup2(read_fd, 0)
    os.close(read_fd)
    os.close(write_fd)
    os.execv(sys.executable, [sys.executable, '-m', 'obot.obot_cmd_motion'])

os.close(read_fd)
output = b''
deadline = time.monotonic() + 10
sent = False
try:
    while time.monotonic() < deadline:
        ready, _, _ = select.select([terminal], [], [], 0.1)
        if ready:
            try:
                output += os.read(terminal, 4096)
            except OSError:
                pass
        if not sent and b'motion> ' in output:
            os.write(terminal, b'q\n')
            sent = True
        done, status = os.waitpid(pid, os.WNOHANG)
        if done:
            print(output.decode(errors='replace'))
            if not sent or not os.WIFEXITED(status):
                sys.exit(1)
            sys.exit(os.WEXITSTATUS(status))
    os.kill(pid, signal.SIGKILL)
    os.waitpid(pid, 0)
    print(output.decode(errors='replace'), file=sys.stderr)
    sys.exit(1)
finally:
    os.close(write_fd)
    os.close(terminal)
"""
    result = subprocess.run(
        [sys.executable, '-c', script], capture_output=True, timeout=15,
        env=os.environ.copy(),
    )
    assert result.returncode == 0, (result.stdout, result.stderr)
    assert b'Fatal Python error' not in result.stderr


def test_stdin_eof_keeps_controller_running(isolated_ros_env):
    """Closed stdin must not terminate the systemd motion controller."""
    with subprocess.Popen(
        [sys.executable, '-m', 'obot.obot_cmd_motion'],
        stdin=subprocess.PIPE,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        env=os.environ.copy(),
    ) as process:
        process.stdin.close()
        try:
            process.wait(timeout=2)
        except subprocess.TimeoutExpired:
            pass  # Expected: controller remains running.
        else:
            pytest.fail(
                f"Controller exited on stdin EOF: {process.returncode}"
            )
        finally:
            process.terminate()
            try:
                process.communicate(timeout=5)
            except (subprocess.TimeoutExpired, ValueError):
                process.kill()
                process.wait()
