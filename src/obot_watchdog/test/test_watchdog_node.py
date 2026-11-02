"""Node-level tests with ROS 2 but no CAN hardware and no robot network."""

import os
from types import SimpleNamespace

import pytest

os.environ['ROS_DOMAIN_ID'] = '97'
os.environ['ROS_LOCALHOST_ONLY'] = '1'

import rclpy  # noqa: E402
from rclpy.parameter import Parameter  # noqa: E402

from obot_watchdog.core import Frame, Notify, CRITICAL  # noqa: E402
from obot_watchdog.node import Obot2Watchdog  # noqa: E402


@pytest.fixture(scope='module', autouse=True)
def ros_context():
    rclpy.init()
    yield
    rclpy.shutdown()


@pytest.fixture()
def node(tmp_path):
    overrides = [
        Parameter('state_file', value=str(tmp_path / 'state.json')),
        Parameter('fault_log_path', value=str(tmp_path / 'faults.jsonl')),
        Parameter('can_interface', value='can_does_not_exist'),
    ]
    n = Obot2Watchdog(parameter_overrides=overrides)
    yield n
    n.close()
    n.destroy_node()


class Recorder:
    def __init__(self):
        self.msgs = []

    def publish(self, msg):
        self.msgs.append(msg)


def test_defaults_match_obot2_spec(node):
    assert node.dry_run is True
    assert node.cfg.automatic_recovery_enabled is False
    assert (node.cfg.left_node_id, node.cfg.right_node_id) == (61, 62)
    assert node.cfg.robot_name == 'OBot-2'
    assert node.p['deadman_button'] == 4
    assert node.p['motion_inhibit_topic'] == '/cmd_vel_watchdog'
    assert node.p['notification_topic'] == '/obot/watchdog_event'
    assert node.p['can_bitrate'] == 1000000
    assert node.cfg.max_recoveries == 3
    assert node.cfg.observation_window == 60.0


def test_starts_inhibited_and_publishes_zero_stream_and_lock(node):
    node.pub_lock, node.pub_inhibit = Recorder(), Recorder()
    node._tick()
    assert node.core.inhibit
    assert node.pub_lock.msgs[-1].data is True
    twist = node.pub_inhibit.msgs[-1]
    assert twist.twist.linear.x == 0.0 and twist.twist.angular.z == 0.0
    assert twist.header.stamp.sec or twist.header.stamp.nanosec


def test_notification_publish_failure_is_contained(node):
    class Boom:
        def publish(self, msg):
            raise RuntimeError('DDS down')
    node.pub_notify = Boom()
    node._execute(Notify(CRITICAL, 'OBot-2: test'))
    node.pub_lock = Recorder()
    node._tick()
    assert node.core.inhibit
    assert node.pub_lock.msgs[-1].data is True


def test_notification_text_names_robot_and_level(node):
    node.pub_notify = Recorder()
    node._execute(Notify(CRITICAL, 'OBot-2: lockout'))
    assert node.pub_notify.msgs[-1].data == '[CRITICAL] OBot-2: lockout'


def test_dry_run_blocks_state_changing_frames_only(node):
    sent = []
    node.can.send = lambda i, d, r: sent.append((i, d, r)) or None
    node._send(Frame('clear_errors', 'left', 0x7B8, b'', False, True))
    assert sent == []
    node._send(Frame('get_error', 'left', 0x7A3, b'', True, False))
    assert sent == [(0x7A3, b'', True)]
    node.dry_run = False
    node._send(Frame('clear_errors', 'left', 0x7B8, b'', False, True))
    assert sent[-1] == (0x7B8, b'', False)


def test_joy_button_out_of_range_cannot_confirm_release(node):
    node._on_joy(SimpleNamespace(buttons=[0, 0]))
    assert node.f1 is None
    node._on_joy(SimpleNamespace(buttons=[0, 0, 0, 0, 1]))
    assert node.f1 is True
    node._on_joy(SimpleNamespace(buttons=[0, 0, 0, 0, 0]))
    assert node.f1 is False


def test_internal_error_fails_closed(node):
    node.core.inhibit = False
    node.core.tick = lambda now, snap: 1 / 0
    node.pub_lock = Recorder()
    node._tick()
    assert node.core.inhibit
    assert node.pub_lock.msgs[-1].data is True
