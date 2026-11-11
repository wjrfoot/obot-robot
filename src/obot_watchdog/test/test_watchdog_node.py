"""ROS wrapper tests with mocked timers and CAN transport."""

import os
from types import SimpleNamespace

import pytest

os.environ['ROS_DOMAIN_ID'] = '97'
os.environ['ROS_LOCALHOST_ONLY'] = '1'

import rclpy  # noqa: E402
from rclpy.parameter import Parameter  # noqa: E402

from obot_watchdog import node as watchdog_node  # noqa: E402
from obot_watchdog.core import Frame, NodeTel, Notify, CRITICAL  # noqa: E402
from obot_watchdog.node import (  # noqa: E402
    DIAGNOSTIC_STAGGER_S, DIAGNOSTIC_TIMER_PERIOD_S, Obot2Watchdog,
)


@pytest.fixture(scope='module', autouse=True)
def ros_context():
    rclpy.init()
    yield
    rclpy.shutdown()


class Recorder:
    def __init__(self):
        self.msgs = []

    def publish(self, msg):
        self.msgs.append(msg)


class MockTimer:
    def __init__(self, period, callback):
        self.period = period
        self.callback = callback
        self.cancelled = False

    def cancel(self):
        self.cancelled = True


class MockCan:
    def __init__(self, interface, left, right, logger=None):
        self.interface = interface
        self.left = left
        self.right = right
        self.sent = []
        self.send_result = None

    def start(self):
        pass

    def stop(self):
        pass

    def send(self, can_id, data, rtr):
        self.sent.append((can_id, data, rtr))
        return self.send_result

    def snapshot_nodes(self):
        return self.left, self.right, 'ERROR-ACTIVE', 0.0, 0


class MockPoller:
    def __init__(self, *args):
        self.stopped = False

    def start(self):
        pass

    def stop(self):
        self.stopped = True


def make_node(tmp_path, monkeypatch):
    timers = []

    def create_timer(_node, period, callback):
        timer = MockTimer(period, callback)
        timers.append(timer)
        return timer

    monkeypatch.setattr(Obot2Watchdog, 'create_timer', create_timer)
    monkeypatch.setattr(watchdog_node.can_io, 'make_nodes',
                        lambda left, right: (
                            NodeTel('left', left), NodeTel('right', right)))
    monkeypatch.setattr(
        watchdog_node.can_io, 'CanTelemetry', MockCan)
    monkeypatch.setattr(
        watchdog_node.can_io, 'CanStatePoller', MockPoller)
    overrides = [
        Parameter('fault_log_path', value=str(tmp_path / 'faults.jsonl')),
        Parameter('can_interface', value='mock_can'),
    ]
    node = Obot2Watchdog(parameter_overrides=overrides)
    return node, timers


@pytest.fixture()
def node(tmp_path, monkeypatch):
    instance, _ = make_node(tmp_path, monkeypatch)
    yield instance
    instance.close()
    instance.destroy_node()


@pytest.fixture()
def diagnostic_timing(tmp_path, monkeypatch):
    node, timers = make_node(tmp_path, monkeypatch)
    clock = SimpleNamespace(now=0.0)
    monkeypatch.setattr(
        watchdog_node, 'time',
        SimpleNamespace(monotonic=lambda: clock.now))
    diagnostic_timer = next(
        timer for timer in timers
        if timer.period == DIAGNOSTIC_TIMER_PERIOD_S)
    yield node, clock, diagnostic_timer
    node.close()
    node.destroy_node()


def test_configuration_preserves_polling_and_recovery_defaults(node):
    assert node.p['monitor_interval'] == 1.0
    assert node.p['get_error_miss_limit'] == 5
    assert node.dry_run is False
    assert node.cfg.max_recoveries == 3
    assert node.cfg.retry_interval_s == 5.0


def test_watchdog_has_no_joystick_or_velocity_interfaces(node):
    assert node.count_subscribers('/joy') == 0
    assert node.count_publishers('/cmd_vel_watchdog') == 0
    assert node.count_publishers('/cmd_vel_lock') == 0
    assert node.count_publishers('/watchdog_lock') == 0
    assert node.count_publishers('/obot/watchdog_event') == 1


def test_notification_is_published_with_existing_topic_text(node):
    recorder = Recorder()
    node.pub_notify = recorder
    node._execute(Notify(CRITICAL, 'OBot-2: controller fault'))
    assert len(recorder.msgs) == 1
    assert recorder.msgs[0].data == (
        '[CRITICAL] OBot-2: controller fault')


def test_notification_publish_failure_is_logged(node):
    class BrokenPublisher:
        def publish(self, _msg):
            raise RuntimeError('DDS unavailable')

    messages = []
    node.pub_notify = BrokenPublisher()
    node.get_logger = lambda: SimpleNamespace(
        error=messages.append, info=messages.append,
        warning=messages.append)
    node._execute(Notify(CRITICAL, 'fault'))
    assert any('Notification publish failed' in message
               for message in messages)


def test_can_send_error_is_logged(node):
    messages = []
    node.can.send_result = 'bus down'
    node.get_logger = lambda: SimpleNamespace(warning=messages.append)
    node._send(Frame('clear_errors', 'left', 0x7B8, b'', False, True))
    assert any('CAN send clear_errors to left failed: bus down' in message
               for message in messages)


def test_left_get_error_transmits_immediately(diagnostic_timing):
    node, clock, _ = diagnostic_timing
    node._execute(Frame('get_error', 'left', 0x7A3, b'', True, False))
    assert node.can.sent == [(0x7A3, b'', True)]
    assert node._pending_diagnostic is None


def test_right_get_error_transmits_only_after_50_ms(diagnostic_timing):
    node, clock, timer = diagnostic_timing
    node._execute(Frame('get_error', 'left', 0x7A3, b'', True, False))
    clock.now = 0.01
    node._execute(Frame('get_error', 'right', 0x7C3, b'', True, False))
    assert node.can.sent == [(0x7A3, b'', True)]

    clock.now = DIAGNOSTIC_STAGGER_S - 0.001
    timer.callback()
    assert len(node.can.sent) == 1

    clock.now = DIAGNOSTIC_STAGGER_S
    timer.callback()
    assert node.can.sent == [
        (0x7A3, b'', True),
        (0x7C3, b'', True),
    ]
    assert node._pending_diagnostic is None


def test_voltage_requests_are_immediate(diagnostic_timing):
    node, _, _ = diagnostic_timing
    node._execute(Frame('get_voltage', 'left', 0x7B7, b'', True, False))
    node._execute(Frame('get_voltage', 'right', 0x7D7, b'', True, False))
    assert node.can.sent == [
        (0x7B7, b'', True),
        (0x7D7, b'', True),
    ]


def test_recovery_commands_are_immediate(diagnostic_timing):
    node, _, _ = diagnostic_timing
    node._execute(Frame('clear_errors', 'right', 0x7D8, b'', False, True))
    assert node.can.sent == [(0x7D8, b'', False)]


def test_overlapping_right_diagnostics_send_outstanding_request(diagnostic_timing):
    node, clock, timer = diagnostic_timing
    messages = []
    node.get_logger = lambda: SimpleNamespace(warning=messages.append)
    first = Frame('get_error', 'right', 0x7C3, b'', True, False)
    second = Frame('get_error', 'right', 0x7C3, b'', True, False)
    node._execute(first)
    clock.now = 1.0
    node._execute(second)

    assert node.can.sent == [(0x7C3, b'', True)]
    assert len(messages) == 1
    assert 'Overlapping right Get_Error request' in messages[0]

    clock.now = 1.049
    timer.callback()
    assert len(node.can.sent) == 1
    clock.now = 1.05
    timer.callback()
    assert node.can.sent == [
        (0x7C3, b'', True),
        (0x7C3, b'', True),
    ]


def test_shutdown_cancels_timer_and_clears_queued_request(diagnostic_timing):
    node, _, timer = diagnostic_timing
    node._execute(Frame('get_error', 'right', 0x7C3, b'', True, False))
    node.close()
    assert timer.cancelled
    assert node._pending_diagnostic is None
