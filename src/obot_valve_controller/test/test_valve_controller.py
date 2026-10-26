"""Hardware-free tests for valve actions, Joy edges, and rumble."""

import sys
import time
from concurrent.futures import ThreadPoolExecutor
from types import SimpleNamespace

from obot_valve_controller.client_logic import (
    ButtonEdgeDetector,
    ValveRumbleState,
)
from obot_valve_controller.gpio_backend import (
    GpiodGPIOOutput,
    MockGPIOOutput,
)
from obot_valve_controller.ps4_valve_client import PS4ValveClient
from obot_valve_controller.valve_server import ValveServer

from obot_valve_interfaces.action import SprayValve
from obot_valve_interfaces.msg import ValveCompletion

import pytest

import rclpy
from rclpy.action import GoalResponse
from rclpy.parameter import Parameter


@pytest.fixture(scope='module', autouse=True)
def ros_context():
    """Initialize one ROS context for the test module."""
    if not rclpy.ok():
        rclpy.init()
    yield
    if rclpy.ok():
        rclpy.shutdown()


class FakeGoalHandle:
    """Provide the goal-handle surface needed by the execute callback."""

    def __init__(self, mode, feedback_exception=None):
        """Initialize a fake handle for the requested spray mode."""
        self.request = SimpleNamespace(spray_mode=mode)
        self.is_cancel_requested = False
        self.feedback = []
        self.terminal_state = None
        self.feedback_exception = feedback_exception

    def publish_feedback(self, message):
        """Capture feedback or raise the configured execution failure."""
        if self.feedback_exception:
            raise self.feedback_exception
        self.feedback.append(message)

    def succeed(self):
        """Record the successful terminal state."""
        self.terminal_state = 'succeeded'

    def canceled(self):
        """Record the canceled terminal state."""
        self.terminal_state = 'canceled'

    def abort(self):
        """Record the aborted terminal state."""
        self.terminal_state = 'aborted'


def make_server(duration=0.08, gpio=None, feedback_rate=50.0):
    """Create a server with an injected mock GPIO backend."""
    gpio = gpio or MockGPIOOutput('/dev/mock', 22, True)
    parameters = [
        Parameter('circle_duration', value=duration),
        Parameter('square_duration', value=0.12),
        Parameter('triangle_duration', value=0.16),
        Parameter('feedback_rate', value=feedback_rate),
        Parameter('mock_gpio', value=True),
    ]
    node = ValveServer(
        gpio_output_factory=lambda _chip, _pin, _high, _mock: gpio,
        parameter_overrides=parameters,
    )
    return node, gpio


def tear_down_node(node):
    """Destroy a test node and release its mock GPIO output."""
    node.destroy_node()


def test_timed_spray_uses_mode_duration_and_completes_once(monkeypatch):
    """Complete a timed goal with its configured mode duration."""
    node, gpio = make_server()
    completions = []
    monkeypatch.setattr(
        node,
        '_publish_completion',
        lambda *args: completions.append(args),
    )
    try:
        assert node._durations == {0: 0.08, 1: 0.12, 2: 0.16}
        goal = FakeGoalHandle(1)
        result = node._execute_callback(goal)
        assert result.completed
        assert not result.canceled
        assert goal.terminal_state == 'succeeded'
        assert result.actual_duration >= 0.12
        assert not gpio.is_open
        assert completions == [(1, ValveCompletion.SUCCEEDED,
                                result.actual_duration)]
    finally:
        tear_down_node(node)


def test_cancel_closes_gpio_and_publishes_one_completion(monkeypatch):
    """Stop a running goal promptly when its cancel flag is set."""
    node, gpio = make_server(duration=1.0, feedback_rate=0.1)
    completions = []
    monkeypatch.setattr(
        node,
        '_publish_completion',
        lambda *args: completions.append(args),
    )
    goal = FakeGoalHandle(0)
    try:
        with ThreadPoolExecutor(max_workers=1) as pool:
            future = pool.submit(node._execute_callback, goal)
            deadline = time.monotonic() + 1.0
            while not gpio.is_open and time.monotonic() < deadline:
                time.sleep(0.005)
            assert gpio.is_open
            cancel_time = time.monotonic()
            goal.is_cancel_requested = True
            result = future.result(timeout=1.0)

        assert result.canceled
        assert time.monotonic() - cancel_time < 0.3
        assert goal.terminal_state == 'canceled'
        assert not gpio.is_open
        assert len(completions) == 1
        assert completions[0][1] == ValveCompletion.CANCELED
    finally:
        tear_down_node(node)


def test_overlapping_and_invalid_goals_are_rejected():
    """Reject concurrent goals and modes outside the supported range."""
    node, _gpio = make_server()
    try:
        assert node._goal_callback(SprayValve.Goal(spray_mode=0)) == (
            GoalResponse.ACCEPT
        )
        assert node._goal_callback(SprayValve.Goal(spray_mode=1)) == (
            GoalResponse.REJECT
        )
        assert node._goal_callback(SprayValve.Goal(spray_mode=9)) == (
            GoalResponse.REJECT
        )
        node._execute_callback(FakeGoalHandle(0))
        assert node._goal_callback(SprayValve.Goal(spray_mode=2)) == (
            GoalResponse.ACCEPT
        )
        node._execute_callback(FakeGoalHandle(2))
    finally:
        tear_down_node(node)


def test_exception_closes_gpio_and_emits_one_aborted_completion(monkeypatch):
    """Abort and close GPIO after an exception during execution."""
    node, gpio = make_server(duration=0.5)
    completions = []
    monkeypatch.setattr(
        node,
        '_publish_completion',
        lambda *args: completions.append(args),
    )
    goal = FakeGoalHandle(0, feedback_exception=RuntimeError('test failure'))
    try:
        result = node._execute_callback(goal)
        assert not result.completed
        assert not result.canceled
        assert goal.terminal_state == 'aborted'
        assert not gpio.is_open
        assert len(completions) == 1
        assert completions[0][1] == ValveCompletion.ABORTED
    finally:
        tear_down_node(node)


def test_gpio_is_off_on_shutdown():
    """Command GPIO OFF and release the line on node destruction."""
    node, gpio = make_server()
    node._command_valve(True)
    node.destroy_node()
    assert not gpio.is_open
    assert gpio.closed


@pytest.mark.parametrize(
    ('active_high', 'off_value', 'open_value'),
    [(True, 0, 1), (False, 1, 0)],
)
def test_gpiod_backend_requests_one_line_and_maps_polarity(
    monkeypatch,
    active_high,
    off_value,
    open_value,
):
    """Use the Ubuntu libgpiod 1.x API and honor output polarity."""
    calls = []

    class FakeLine:
        def request(self, **kwargs):
            calls.append(('request', kwargs))

        def set_value(self, value):
            calls.append(('set_value', value))

        def release(self):
            calls.append(('release',))

    class FakeChip:
        def __init__(self, path):
            calls.append(('chip', path))

        def get_line(self, offset):
            calls.append(('get_line', offset))
            return FakeLine()

        def close(self):
            calls.append(('chip_close',))

    fake_gpiod = SimpleNamespace(
        Chip=FakeChip,
        LINE_REQ_DIR_OUT=3,
    )
    monkeypatch.setitem(sys.modules, 'gpiod', fake_gpiod)
    backend = GpiodGPIOOutput('/dev/gpiochip4', 22, active_high)
    backend.set_open(True)
    backend.set_open(False)
    backend.close()

    assert calls == [
        ('chip', '/dev/gpiochip4'),
        ('get_line', 22),
        (
            'request',
            {
                'consumer': 'obot-valve-controller',
                'type': 3,
                'default_val': off_value,
            },
        ),
        ('set_value', open_value),
        ('set_value', off_value),
        ('release',),
        ('chip_close',),
    ]


def test_button_rising_edges_only():
    """Emit one edge per press, not for a held button."""
    detector = ButtonEdgeDetector({
        'circle': 0,
        'square': 1,
        'triangle': 2,
        'cross': 3,
    })
    assert detector.pressed_edges([0, 0, 0, 0]) == set()
    assert detector.pressed_edges([1, 0, 0, 0]) == {'circle'}
    assert detector.pressed_edges([1, 0, 0, 0]) == set()
    assert detector.pressed_edges([0, 0, 0, 0]) == set()
    assert detector.pressed_edges([0, 0, 0, 1]) == {'cross'}


def test_rumble_tracks_valve_state_and_timeout():
    """Start, refresh, stop, and time out valve-driven rumble."""
    rumble = ValveRumbleState(0.5, 0.6, 0.2)
    assert rumble.rumble_output(now=10.0) is None
    rumble.receive_state(True, received_at=10.0)
    assert rumble.rumble_output(now=10.0) == 0.5
    assert rumble.rumble_output(now=10.1) is None
    assert rumble.rumble_output(now=10.2) == 0.5
    rumble.receive_state(False, received_at=10.21)
    assert rumble.rumble_output(now=10.21) == 0.0
    assert rumble.rumble_output(now=10.22) is None

    rumble.receive_state(True, received_at=11.0)
    assert rumble.rumble_output(now=11.0) == 0.5
    assert rumble.rumble_output(now=11.61) == 0.0


def test_rumble_stops_when_client_stops():
    """Keep rumble stopped after client shutdown."""
    rumble = ValveRumbleState(0.5, 0.6, 0.2)
    rumble.receive_state(True, received_at=20.0)
    assert rumble.rumble_output(now=20.0) == 0.5
    assert rumble.stop(now=20.1) == 0.0
    assert rumble.rumble_output(now=20.1) is None


def test_client_ignores_spray_buttons_while_goal_is_active(monkeypatch):
    """Ignore additional spray button presses while a goal is active."""
    parameters = [
        Parameter('circle_button', value=0),
        Parameter('square_button', value=1),
        Parameter('triangle_button', value=2),
        Parameter('cross_button', value=3),
    ]
    node = PS4ValveClient(parameter_overrides=parameters)
    starts = []
    monkeypatch.setattr(node, '_start_spray', starts.append)
    node._active = True
    try:
        node._on_joy(SimpleNamespace(buttons=[1, 1, 1, 0]))
        assert starts == []
        assert node._active
    finally:
        node.destroy_node()


def test_cross_requests_cancel_while_waiting_for_goal_acceptance():
    """Remember Cross cancellation while waiting for goal acceptance."""
    parameters = [
        Parameter('circle_button', value=0),
        Parameter('square_button', value=1),
        Parameter('triangle_button', value=2),
        Parameter('cross_button', value=3),
    ]
    node = PS4ValveClient(parameter_overrides=parameters)
    node._active = True
    try:
        node._on_joy(SimpleNamespace(buttons=[0, 0, 0, 1]))
        assert node._cancel_pending
        assert node._active
    finally:
        node.destroy_node()
