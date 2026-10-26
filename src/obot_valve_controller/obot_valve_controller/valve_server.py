"""Robot-side ROS 2 spray action server."""

import math
import threading
import time

from obot_valve_controller.gpio_backend import create_gpio_output

from obot_valve_interfaces.action import SprayValve
from obot_valve_interfaces.msg import ValveCompletion

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)

from std_msgs.msg import Bool


MAX_SPRAY_DURATION_SEC = 300.0


class ValveServer(Node):
    """Serve exclusive spray actions and command the valve GPIO."""

    def __init__(self, gpio_output_factory=None, parameter_overrides=None):
        """Initialize parameters, safe GPIO output, topics, and action."""
        super().__init__(
            'valve_server',
            parameter_overrides=parameter_overrides,
        )

        self.declare_parameter('circle_duration', 1.0)
        self.declare_parameter('square_duration', 3.0)
        self.declare_parameter('triangle_duration', 5.0)
        self.declare_parameter('gpio_chip', '/dev/gpiochip4')
        self.declare_parameter('gpio_pin', 22)
        self.declare_parameter('active_high', True)
        self.declare_parameter('feedback_rate', 10.0)
        self.declare_parameter('mock_gpio', False)

        self._durations = {
            0: self._duration_parameter('circle_duration'),
            1: self._duration_parameter('square_duration'),
            2: self._duration_parameter('triangle_duration'),
        }
        self._gpio_chip = str(self.get_parameter('gpio_chip').value)
        self._gpio_pin = int(self.get_parameter('gpio_pin').value)
        self._active_high = bool(self.get_parameter('active_high').value)
        self._feedback_rate = float(
            self.get_parameter('feedback_rate').value
        )
        mock_gpio = bool(self.get_parameter('mock_gpio').value)
        self._validate_parameters()

        self._lock = threading.RLock()
        self._reservation_changed = threading.Condition(self._lock)
        self._reserved = False
        self._shutting_down = False
        self._valve_open = False

        status_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        completion_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        self._state_publisher = self.create_publisher(
            Bool,
            '/obot/valve/state',
            status_qos,
        )
        self._completion_publisher = self.create_publisher(
            ValveCompletion,
            '/obot/valve/completion',
            completion_qos,
        )

        factory = gpio_output_factory or create_gpio_output
        self._gpio = factory(
            self._gpio_chip,
            self._gpio_pin,
            self._active_high,
            mock_gpio,
        )
        try:
            self._command_valve(False)
            self._state_timer = self.create_timer(
                0.1,
                self._publish_valve_state,
            )
            self._callback_group = ReentrantCallbackGroup()
            self._action_server = ActionServer(
                self,
                SprayValve,
                '/obot/spray',
                execute_callback=self._execute_callback,
                goal_callback=self._goal_callback,
                cancel_callback=self._cancel_callback,
                callback_group=self._callback_group,
            )
        except Exception:
            try:
                self._gpio.set_open(False)
            except Exception as exc:
                self.get_logger().error(
                    f'Failed to command valve OFF during initialization: {exc}'
                )
            try:
                self._gpio.close()
            except Exception as exc:
                self.get_logger().error(
                    f'Failed to release GPIO after initialization: {exc}'
                )
            raise

        if mock_gpio:
            self.get_logger().warning(
                'Using mock GPIO; no physical relay is controlled'
            )
        else:
            self.get_logger().info(
                f'Valve output ready on {self._gpio_chip} line '
                f'{self._gpio_pin}'
            )

    def _duration_parameter(self, name):
        value = float(self.get_parameter(name).value)
        if (
            not math.isfinite(value)
            or not 0.0 <= value <= MAX_SPRAY_DURATION_SEC
        ):
            raise ValueError(
                f'{name} must be finite and between 0 and '
                f'{MAX_SPRAY_DURATION_SEC} seconds'
            )
        return value

    def _validate_parameters(self):
        if not self._gpio_chip:
            raise ValueError('gpio_chip must not be empty')
        if self._gpio_pin < 0:
            raise ValueError('gpio_pin must be nonnegative')
        if (
            not math.isfinite(self._feedback_rate)
            or not 0.1 <= self._feedback_rate <= 100.0
        ):
            raise ValueError('feedback_rate must be between 0.1 and 100 Hz')

    def _goal_callback(self, request):
        mode = int(request.spray_mode)
        if mode not in self._durations:
            self.get_logger().warning(f'Rejecting invalid spray mode {mode}')
            return GoalResponse.REJECT

        with self._lock:
            if self._shutting_down:
                self.get_logger().warning(
                    'Rejecting spray goal while shutting down'
                )
                return GoalResponse.REJECT
            if self._reserved:
                self.get_logger().warning(
                    'Rejecting spray goal because another goal is active'
                )
                return GoalResponse.REJECT
            self._reserved = True
        return GoalResponse.ACCEPT

    def _cancel_callback(self, _goal_handle):
        return CancelResponse.ACCEPT

    def _command_valve(self, opened):
        with self._lock:
            if opened and self._shutting_down:
                raise RuntimeError('Cannot open valve while shutting down')
            if opened:
                self._gpio.set_open(True)
                self._valve_open = True
            else:
                try:
                    self._gpio.set_open(False)
                finally:
                    self._valve_open = False

    def _is_valve_open(self):
        with self._lock:
            return self._valve_open

    def _publish_valve_state(self):
        message = Bool()
        message.data = self._is_valve_open()
        self._state_publisher.publish(message)

    def _publish_completion(self, mode, outcome, actual_duration):
        message = ValveCompletion()
        message.spray_mode = mode
        message.outcome = outcome
        message.actual_duration = actual_duration
        self._completion_publisher.publish(message)

    def _execute_callback(self, goal_handle):
        mode = int(goal_handle.request.spray_mode)
        duration = self._durations.get(mode)
        outcome = ValveCompletion.ABORTED
        result = SprayValve.Result()
        start_time = None
        message = 'Spray action aborted'

        try:
            if duration is None:
                raise ValueError(f'Invalid spray mode {mode}')
            with self._lock:
                if self._shutting_down:
                    raise RuntimeError('Valve server is shutting down')

            self._command_valve(True)
            start_time = time.monotonic()
            deadline = start_time + duration
            feedback_period = 1.0 / self._feedback_rate
            poll_period = min(0.05, feedback_period)
            next_feedback_time = start_time

            while True:
                if self._is_shutting_down():
                    message = 'Valve server is shutting down'
                    break
                if goal_handle.is_cancel_requested:
                    outcome = ValveCompletion.CANCELED
                    message = 'Spray canceled'
                    break

                now = time.monotonic()
                if now >= deadline:
                    outcome = ValveCompletion.SUCCEEDED
                    message = 'Spray completed'
                    break

                if now >= next_feedback_time:
                    feedback = SprayValve.Feedback()
                    feedback.elapsed_time = now - start_time
                    feedback.remaining_time = max(0.0, deadline - now)
                    feedback.valve_open = self._is_valve_open()
                    goal_handle.publish_feedback(feedback)
                    next_feedback_time = now + feedback_period
                time.sleep(min(poll_period, deadline - now))

        except Exception as exc:
            message = f'Spray execution failed: {exc}'
            self.get_logger().error(message)

        try:
            self._command_valve(False)
        except Exception as exc:
            outcome = ValveCompletion.ABORTED
            message = f'{message}; failed to command valve OFF: {exc}'
            self.get_logger().error(message)

        actual_duration = (
            0.0
            if start_time is None
            else max(0.0, time.monotonic() - start_time)
        )
        result.completed = outcome == ValveCompletion.SUCCEEDED
        result.canceled = outcome == ValveCompletion.CANCELED
        result.actual_duration = actual_duration
        result.message = message

        try:
            if outcome == ValveCompletion.SUCCEEDED:
                goal_handle.succeed()
            elif outcome == ValveCompletion.CANCELED:
                goal_handle.canceled()
            else:
                goal_handle.abort()
        except Exception as exc:
            outcome = ValveCompletion.ABORTED
            result.completed = False
            result.canceled = False
            result.message = f'{message}; failed to set terminal state: {exc}'
            self.get_logger().error(result.message)

        try:
            self._publish_completion(mode, outcome, actual_duration)
        except Exception as exc:
            self.get_logger().error(
                f'Failed to publish spray completion event: {exc}'
            )
        finally:
            with self._reservation_changed:
                self._reserved = False
                self._reservation_changed.notify_all()

        return result

    def _is_shutting_down(self):
        with self._lock:
            return self._shutting_down

    def destroy_node(self):
        """Command OFF, wait briefly for execution, and release GPIO."""
        with self._reservation_changed:
            self._shutting_down = True
        try:
            self._command_valve(False)
        except Exception as exc:
            self.get_logger().error(
                f'Failed to command valve OFF during shutdown: {exc}'
            )

        with self._reservation_changed:
            finished = self._reservation_changed.wait_for(
                lambda: not self._reserved,
                timeout=2.0,
            )
        if not finished:
            self.get_logger().error(
                'Active spray did not stop before GPIO release timeout'
            )
        try:
            self._gpio.close()
        except Exception as exc:
            self.get_logger().error(
                f'Failed to release GPIO line during shutdown: {exc}'
            )
        return super().destroy_node()


def main(args=None):
    """Run the valve action server in a multithreaded ROS executor."""
    rclpy.init(args=args)
    node = None
    executor = MultiThreadedExecutor(num_threads=4)
    try:
        node = ValveServer()
        executor.add_node(node)
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        executor.shutdown()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
