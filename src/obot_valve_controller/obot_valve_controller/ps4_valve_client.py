"""Laptop-side joystick action client and commanded-state rumble driver."""

import math
import time

from action_msgs.msg import GoalStatus

from obot_valve_controller.client_logic import (
    ButtonEdgeDetector,
    ValveRumbleState,
)

from obot_valve_interfaces.action import SprayValve

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)

from sensor_msgs.msg import Joy, JoyFeedback, JoyFeedbackArray

from std_msgs.msg import Bool


class PS4ValveClient(Node):
    """Map Joy button edges to spray goals and valve state to rumble."""

    def __init__(self, parameter_overrides=None):
        """Initialize controls, action client, and subscriptions."""
        super().__init__(
            'ps4_valve_client',
            parameter_overrides=parameter_overrides,
        )

        self.declare_parameter('joy_topic', '/joy')
        self.declare_parameter('action_name', '/obot/spray')
        self.declare_parameter('valve_state_topic', '/obot/valve/state')
        self.declare_parameter('feedback_topic', '/joy/set_feedback')
        self.declare_parameter('rumble_intensity', 0.5)
        self.declare_parameter('valve_feedback_timeout', 0.6)
        self.declare_parameter('rumble_refresh_period', 0.2)
        self.declare_parameter('circle_button', -1)
        self.declare_parameter('square_button', -1)
        self.declare_parameter('triangle_button', -1)
        self.declare_parameter('cross_button', -1)

        self._joy_topic = str(self.get_parameter('joy_topic').value)
        self._state_topic = str(
            self.get_parameter('valve_state_topic').value
        )
        self._feedback_topic = str(
            self.get_parameter('feedback_topic').value
        )
        self._rumble_intensity = float(
            self.get_parameter('rumble_intensity').value
        )
        self._state_timeout = float(
            self.get_parameter('valve_feedback_timeout').value
        )
        self._rumble_refresh_period = float(
            self.get_parameter('rumble_refresh_period').value
        )
        self._validate_parameters()

        button_indices = {
            'circle': int(self.get_parameter('circle_button').value),
            'square': int(self.get_parameter('square_button').value),
            'triangle': int(self.get_parameter('triangle_button').value),
            'cross': int(self.get_parameter('cross_button').value),
        }
        configured_indices = [
            index for index in button_indices.values() if index >= 0
        ]
        if len(configured_indices) != len(set(configured_indices)):
            raise ValueError('Configured face-button indices must be unique')
        self._button_edges = ButtonEdgeDetector(button_indices)
        if any(index < 0 for index in button_indices.values()):
            self.get_logger().warning(
                'One or more face-button indices are unset; set the '
                'circle_button, square_button, triangle_button, and '
                'cross_button parameters using the live /joy message layout'
            )

        self._rumble = ValveRumbleState(
            self._rumble_intensity,
            self._state_timeout,
            self._rumble_refresh_period,
        )
        self._action_client = ActionClient(
            self,
            SprayValve,
            str(self.get_parameter('action_name').value),
        )
        self._active = False
        self._goal_handle = None
        self._cancel_pending = False
        self._cancel_requested = False
        self._invalid_index_warnings = set()

        state_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._feedback_publisher = self.create_publisher(
            JoyFeedbackArray,
            self._feedback_topic,
            10,
        )
        self._joy_subscription = self.create_subscription(
            Joy,
            self._joy_topic,
            self._on_joy,
            10,
        )
        self._state_subscription = self.create_subscription(
            Bool,
            self._state_topic,
            self._on_valve_state,
            state_qos,
        )
        self._rumble_timer = self.create_timer(
            min(0.05, self._rumble_refresh_period / 2.0),
            self._refresh_rumble,
        )

    def _validate_parameters(self):
        if (
            not math.isfinite(self._rumble_intensity)
            or not 0.0 <= self._rumble_intensity <= 1.0
        ):
            raise ValueError('rumble_intensity must be between 0.0 and 1.0')
        if (
            not math.isfinite(self._state_timeout)
            or not 0.0 < self._state_timeout <= 60.0
        ):
            raise ValueError(
                'valve_feedback_timeout must be greater than zero and '
                'no more than 60 seconds'
            )
        if (
            not math.isfinite(self._rumble_refresh_period)
            or not 0.0 < self._rumble_refresh_period <= 5.0
        ):
            raise ValueError(
                'rumble_refresh_period must be greater than zero and '
                'no more than 5 seconds'
            )

    def _on_joy(self, message):
        edges = self._button_edges.pressed_edges(message.buttons)
        for name, index in self._button_edges.button_indices.items():
            if (
                index >= len(message.buttons)
                and index >= 0
                and name not in self._invalid_index_warnings
            ):
                self.get_logger().warning(
                    f'{name}_button index {index} is absent from /joy; '
                    'check the joystick mapping'
                )
                self._invalid_index_warnings.add(name)

        if 'cross' in edges:
            self._request_cancel()
        if self._active:
            return

        modes = (
            ('circle', 0),
            ('square', 1),
            ('triangle', 2),
        )
        for name, mode in modes:
            if name in edges:
                self._start_spray(mode)
                break

    def _start_spray(self, mode):
        if self._active:
            return
        if not self._action_client.server_is_ready():
            self.get_logger().warning(
                'Spray action server is unavailable; ignoring button press'
            )
            return

        goal = SprayValve.Goal()
        goal.spray_mode = mode
        self._active = True
        self._goal_handle = None
        self._cancel_pending = False
        self._cancel_requested = False
        try:
            future = self._action_client.send_goal_async(goal)
            future.add_done_callback(self._on_goal_response)
        except Exception as exc:
            self._clear_active_goal()
            self.get_logger().error(f'Failed to send spray goal: {exc}')

    def _on_goal_response(self, future):
        try:
            goal_handle = future.result()
        except Exception as exc:
            self.get_logger().error(f'Spray goal request failed: {exc}')
            self._clear_active_goal()
            return

        if not goal_handle.accepted:
            self.get_logger().warning('Spray goal was rejected by the robot')
            self._clear_active_goal()
            return

        self._goal_handle = goal_handle
        self.get_logger().info('Spray goal accepted')
        try:
            result_future = goal_handle.get_result_async()
            result_future.add_done_callback(self._on_result)
        except Exception as exc:
            self.get_logger().error(
                f'Failed to request spray result; input remains locked: {exc}'
            )
            return
        if self._cancel_pending:
            self._send_cancel_request()

    def _request_cancel(self):
        if not self._active:
            return
        if self._goal_handle is None:
            self._cancel_pending = True
            return
        self._send_cancel_request()

    def _send_cancel_request(self):
        if self._goal_handle is None or self._cancel_requested:
            return
        self._cancel_requested = True
        try:
            future = self._goal_handle.cancel_goal_async()
            future.add_done_callback(self._on_cancel_response)
        except Exception as exc:
            self._cancel_requested = False
            self.get_logger().error(
                f'Failed to request spray cancellation: {exc}'
            )

    def _on_cancel_response(self, future):
        try:
            response = future.result()
        except Exception as exc:
            self._cancel_requested = False
            self.get_logger().error(
                f'Spray cancellation request failed: {exc}'
            )
            return
        if not response.goals_canceling:
            self._cancel_requested = False
            self.get_logger().warning(
                'Robot did not accept the spray cancellation request'
            )

    def _on_result(self, future):
        try:
            wrapped_result = future.result()
        except Exception as exc:
            self.get_logger().error(
                'Failed to receive spray result; input remains locked because '
                f'the terminal state is unknown: {exc}'
            )
            return

        status = wrapped_result.status
        result = wrapped_result.result
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info(result.message)
        elif status == GoalStatus.STATUS_CANCELED:
            self.get_logger().info('Spray action canceled')
        else:
            self.get_logger().error(
                'Spray action ended with status '
                f'{status}: {result.message}'
            )
        self._clear_active_goal()

    def _clear_active_goal(self):
        self._active = False
        self._goal_handle = None
        self._cancel_pending = False
        self._cancel_requested = False

    def _on_valve_state(self, message):
        self._rumble.receive_state(message.data, time.monotonic())
        self._publish_rumble_if_changed()

    def _refresh_rumble(self):
        self._publish_rumble_if_changed()

    def _publish_rumble_if_changed(self):
        intensity = self._rumble.rumble_output(time.monotonic())
        if intensity is not None:
            self._publish_rumble(intensity)

    def _publish_rumble(self, intensity):
        feedback = JoyFeedback()
        feedback.type = JoyFeedback.TYPE_RUMBLE
        feedback.id = 0
        feedback.intensity = float(intensity)
        message = JoyFeedbackArray()
        message.array = [feedback]
        self._feedback_publisher.publish(message)

    def destroy_node(self):
        """Send zero rumble feedback before destroying the node."""
        intensity = self._rumble.stop(time.monotonic())
        if intensity is not None:
            self._publish_rumble(intensity)
        return super().destroy_node()


def main(args=None):
    """Run the joystick action client."""
    rclpy.init(args=args)
    node = None
    try:
        node = PS4ValveClient()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
