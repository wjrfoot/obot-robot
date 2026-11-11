"""ROS 2 wrapper for the independent OBot-2 ODrive fault watchdog."""

from datetime import datetime, timezone
import json
import os
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from . import can_io
from .core import Config, Core, Frame, Log, Notify, Snapshot


DIAGNOSTIC_STAGGER_S = 0.05
DIAGNOSTIC_TIMER_PERIOD_S = 0.01

PARAMS = {
    'can_interface': 'can0',
    'can_bitrate': 1000000,
    'left_node_id': 61,
    'right_node_id': 62,
    'robot_name': 'OBot-2',
    'monitor_interval': 1.0,
    'telemetry_stale_s': 2.0,
    'get_error_miss_limit': 5,
    'recoverable_error_mask': 0x100,
    'max_recoveries': 3,
    'retry_interval_s': 5.0,
    'verify_timeout_s': 3.0,
    'command_gap_s': 0.5,
    'notification_topic': '/obot/watchdog_event',
    'fault_log_path': '~/obot_logs/obot2_watchdog_faults.jsonl',
    'dry_run': False,
}


def _expand(path):
    return os.path.abspath(os.path.expanduser(path))


class FaultLog:
    """Append-only diagnostic log with an fsync per event."""

    def __init__(self, path):
        self.path = _expand(path)

    def write(self, record):
        os.makedirs(os.path.dirname(self.path), exist_ok=True)
        with open(self.path, 'a') as log_file:
            log_file.write(json.dumps(record, default=str) + '\n')
            log_file.flush()
            os.fsync(log_file.fileno())


class Obot2Watchdog(Node):
    """Monitor two S1 controllers and recover each independently."""

    def __init__(self, **node_kwargs):
        super().__init__('obot2_watchdog', **node_kwargs)
        for name, default in PARAMS.items():
            self.declare_parameter(name, default)
        self.p = {key: self.get_parameter(key).value for key in PARAMS}
        self.dry_run = bool(self.p['dry_run'])
        self.cfg = Config(
            robot_name=self.p['robot_name'],
            left_node_id=int(self.p['left_node_id']),
            right_node_id=int(self.p['right_node_id']),
            monitor_interval=float(self.p['monitor_interval']),
            telemetry_stale_s=float(self.p['telemetry_stale_s']),
            get_error_miss_limit=int(self.p['get_error_miss_limit']),
            recoverable_error_mask=int(self.p['recoverable_error_mask']),
            max_recoveries=int(self.p['max_recoveries']),
            retry_interval_s=float(self.p['retry_interval_s']),
            verify_timeout_s=float(self.p['verify_timeout_s']),
            command_gap_s=float(self.p['command_gap_s']),
        )
        self.core = Core(self.cfg, now0=time.monotonic())
        self.fault_log = FaultLog(self.p['fault_log_path'])

        left, right = can_io.make_nodes(
            self.cfg.left_node_id, self.cfg.right_node_id)
        self.can = can_io.CanTelemetry(
            self.p['can_interface'], left, right, self.get_logger())
        self.poller = can_io.CanStatePoller(
            self.can, self.p['can_interface'], self.cfg.monitor_interval,
            int(self.p['can_bitrate']))
        self.pub_notify = self.create_publisher(
            String, self.p['notification_topic'], 10)
        self.pub_state = self.create_publisher(
            String, '/obot2/watchdog/state', 10)

        try:
            self.can.start()
        except OSError as exc:
            self.get_logger().error(
                f'Cannot open {self.p["can_interface"]}: {exc}')
        self.poller.start()

        self.create_timer(0.1, self._tick)
        self.create_timer(1.0, self._publish_state)
        self._diagnostic_timer = self.create_timer(
            DIAGNOSTIC_TIMER_PERIOD_S, self._flush_diagnostic_request)
        self._pending_diagnostic = None
        self._left_get_error_sent_at = None
        self.get_logger().info(
            f'{self.cfg.robot_name} watchdog started: '
            f'dry_run={self.dry_run}, automatic_recovery_enabled=True, '
            f'nodes={self.cfg.left_node_id}/{self.cfg.right_node_id}')

    def _snapshot(self):
        left, right, can_state, can_time, can_seq = (
            self.can.snapshot_nodes())
        return Snapshot(
            left=left, right=right, can_state=can_state,
            can_time=can_time, can_seq=can_seq)

    def _tick(self):
        try:
            actions = self.core.tick(time.monotonic(), self._snapshot())
        except Exception as exc:
            self.get_logger().error(f'Watchdog tick failed: {exc}')
            return
        for action in actions:
            self._execute(action)

    def _publish_state(self):
        self.pub_state.publish(String(data=self.core.state.value))

    def _execute(self, action):
        if isinstance(action, Notify):
            self._notify(action)
        elif isinstance(action, Frame):
            if action.label == 'get_error' and action.node == 'left':
                self._send(action)
                self._left_get_error_sent_at = time.monotonic()
            elif action.label == 'get_error' and action.node == 'right':
                if self._pending_diagnostic is not None:
                    pending, _ = self._pending_diagnostic
                    self.get_logger().warning(
                        'Overlapping right Get_Error request; sending the '
                        'outstanding diagnostic request immediately')
                    self._pending_diagnostic = None
                    self._send(pending)
                stagger_from = self._left_get_error_sent_at
                if stagger_from is None:
                    stagger_from = time.monotonic()
                self._pending_diagnostic = (
                    action, stagger_from + DIAGNOSTIC_STAGGER_S)
                self._left_get_error_sent_at = None
            else:
                self._send(action)
        elif isinstance(action, Log):
            self._record(action.record)

    def _flush_diagnostic_request(self):
        if self._pending_diagnostic is None:
            return
        frame, send_at = self._pending_diagnostic
        if time.monotonic() < send_at:
            return
        self._pending_diagnostic = None
        self._send(frame)

    def _notify(self, action):
        text = f'[{action.level}] {action.text}'
        if action.level == 'CRITICAL':
            self.get_logger().error(text)
        elif action.level == 'WARNING':
            self.get_logger().warning(text)
        else:
            self.get_logger().info(text)
        try:
            self.pub_notify.publish(String(data=text))
        except Exception as exc:
            self.get_logger().error(f'Notification publish failed: {exc}')

    def _send(self, frame):
        if frame.mutating and self.dry_run:
            self.get_logger().warning(
                f'DRY RUN: would send {frame.label} to {frame.node} '
                f'(0x{frame.can_id:03X}#{frame.data.hex()})')
            return
        error = self.can.send(frame.can_id, frame.data, frame.rtr)
        if error:
            self.get_logger().warning(
                f'CAN send {frame.label} to {frame.node} failed: {error}')

    def _record(self, record):
        event = dict(record)
        event['timestamp'] = datetime.now(timezone.utc).isoformat()
        event['dry_run'] = self.dry_run
        self.get_logger().info('watchdog_event ' + json.dumps(
            event, default=str))
        try:
            self.fault_log.write(event)
        except OSError as exc:
            self.get_logger().error(f'Cannot write fault log: {exc}')

    def close(self):
        self._diagnostic_timer.cancel()
        self._pending_diagnostic = None
        self.poller.stop()
        self.can.stop()


def main():
    rclpy.init()
    node = Obot2Watchdog()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
