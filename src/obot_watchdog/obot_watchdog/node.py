"""OBot-2 ODrive S1 fault watchdog ROS 2 node.

Thin ROS wrapper around obot_watchdog.core.Core. All blocking work (CAN
receive, link-state polling) runs in threads; callbacks only copy data.
"""

from datetime import datetime, timezone
import json
import math
import os
import time

from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger

from . import can_io
from .core import (
    Config, Core, Frame, Log, Notify, Persist, Snapshot,
)

CMD_FRESH_S = 0.5
ZERO_EPS = 1e-6


def _expand(path):
    return os.path.abspath(os.path.expanduser(path))


class StateStore:
    """Tiny persistent store so a restart cannot silently clear a fault."""

    def __init__(self, path):
        self.path = _expand(path)

    def load(self):
        try:
            with open(self.path) as fh:
                data = json.load(fh)
            return bool(data.get('fault_active')), data.get('lockout_reason')
        except FileNotFoundError:
            return False, None
        except (OSError, ValueError):
            # Unreadable state is treated as an interrupted fault.
            return True, 'STATE_FILE_UNREADABLE'

    def save(self, fault_active, lockout_reason):
        os.makedirs(os.path.dirname(self.path), exist_ok=True)
        tmp = self.path + '.tmp'
        with open(tmp, 'w') as fh:
            json.dump({'fault_active': fault_active,
                       'lockout_reason': lockout_reason}, fh)
            fh.flush()
            os.fsync(fh.fileno())
        os.replace(tmp, self.path)


class FaultLog:

    def __init__(self, path):
        self.path = _expand(path)

    def write(self, record):
        os.makedirs(os.path.dirname(self.path), exist_ok=True)
        with open(self.path, 'a') as fh:
            fh.write(json.dumps(record, default=str) + '\n')
            fh.flush()
            os.fsync(fh.fileno())


PARAMS = {
    'can_interface': 'can0',
    'can_bitrate': 1000000,
    'left_node_id': 61,
    'right_node_id': 62,
    'robot_name': 'OBot-2',
    'joy_topic': '/joy',
    'deadman_button': 4,
    'monitor_interval': 2.0,
    'observation_window': 60.0,
    'max_recoveries': 3,
    'verify_timeout_s': math.nan,
    'voltage_min_v': math.nan,
    'voltage_normal_max_v': math.nan,
    'voltage_unsafe_v': math.nan,
    'voltage_settle_timeout_s': math.nan,
    'motion_inhibit_topic': '/cmd_vel_watchdog',
    'motion_lock_topic': '/watchdog_lock',
    'notification_topic': '/obot/watchdog_event',
    'mux_output_topic': '/diff_drive_controller/cmd_vel',
    'odom_topic': '/diff_drive_controller/odom',
    'dry_run': True,
    'automatic_recovery_enabled': False,
    'recoverable_error_mask': 0x100,
    'telemetry_stale_s': 2.0,
    'get_error_miss_limit': 5,
    'joy_timeout_s': 1.0,
    'require_voltage_telemetry': True,
    'fault_log_path': '~/obot_logs/obot2_watchdog_faults.jsonl',
    'state_file': '~/obot_logs/obot2_watchdog_state.json',
}


class Obot2Watchdog(Node):

    def __init__(self, **node_kwargs):
        super().__init__('obot2_watchdog', **node_kwargs)
        for name, default in PARAMS.items():
            self.declare_parameter(name, default)
        p = {k: self.get_parameter(k).value for k in PARAMS}
        self.p = p
        self.dry_run = bool(p['dry_run'])

        self.cfg = Config(
            robot_name=p['robot_name'],
            left_node_id=int(p['left_node_id']),
            right_node_id=int(p['right_node_id']),
            monitor_interval=float(p['monitor_interval']),
            telemetry_stale_s=float(p['telemetry_stale_s']),
            get_error_miss_limit=int(p['get_error_miss_limit']),
            joy_timeout_s=float(p['joy_timeout_s']),
            observation_window=float(p['observation_window']),
            max_recoveries=int(p['max_recoveries']),
            automatic_recovery_enabled=bool(p['automatic_recovery_enabled']),
            verify_timeout_s=float(p['verify_timeout_s']),
            voltage_settle_timeout_s=float(p['voltage_settle_timeout_s']),
            voltage_min_v=float(p['voltage_min_v']),
            voltage_normal_max_v=float(p['voltage_normal_max_v']),
            voltage_unsafe_v=float(p['voltage_unsafe_v']),
            recoverable_error_mask=int(p['recoverable_error_mask']),
            require_voltage_telemetry=bool(p['require_voltage_telemetry']),
        )
        now = time.monotonic()
        self.core = Core(self.cfg, now0=now)
        self.store = StateStore(p['state_file'])
        self.fault_log = FaultLog(p['fault_log_path'])

        left, right = can_io.make_nodes(
            self.cfg.left_node_id, self.cfg.right_node_id)
        self.can = can_io.CanTelemetry(
            p['can_interface'], left, right, self.get_logger())
        self.poller = can_io.CanStatePoller(
            self.can, p['can_interface'], self.cfg.monitor_interval,
            int(p['can_bitrate']))

        self.joy_time = None
        self.f1 = None
        self.cmd_time = None
        self.cmd_vel = None
        self.measured_vel = None

        self.pub_notify = self.create_publisher(
            String, p['notification_topic'], 10)
        self.pub_lock = self.create_publisher(Bool, p['motion_lock_topic'], 10)
        self.pub_inhibit = self.create_publisher(
            TwistStamped, p['motion_inhibit_topic'], 10)
        self.pub_state = self.create_publisher(
            String, '/obot2/watchdog/state', 10)
        self.create_subscription(Joy, p['joy_topic'], self._on_joy, 10)
        self.create_subscription(
            TwistStamped, p['mux_output_topic'], self._on_mux_out, 10)
        self.create_subscription(Odometry, p['odom_topic'], self._on_odom, 10)
        self.create_service(
            Trigger, '/obot2/watchdog/manual_reset', self._on_reset)

        fault_active, reason = self.store.load()
        for act in self.core.restore(now, fault_active, reason):
            self._execute(act)

        try:
            self.can.start()
        except OSError as exc:
            self.get_logger().error(
                f'Cannot open {p["can_interface"]}: {exc}. '
                'Motion stays inhibited.')
        self.poller.start()

        self.create_timer(0.1, self._tick)
        self.create_timer(1.0, self._publish_state)
        self.get_logger().warn(
            f'{self.cfg.robot_name} watchdog started: dry_run={self.dry_run}, '
            f'automatic_recovery_enabled='
            f'{self.cfg.automatic_recovery_enabled}, '
            f'nodes={self.cfg.left_node_id}/{self.cfg.right_node_id}, '
            f'f1_button={p["deadman_button"]}')

    # ---- subscriptions -------------------------------------------------
    def _on_joy(self, msg):
        idx = int(self.p['deadman_button'])
        self.joy_time = time.monotonic()
        # A short buttons array cannot confirm release.
        self.f1 = bool(msg.buttons[idx]) if idx < len(msg.buttons) else None

    def _on_mux_out(self, msg):
        self.cmd_time = time.monotonic()
        self.cmd_vel = (msg.twist.linear.x, msg.twist.angular.z)

    def _on_odom(self, msg):
        self.measured_vel = (msg.twist.twist.linear.x,
                             msg.twist.twist.angular.z)

    def _cmd_path_safe(self, now):
        return (self.cmd_time is not None
                and now - self.cmd_time <= CMD_FRESH_S
                and self.cmd_vel is not None
                and all(abs(v) < ZERO_EPS for v in self.cmd_vel))

    def _snapshot(self, now):
        left, right, can_state, can_time, can_seq = self.can.snapshot_nodes()
        f1 = self.f1
        return Snapshot(
            left=left, right=right, can_state=can_state, can_time=can_time,
            can_seq=can_seq, joy_time=self.joy_time,
            f1_pressed=f1 if f1 is not None else None,
            cmd_path_safe=self._cmd_path_safe(now),
            commanded_vel=self.cmd_vel, measured_vel=self.measured_vel)

    # ---- periodic ------------------------------------------------------
    def _tick(self):
        now = time.monotonic()
        try:
            actions = self.core.tick(now, self._snapshot(now))
        except Exception as exc:
            # Fail closed on any internal error.
            self.core.inhibit = True
            self.get_logger().error(f'Watchdog tick failed: {exc}')
            actions = []
        for act in actions:
            self._execute(act)
        self._publish_motion_guard()

    def _publish_motion_guard(self):
        inhibit = self.core.inhibit
        self.pub_lock.publish(Bool(data=inhibit))
        if inhibit:
            msg = TwistStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'base_link'
            self.pub_inhibit.publish(msg)

    def _publish_state(self):
        text = f'{self.core.state.value} inhibit={self.core.inhibit}'
        if self.core.lockout_reason:
            text += f' lockout={self.core.lockout_reason}'
        self.pub_state.publish(String(data=text))

    # ---- action execution ----------------------------------------------
    def _execute(self, act):
        if isinstance(act, Notify):
            self._notify(act)
        elif isinstance(act, Frame):
            self._send(act)
        elif isinstance(act, Log):
            self._record(act.record)
        elif isinstance(act, Persist):
            try:
                self.store.save(act.fault_active, act.lockout_reason)
            except OSError as exc:
                self.get_logger().error(f'Cannot persist state: {exc}')

    def _notify(self, act):
        text = f'[{act.level}] {act.text}'
        try:
            if act.level == 'CRITICAL':
                self.get_logger().error(text)
            elif act.level == 'WARNING':
                self.get_logger().warn(text)
            else:
                self.get_logger().info(text)
        except Exception as exc:
            print(f'Watchdog notification logging failed: {exc}',
                  flush=True)

        try:
            self.pub_notify.publish(String(data=text))
        except Exception as exc:
            self.get_logger().error(f'Notification publish failed: {exc}')

    def _send(self, frame):
        if frame.mutating and self.dry_run:
            self.get_logger().warn(
                f'DRY RUN: would send {frame.label} to {frame.node} '
                f'(0x{frame.can_id:03X}#{frame.data.hex()})')
            return
        err = self.can.send(frame.can_id, frame.data, frame.rtr)
        if err:
            self.get_logger().warn(
                f'CAN send {frame.label} to {frame.node} failed: {err}')
        elif frame.mutating:
            self.get_logger().warn(
                f'Sent {frame.label} to {frame.node} '
                f'(0x{frame.can_id:03X}#{frame.data.hex()}); '
                'acceptance is verified from heartbeat/Get_Error')

    def _record(self, record):
        record = dict(record)
        record['timestamp'] = datetime.now(timezone.utc).isoformat()
        record['dry_run'] = self.dry_run
        self.get_logger().info('watchdog_event ' + json.dumps(
            record, default=str))
        try:
            self.fault_log.write(record)
        except OSError as exc:
            self.get_logger().error(f'Cannot write fault log: {exc}')

    # ---- service -------------------------------------------------------
    def _on_reset(self, request, response):
        now = time.monotonic()
        ok, message = self.core.request_manual_reset(
            now, self._snapshot(now))
        for act in self.core.actions:
            self._execute(act)
        response.success = ok
        response.message = message
        self.get_logger().warn(f'manual_reset: {message}')
        return response

    def close(self):
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
