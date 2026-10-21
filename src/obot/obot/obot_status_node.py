"""Read-only OBot status aggregator publishing /obot/status and /obot/diagnostics.

Never transmits CAN frames and never commands the drivetrain.
"""
import os
import time

from builtin_interfaces.msg import Time as TimeMsg
from geometry_msgs.msg import TwistStamped
from lifecycle_msgs.srv import GetState
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CompressedImage, LaserScan
from std_msgs.msg import Bool, UInt8
from std_srvs.srv import Trigger

from obot import obot_status_lib as lib
from obot.obot_fault_store import FaultStore
from obot.obot_odrive_monitor import CanReader, OdriveMonitor
from obot_msgs.msg import ObotDiagnostics, ObotStatus

NAV2_NODES = ['controller_server', 'smoother_server', 'planner_server',
              'behavior_server', 'velocity_smoother', 'collision_monitor',
              'bt_navigator', 'waypoint_follower']
LOCALIZATION_NODES = {1: 'amcl', 2: 'slam_toolbox'}


def _stamp(epoch):
    msg = TimeMsg()
    if epoch:
        msg.sec = int(epoch)
        msg.nanosec = int((epoch - int(epoch)) * 1e9)
    return msg


class ObotStatusNode(Node):
    def __init__(self):
        super().__init__('obot_status_node')
        p = self.declare_parameter
        self.log_dir = p('log_dir', '~/obot_logs').value
        self.can_if = p('can_interface', 'can0').value
        self.wifi_if = p('wifi_interface', 'wlan0').value
        self.enable_can = p('enable_can_monitor', True).value
        self.publish_rate = p('publish_rate_hz', 1.0).value
        timeouts = {
            'scan': p('scan_timeout_s', 2.0).value,
            'camera1': p('camera_timeout_s', 3.0).value,
            'camera2': p('camera_timeout_s_rear', 3.0).value,
            'odom': p('odom_timeout_s', 1.0).value,
            'valve': p('valve_timeout_s', 1.0).value,
            'teleop': 1.0, 'nav': 1.0,
        }
        self.fresh = lib.Freshness(timeouts)
        self.valve_on = None
        self.store = FaultStore(self.log_dir, self.get_logger())
        self.monitor = OdriveMonitor(
            self.store,
            history_len=int(p('voltage_history_samples', 3000).value),
            pre_s=p('fault_pre_window_s', 10.0).value,
            post_s=p('fault_post_window_s', 10.0).value,
            stale_s=p('telemetry_stale_s', 3.0).value,
            voltage_range=(0.0, p('max_valid_voltage_v', 80.0).value))
        self.store.append_event({'type': 'node_start', 't_epoch': time.time()})

        self.cpu = lib.CpuMeter()
        self.cpu.sample()
        self.can_reader = CanReader(self.can_if)
        self._can_retry_t = 0.0

        def mark(name):
            return lambda _m: self.fresh.mark(name, time.monotonic())
        sd = qos_profile_sensor_data
        self.create_subscription(LaserScan, p('scan_topic', '/scan').value,
                                 mark('scan'), sd)
        self.create_subscription(
            CompressedImage,
            p('camera1_topic', '/front_camera/image_raw/compressed').value,
            mark('camera1'), sd)
        self.create_subscription(
            CompressedImage,
            p('camera2_topic', '/rear_camera/image_raw/compressed').value,
            mark('camera2'), sd)
        self.create_subscription(
            Odometry, p('odom_topic', '/diff_drive_controller/odom').value,
            mark('odom'), sd)
        self.create_subscription(TwistStamped, '/cmd_vel_teleop',
                                 mark('teleop'), 10)
        self.create_subscription(TwistStamped, '/cmd_vel_nav', mark('nav'), 10)
        self.create_subscription(
            Bool, p('valve_state_topic', '/spray/valve_state').value,
            self._on_valve, 10)
        self.create_subscription(UInt8, '/obot/cyd/command', self._on_command, 10)
        self.create_service(Trigger, '/obot/reset_session_extrema',
                            self._on_reset_srv)

        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE)
        self.pub_status = self.create_publisher(ObotStatus, '/obot/status', qos)
        self.pub_diag = self.create_publisher(
            ObotDiagnostics, '/obot/diagnostics', qos)

        self.lc_clients = {}
        self.lc_states = {}
        self.lc_pending = {}
        for name in NAV2_NODES + list(LOCALIZATION_NODES.values()):
            self.lc_clients[name] = self.create_client(
                GetState, '/%s/get_state' % name)

        self.create_timer(0.05, self._poll_can)
        self.create_timer(2.0, self._poll_lifecycle)
        self.create_timer(1.0 / self.publish_rate, self._publish)
        self.get_logger().info('obot_status_node up; logging to %s'
                               % os.path.expanduser(self.log_dir))

    # ---- inputs ------------------------------------------------------
    def _on_valve(self, msg):
        self.valve_on = bool(msg.data)
        self.fresh.mark('valve', time.monotonic())

    def _do_reset(self):
        self.monitor.reset_session_extrema(time.time())
        self.get_logger().info('Session voltage extrema reset')

    def _on_command(self, msg):
        if msg.data == 1:
            self._do_reset()

    def _on_reset_srv(self, _req, resp):
        self._do_reset()
        resp.success = True
        resp.message = 'session extrema reset'
        return resp

    def _poll_can(self):
        if not self.enable_can:
            return
        now = time.monotonic()
        if self.can_reader.sock is None:
            if now < self._can_retry_t:
                return
            self._can_retry_t = now + 2.0
            try:
                self.can_reader.open()
            except OSError:
                return
        try:
            frames = self.can_reader.read_all()
        except OSError:
            self.can_reader.close()
            return
        epoch = time.time()
        for can_id, data, rtr in frames:
            self.monitor.handle_frame(can_id, data, now, epoch, rtr)
        self.monitor.tick(now, epoch)

    def _poll_lifecycle(self):
        for name, client in self.lc_clients.items():
            fut = self.lc_pending.get(name)
            if fut is not None and fut.done():
                try:
                    self.lc_states[name] = fut.result().current_state.id
                except Exception:
                    self.lc_states[name] = None
                self.lc_pending[name] = None
            elif fut is not None:
                continue
            if client.service_is_ready():
                self.lc_pending[name] = client.call_async(GetState.Request())
            elif fut is None:
                self.lc_states[name] = None

    # ---- status ------------------------------------------------------
    def _nav2_state(self):
        return lib.map_lifecycle({n: self.lc_states.get(n) for n in NAV2_NODES})

    def _localization(self):
        for mode, name in LOCALIZATION_NODES.items():
            if self.lc_states.get(name) is not None:
                return (lib.OK if self.lc_states[name] == lib.LC_ACTIVE
                        else lib.WARN), mode
        return lib.UNKNOWN, 0

    def _publish(self):
        now = time.monotonic()
        epoch = time.time()
        stamp = _stamp(epoch)
        snaps = {n: self.monitor.snapshot(n, now) for n in (0, 1)}
        can_status, can_code, can_rx, can_tx = lib.read_can_iface(self.can_if)
        avail, flags = lib.read_throttled()
        thr = lib.decode_throttled(flags)

        st = ObotStatus()
        st.stamp = stamp
        st.telemetry_link = self.fresh.state('odom', now)
        st.nav2 = self._nav2_state()
        st.localization, st.localization_mode = self._localization()
        st.lidar = self.fresh.state('scan', now)
        st.camera1 = self.fresh.state('camera1', now)
        st.camera2 = self.fresh.state('camera2', now)
        st.can_interface = can_status
        st.wifi_link = lib.wifi_state(self.wifi_if)
        if self.fresh.state('teleop', now) == lib.OK:
            st.control_mode = ObotStatus.CONTROL_MANUAL
        elif self.fresh.state('nav', now) == lib.OK:
            st.control_mode = ObotStatus.CONTROL_NAV2
        else:
            st.control_mode = ObotStatus.CONTROL_UNKNOWN
        vs = self.fresh.state('valve', now)
        if vs == lib.UNKNOWN:
            st.valve = ObotStatus.VALVE_UNKNOWN
        elif vs == lib.STALE:
            st.valve = ObotStatus.VALVE_STALE
        else:
            st.valve = (ObotStatus.VALVE_ON if self.valve_on
                        else ObotStatus.VALVE_OFF)
        hb, axis, err = [], [], []
        for n in (0, 1):
            s = snaps[n]
            if not s['heartbeat_seen']:
                hb.append(lib.UNKNOWN)
            else:
                hb.append(lib.OK if s['heartbeat_fresh'] else lib.STALE)
            axis.append(int(s['axis_state']) & 0xFF)
            mask = s['mask']
            if mask is None:
                err.append(lib.UNKNOWN if not s['heartbeat_seen'] else lib.STALE)
            elif mask == 0:
                err.append(lib.OK)
            elif mask & ~0x1 == 0:
                err.append(lib.WARN)
            else:
                err.append(lib.FAULT)
        st.motor_heartbeat, st.motor_axis_state, st.motor_error = hb, axis, err
        st.fault_count = min(65535, sum(s['fault_count'] for s in snaps.values()))
        items = [st.nav2, st.localization, st.lidar, st.camera1, st.camera2,
                 st.can_interface, st.wifi_link] + err
        st.alert = any(i == lib.FAULT for i in items)
        self.pub_status.publish(st)

        d = ObotDiagnostics()
        d.stamp = stamp
        d.pi_cpu_temp_c = float(lib.pi_temperature_c())
        d.pi_cpu_percent = float(self.cpu.sample())
        d.throttle_available = avail
        d.throttle_flags = flags
        d.undervoltage_now = thr['undervoltage_now']
        d.throttled_now = thr['throttled_now']
        d.undervoltage_since_boot = thr['undervoltage_since_boot']
        d.throttled_since_boot = thr['throttled_since_boot']
        d.bus_voltage_v = [snaps[n]['voltage'] for n in (0, 1)]
        d.bus_voltage_session_min_v = [snaps[n]['sess_min'] for n in (0, 1)]
        d.bus_voltage_session_max_v = [snaps[n]['sess_max'] for n in (0, 1)]
        d.bus_voltage_alltime_min_v = [snaps[n]['all_min'] for n in (0, 1)]
        d.bus_voltage_alltime_max_v = [snaps[n]['all_max'] for n in (0, 1)]
        d.controller_temp_c = [snaps[n]['temp_c'] for n in (0, 1)]
        d.active_errors = [snaps[n]['active_errors'] for n in (0, 1)]
        d.disarm_reason = [snaps[n]['disarm_reason'] for n in (0, 1)]
        d.fault_count = [min(65535, snaps[n]['fault_count']) for n in (0, 1)]
        d.last_fault_stamp = [_stamp(snaps[n]['last_fault_epoch'])
                              for n in (0, 1)]
        d.can_state = can_code
        d.can_rx_errors = can_rx
        d.can_tx_errors = can_tx
        d.session_extrema_reset_stamp = _stamp(self.monitor.session_reset_epoch)
        self.pub_diag.publish(d)

    def shutdown(self):
        self.monitor.flush(time.monotonic(), time.time())
        self.can_reader.close()


def main(args=None):
    rclpy.init(args=args)
    node = ObotStatusNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

