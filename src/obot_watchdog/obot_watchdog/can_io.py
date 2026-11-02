"""SocketCAN receive/transmit and CAN link-state polling (thread based).

Blocking I/O lives here so the ROS callbacks and the state machine never
block. Frames are parsed with the pure helpers in odrive_can.
"""

import copy
import re
import socket
import struct
import subprocess
import threading
import time

from . import odrive_can as oc
from .core import NodeTel


class CanTelemetry:

    def __init__(self, interface, left, right, logger=None):
        self.interface = interface
        self.log = logger
        self.lock = threading.Lock()
        self.nodes = {left.node_id: left, right.node_id: right}
        self.left, self.right = left, right
        self.sock = None
        self.running = False
        self.thread = None
        self.can_state = None
        self.can_time = None
        self.can_seq = 0
        self.rx_error = None

    def ingest(self, raw, now=None):
        parsed = oc.parse_frame(raw)
        if parsed is None:
            return
        node_id, cmd, data = parsed
        tel = self.nodes.get(node_id)
        if tel is None:
            return
        now = time.monotonic() if now is None else now
        with self.lock:
            if cmd == oc.CMD_HEARTBEAT:
                hb = oc.parse_heartbeat(data)
                if hb:
                    tel.hb_error, tel.hb_state = hb
                    tel.hb_time = now
            elif cmd == oc.CMD_GET_ERROR:
                ge = oc.parse_get_error(data)
                if ge:
                    tel.ge_active, tel.ge_disarm = ge
                    tel.ge_time = now
            elif cmd == oc.CMD_GET_BUS_VOLTAGE_CURRENT:
                volts = oc.parse_voltage(data)
                if volts is not None:
                    tel.v = volts
                    tel.v_time = now

    def snapshot_nodes(self):
        with self.lock:
            return (copy.copy(self.left), copy.copy(self.right),
                    self.can_state, self.can_time, self.can_seq)

    def set_can_state(self, state, now=None):
        with self.lock:
            self.can_state = state
            self.can_time = time.monotonic() if now is None else now
            self.can_seq += 1

    def start(self):
        self.sock = socket.socket(
            socket.PF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
        self.sock.settimeout(0.2)
        self.sock.bind((self.interface,))
        self.running = True
        self.thread = threading.Thread(target=self._rx_loop, daemon=True)
        self.thread.start()

    def stop(self):
        self.running = False
        if self.thread:
            self.thread.join(timeout=1.0)
        if self.sock:
            self.sock.close()

    def _rx_loop(self):
        while self.running:
            try:
                raw = self.sock.recv(oc.CAN_FRAME_SIZE)
            except socket.timeout:
                continue
            except OSError as exc:
                self.rx_error = str(exc)
                if self.log:
                    self.log.error(f'CAN receive failed: {exc}')
                time.sleep(0.5)
                continue
            self.ingest(raw)

    def send(self, can_id, data=b'', rtr=False):
        """Transmit one standard frame; returns None or an error string."""
        if self.sock is None:
            return 'CAN socket not open'
        flags = oc.CAN_RTR_FLAG if rtr else 0
        frame = struct.pack(
            oc.CAN_FRAME_FMT, can_id | flags, len(data),
            data.ljust(8, b'\x00'))
        try:
            self.sock.send(frame)
        except OSError as exc:
            return str(exc)
        return None


_STATE_RE = re.compile(r'can state ([A-Z-]+)')
_BITRATE_RE = re.compile(r'bitrate (\d+)')


def parse_can_link(text, expected_bitrate=None):
    """Return the CAN state from `ip -details link show` output.

    A bitrate that differs from the expected one is reported as
    BITRATE_MISMATCH so the watchdog treats it as a CAN fault.
    """
    match = _STATE_RE.search(text)
    if not match:
        return 'UNKNOWN'
    bitrate = _BITRATE_RE.search(text)
    if (expected_bitrate and bitrate
            and int(bitrate.group(1)) != expected_bitrate):
        return f'BITRATE_MISMATCH_{bitrate.group(1)}'
    return match.group(1)


def read_can_state(interface, expected_bitrate=None, timeout=2.0):
    """Return the kernel CAN state (e.g. ERROR-ACTIVE) or 'UNKNOWN'."""
    try:
        result = subprocess.run(
            ['ip', '-details', 'link', 'show', interface],
            text=True, capture_output=True, timeout=timeout, check=False)
    except (OSError, subprocess.TimeoutExpired):
        return 'UNKNOWN'
    return parse_can_link(result.stdout, expected_bitrate)


class CanStatePoller(threading.Thread):

    def __init__(self, telemetry, interface, period, expected_bitrate=None):
        super().__init__(daemon=True)
        self.expected_bitrate = expected_bitrate
        self.telemetry = telemetry
        self.interface = interface
        self.period = period
        self.stop_event = threading.Event()

    def run(self):
        while not self.stop_event.is_set():
            self.telemetry.set_can_state(read_can_state(
                self.interface, self.expected_bitrate))
            self.stop_event.wait(self.period)

    def stop(self):
        self.stop_event.set()


def make_nodes(left_id, right_id):
    return NodeTel('left', left_id), NodeTel('right', right_id)
