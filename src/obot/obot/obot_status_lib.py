"""Pure helpers for obot_status_node: Pi metrics, freshness, CAN iface, mapping.

Everything here is free of ROS so it can be tested with synthetic data. File
paths and command runners are injectable.
"""
import re
import subprocess

UNKNOWN, OK, WARN, FAULT, STALE = 0, 1, 2, 3, 4
NAN = float('nan')


def read_text(path):
    with open(path, 'r', encoding='utf-8') as f:
        return f.read()


def pi_temperature_c(path='/sys/class/thermal/thermal_zone0/temp', reader=read_text):
    try:
        value = int(reader(path).strip()) / 1000.0
        return value if -40.0 <= value <= 150.0 else NAN
    except (OSError, ValueError):
        return NAN


def parse_proc_stat(text):
    """Return (busy, total) jiffies from the aggregate cpu line, or None."""
    for line in text.splitlines():
        if line.startswith('cpu '):
            vals = [int(x) for x in line.split()[1:]]
            if len(vals) < 4:
                return None
            idle = vals[3] + (vals[4] if len(vals) > 4 else 0)
            total = sum(vals[:8])
            return total - idle, total
    return None


class CpuMeter:
    def __init__(self, path='/proc/stat', reader=read_text):
        self._path = path
        self._reader = reader
        self._last = None

    def sample(self):
        """CPU percent since the previous call; NaN on the first call or error."""
        try:
            cur = parse_proc_stat(self._reader(self._path))
        except OSError:
            return NAN
        if cur is None:
            return NAN
        last, self._last = self._last, cur
        if last is None or cur[1] <= last[1]:
            return NAN
        return 100.0 * (cur[0] - last[0]) / (cur[1] - last[1])


def decode_throttled(value):
    return {
        'undervoltage_now': bool(value & 0x1),
        'throttled_now': bool(value & 0xE),
        'undervoltage_since_boot': bool(value & 0x10000),
        'throttled_since_boot': bool(value & 0xE0000),
    }


def read_throttled(runner=None):
    """Return (available, raw_flags) from `vcgencmd get_throttled`."""
    runner = runner or _run
    try:
        out = runner(['vcgencmd', 'get_throttled'])
        m = re.search(r'throttled=(0x[0-9a-fA-F]+)', out)
        if not m:
            return False, 0
        return True, int(m.group(1), 16)
    except (OSError, subprocess.SubprocessError):
        return False, 0


def _run(cmd, timeout=2.0):
    return subprocess.run(cmd, capture_output=True, text=True, timeout=timeout,
                          check=True).stdout


CAN_STATE_CODES = {
    'ERROR-ACTIVE': 1, 'ERROR-WARNING': 2, 'ERROR-PASSIVE': 3,
    'BUS-OFF': 4, 'STOPPED': 5,
}


def parse_can_iface(text):
    """Parse `ip -details -statistics link show can0`.

    Returns (status, can_state_code, rx_err, tx_err).
    """
    if not text or 'can' not in text:
        return FAULT, 0, 0, 0
    up = bool(re.search(r'<[^>]*\bUP\b[^>]*>', text))
    m = re.search(r'can state ([A-Z-]+)', text)
    state = CAN_STATE_CODES.get(m.group(1), 0) if m else 0
    b = re.search(r'berr-counter tx (\d+) rx (\d+)', text)
    tx, rx = (int(b.group(1)), int(b.group(2))) if b else (0, 0)
    if not up or state in (4, 5):
        return FAULT, state, rx, tx
    if state == 1:
        return OK, state, rx, tx
    if state in (2, 3):
        return WARN, state, rx, tx
    return UNKNOWN, state, rx, tx


def read_can_iface(interface='can0', runner=None):
    runner = runner or _run
    try:
        return parse_can_iface(
            runner(['ip', '-details', '-statistics', 'link', 'show', interface]))
    except subprocess.CalledProcessError:
        return FAULT, 0, 0, 0
    except (OSError, subprocess.SubprocessError):
        return UNKNOWN, 0, 0, 0


class Freshness:
    """Tracks last-receipt time per named source."""

    def __init__(self, timeouts):
        self.timeouts = dict(timeouts)
        self.last = {}

    def mark(self, name, t):
        self.last[name] = t

    def state(self, name, t):
        """UNKNOWN if never seen, OK if fresh, STALE if expired."""
        if name not in self.last:
            return UNKNOWN
        return OK if t - self.last[name] <= self.timeouts[name] else STALE


# lifecycle_msgs/State ids
LC_UNKNOWN, LC_UNCONFIGURED, LC_INACTIVE, LC_ACTIVE, LC_FINALIZED = 0, 1, 2, 3, 4


def map_lifecycle(states):
    """Map {node_name: state_id or None} to a status code.

    None means the node did not answer. Inactive/unconfigured is WARN (not
    started), finalized or partial availability is FAULT, none answering is
    UNKNOWN. A lifecycle error is reported as a FAULT too.
    """
    if not states:
        return UNKNOWN
    answered = [s for s in states.values() if s is not None]
    if not answered:
        return UNKNOWN
    if any(s in (LC_FINALIZED, LC_UNKNOWN) for s in answered):
        return FAULT
    if len(answered) != len(states):
        return FAULT
    if all(s == LC_ACTIVE for s in answered):
        return OK
    return WARN


def parse_wireless(text, interface):
    """Return link quality (int) for interface from /proc/net/wireless, else None."""
    for line in text.splitlines():
        if line.strip().startswith(interface + ':'):
            parts = line.split()
            try:
                return int(float(parts[2]))
            except (IndexError, ValueError):
                return None
    return None


def wifi_state(interface, reader=read_text):
    try:
        oper = reader('/sys/class/net/%s/operstate' % interface).strip()
    except OSError:
        return UNKNOWN
    return OK if oper == 'up' else FAULT
