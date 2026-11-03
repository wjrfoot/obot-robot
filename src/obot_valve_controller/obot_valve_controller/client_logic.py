"""Hardware-independent input and rumble state helpers."""

import time
from dataclasses import dataclass


class ButtonEdgeDetector:
    """Report only newly pressed buttons for configured zero-based indices."""

    def __init__(self, button_indices):
        """Create an edge detector from button names and Joy indices."""
        self.button_indices = dict(button_indices)
        self._previous = {}

    def pressed_edges(self, buttons):
        """Return names of buttons that changed from released to pressed."""
        edges = set()
        current = {}
        for name, index in self.button_indices.items():
            if index < 0 or index >= len(buttons):
                current[name] = False
                continue
            pressed = buttons[index] > 0
            current[name] = pressed
            if pressed and not self._previous.get(name, False):
                edges.add(name)
        self._previous = current
        return edges


@dataclass
class ValveRumbleState:
    """Determine rumble refresh/stop commands from recent valve state."""

    intensity: float
    timeout_sec: float
    refresh_sec: float = 0.2
    clock: object = time.monotonic

    def __post_init__(self):
        """Initialize the state needed to track refresh timing."""
        self._valve_open = False
        self._last_state_time = None
        self._last_refresh_time = None
        self._rumbling = False
        self._stopped = False

    def receive_state(self, valve_open, received_at=None):
        """Record a commanded valve-state update and its monotonic time."""
        self._valve_open = bool(valve_open)
        now = self.clock() if received_at is None else received_at
        self._last_state_time = now
        self._stopped = False

    def rumble_output(self, now=None):
        """Return a new intensity command, or None when no publish is due."""
        current = self.clock() if now is None else now
        state_fresh = (
            self._last_state_time is not None
            and current - self._last_state_time <= self.timeout_sec
        )
        should_rumble = (
            self._valve_open and state_fresh and not self._stopped
        )
        if not should_rumble:
            if self._rumbling:
                self._rumbling = False
                self._last_refresh_time = current
                return 0.0
            return None

        if (
            not self._rumbling
            or self._last_refresh_time is None
            or current - self._last_refresh_time >= self.refresh_sec - 1e-9
        ):
            self._rumbling = True
            self._last_refresh_time = current
            return self.intensity
        return None

    def stop(self, now=None):
        """Stop rumble and prevent restarts until another state update."""
        current = self.clock() if now is None else now
        was_rumbling = self._rumbling
        self._rumbling = False
        self._stopped = True
        self._last_refresh_time = current
        return 0.0 if was_rumbling else None


class WatchdogRumbleState:
    """Generate timed watchdog alerts with priority over valve rumble."""

    def __init__(self, clock=time.monotonic):
        self.clock = clock
        self._started = None
        self._pattern = ()
        self._intensity = 0.0
        self._event = None

    def trigger(self, event, now=None):
        """Start an alert; return False for unrecognized events."""
        patterns = {
            'fault': (0.25, 0.20, 3),
            'recovered': (0.25, 0.0, 1),
            'lockout': (0.5, 0.2, 3),
        }
        if event not in patterns:
            return False

        # Expire any completed alert before comparing priorities.
        self.output(now)

        priority = {'recovered': 1, 'fault': 2, 'lockout': 3}
        if self.active and priority[event] < priority[self._event]:
            return False

        self._event = event
        on_time, off_time, count = patterns[event]
        self._pattern = (on_time, off_time, count)
        self._started = self.clock() if now is None else now
        self._intensity = 0.8
        return True

    def output(self, now=None):
        """Return current intensity, or None when no alert is active."""
        if self._started is None:
            return None

        current = self.clock() if now is None else now
        on_time, off_time, count = self._pattern
        elapsed = max(0.0, current - self._started)
        cycle = on_time + off_time
        duration = count * on_time + (count - 1) * off_time

        if elapsed >= duration:
            self._started = None
            self._event = None
            return None

        return (
            self._intensity
            if elapsed % cycle < on_time
            else 0.0
        )

    @property
    def active(self):
        return self._started is not None
