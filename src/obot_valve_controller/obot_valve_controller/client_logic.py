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
