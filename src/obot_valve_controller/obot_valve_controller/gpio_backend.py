"""Mock and libgpiod backends for a single valve output."""


class MockGPIOOutput:
    """Logical GPIO output used by tests and safe mock-mode launches."""

    def __init__(self, chip, pin, active_high):
        """Initialize a closed-safe logical output without hardware access."""
        self.chip = chip
        self.pin = pin
        self.active_high = active_high
        self.is_open = False
        self.closed = False

    def set_open(self, opened):
        """Record the requested logical valve state."""
        if self.closed:
            raise RuntimeError('GPIO output has already been released')
        self.is_open = bool(opened)

    def close(self):
        """Release the mock output."""
        self.is_open = False
        self.closed = True


class GpiodGPIOOutput:
    """A single libgpiod 1.x line request for the spray valve."""

    def __init__(self, chip, pin, active_high):
        """Acquire the configured GPIO line once and initialize it OFF."""
        try:
            import gpiod
        except ImportError as exc:
            raise RuntimeError(
                'python3-libgpiod with the libgpiod 1.x Python API is '
                'required '
                'when mock_gpio is false'
            ) from exc

        self._chip = gpiod.Chip(chip)
        self._active_high = active_high
        off_value = 0 if active_high else 1
        try:
            self._line = self._chip.get_line(pin)
            self._line.request(
                consumer='obot-valve-controller',
                type=gpiod.LINE_REQ_DIR_OUT,
                default_val=off_value,
            )
        except Exception:
            self._chip.close()
            raise
        self._closed = False

    def set_open(self, opened):
        """Set the line to the active or inactive electrical level."""
        if self._closed:
            raise RuntimeError('GPIO output has already been released')
        high = bool(opened) == self._active_high
        self._line.set_value(1 if high else 0)

    def close(self):
        """Release the libgpiod line request."""
        if not self._closed:
            try:
                self._line.release()
            finally:
                try:
                    self._chip.close()
                finally:
                    self._closed = True


def create_gpio_output(chip, pin, active_high, mock_gpio=False):
    """Create the selected mock or physical GPIO output."""
    if mock_gpio:
        return MockGPIOOutput(chip, pin, active_high)
    return GpiodGPIOOutput(chip, pin, active_high)
