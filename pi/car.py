"""Hardware layer: pigpio -> L298N motor bridge + HS-311 steering servo.

Nothing here decides anything. ``vehicle.Vehicle`` produces an ``Outputs``; this module
turns it into pin states. Keeping the policy out of the driver is what lets the same
behaviour model run on the Mac simulator.

Safety properties this module is responsible for:

* All six L298N pins are driven OUTPUT + LOW as the very first action after connecting,
  because GPIO5/6 boot HIGH on BCM283x and the bridge is therefore already asserted
  before our process exists.
* Coast is ``EN`` low. ``IN1 == IN2`` with ``EN`` high is a *brake* -- it shorts the
  motor through the bridge. This inverts most people's intuition and is the single
  easiest thing to get backwards here.
* ``close()`` always lands on coast + no servo pulses + LED off, and is safe to call
  twice (the signal handler and the ``finally`` block both call it).
"""

from __future__ import annotations

import config
from protocol import DIR_BRAKE, DIR_COAST, DIR_FWD, DIR_REV

try:  # pragma: no cover - absent on any machine that is not the Pi
    import pigpio
except ImportError:  # pragma: no cover
    pigpio = None


class Car:
    """pigpio-backed actuators. One instance owns all the pins for the process."""

    def __init__(self) -> None:
        if pigpio is None:
            raise RuntimeError(
                "pigpio is not importable. It comes from apt, not pip: "
                "`sudo apt install python3-pigpio`, and the venv must be created "
                "with --system-site-packages."
            )

        self.pi = pigpio.pi()
        if not self.pi.connected:
            raise RuntimeError(
                "cannot connect to pigpiod on localhost:8888. Start it with "
                "`sudo systemctl start pigpiod` (or `sudo pigpiod`) and try again."
            )

        # --- FIRST ACTION, before PWM, before the servo, before anything ---------
        # GPIO5/6 boot HIGH, so IN1/IN2 are asserted from the instant the Pi has power.
        # Until these six writes land, the bridge state is whatever the bootloader left
        # behind. External 10k pull-downs cover the window before this line; this line
        # covers everything after it.
        for pin in config.MOTOR_PINS:
            self.pi.set_mode(pin, pigpio.OUTPUT)
            self.pi.write(pin, 0)

        # DMA-timed software PWM on the enable pins. Deliberately *not* hardware_PWM():
        # the two hardware channels share one clock divider (so ENA and ENB could not be
        # set independently anyway), and snd_bcm2835 claims the PWM peripheral -- it
        # comes back via the HDMI codec path even after the audio overlay is disabled,
        # and then hardware_PWM() silently fights the audio driver.
        for pin in (config.ENA, config.ENB):
            self.pi.set_PWM_range(pin, config.PWM_RANGE)
            self.pi.set_PWM_frequency(pin, config.PWM_FREQ_HZ)
            self.pi.set_PWM_dutycycle(pin, 0)

        self.pi.set_mode(config.LED, pigpio.OUTPUT)
        self.pi.write(config.LED, 0)

        # Active-low momentary button to ground with the internal pull-up enabled:
        # unpressed reads 1, pressed reads 0. Only consulted when config.ESTOP_ENABLED,
        # which stays False until the polarity is confirmed with a meter.
        self.pi.set_mode(config.ESTOP, pigpio.INPUT)
        self.pi.set_pull_up_down(config.ESTOP, pigpio.PUD_UP)

        # Servo starts silent. Pulses begin only once something asks for them.
        self.pi.set_servo_pulsewidth(config.SERVO, 0)

        # Cached last-written state, so a 100 Hz loop does not re-send identical values
        # to the daemon on every tick. None means "unknown, write it".
        self._last_dir: str | None = None
        self._last_duty_byte: int | None = None
        self._last_servo_us: int | None = None
        self._last_led: int | None = None
        self._closed = False

    # -- motors ----------------------------------------------------------------

    def apply(self, out) -> None:
        """Drive the bridge from a ``vehicle.Outputs``."""
        direction = out.direction
        duty_byte = _duty_byte(out.duty)

        if direction == DIR_COAST:
            # Coast: enables low. The IN pins are irrelevant with EN low, but leaving
            # them de-asserted means a stray enable cannot energise the bridge.
            self._set_duty(0)
            self._set_dir_pins(direction, 0, 0, 0, 0)
        elif direction == DIR_BRAKE:
            # Brake: IN1 == IN2 with EN high shorts the motor through the bridge.
            # Both-low and both-high both brake; both-low leaves the pins de-asserted.
            self._set_dir_pins(direction, 0, 0, 0, 0)
            self._set_duty(config.PWM_RANGE)
        elif direction == DIR_FWD:
            self._change_direction(direction, *_levels(True))
            self._set_duty(duty_byte)
        elif direction == DIR_REV:
            self._change_direction(direction, *_levels(False))
            self._set_duty(duty_byte)
        else:  # unknown direction -- treat exactly like a fault: coast.
            self._set_duty(0)
            self._set_dir_pins(DIR_COAST, 0, 0, 0, 0)

    def _change_direction(self, direction: str, a: int, b: int, c: int, d: int) -> None:
        """Flip the IN pins, dropping the enables first if the direction is changing.

        ``Vehicle`` already inserts a coast interval on a reversal, so this should never
        actually catch a spinning motor. It is here because "enables low while the IN
        pins move" costs one write and removes the failure mode entirely.
        """
        if direction != self._last_dir:
            self._set_duty(0)
        self._set_dir_pins(direction, a, b, c, d)

    def _set_dir_pins(self, direction: str, a: int, b: int, c: int, d: int) -> None:
        if direction == self._last_dir:
            return
        self.pi.write(config.IN1, a)
        self.pi.write(config.IN2, b)
        self.pi.write(config.IN3, c)
        self.pi.write(config.IN4, d)
        self._last_dir = direction

    def _set_duty(self, duty_byte: int) -> None:
        if duty_byte == self._last_duty_byte:
            return
        self.pi.set_PWM_dutycycle(config.ENA, duty_byte)
        self.pi.set_PWM_dutycycle(config.ENB, duty_byte)
        self._last_duty_byte = duty_byte

    def coast(self) -> None:
        """The safe resting state: enables low, nothing energised, the car rolls."""
        self._set_duty(0)
        self._set_dir_pins(DIR_COAST, 0, 0, 0, 0)

    # -- servo -----------------------------------------------------------------

    def set_servo_us(self, us: int) -> None:
        """Set the steering pulse width. ``us == 0`` stops the pulse train entirely.

        Stopping matters: the HS-311 hangs off the Pi's own 5 V rail, and a stalled one
        holding position against the linkage can brown out the SoC.
        """
        us = int(us)
        if us == self._last_servo_us:
            return
        self.pi.set_servo_pulsewidth(config.SERVO, us)
        self._last_servo_us = us

    # -- status LED / e-stop ---------------------------------------------------

    def set_led(self, state: bool) -> None:
        value = 1 if state else 0
        if value == self._last_led:
            return
        self.pi.write(config.LED, value)
        self._last_led = value

    def read_estop(self) -> bool:
        """True when the e-stop reads *pressed*.

        Active low with the internal pull-up: unpressed is 1, pressed is 0. Callers
        should only act on this when ``config.ESTOP_ENABLED`` -- the wiring polarity is
        unconfirmed, and a normally-closed switch read this way would look permanently
        pressed.
        """
        try:
            level = self.pi.read(config.ESTOP)
        except Exception:
            # A failed read must not be able to stop the control loop. Report
            # "not pressed" and let the watchdog handle a genuinely dead link.
            return False
        return level == 0 if config.ESTOP_ACTIVE_LOW else level == 1

    # -- housekeeping ----------------------------------------------------------

    def cpu_temp(self) -> float | None:
        """SoC temperature in degrees C, or None if it cannot be read."""
        try:
            with open("/sys/class/thermal/thermal_zone0/temp") as fh:
                return int(fh.read().strip()) / 1000.0
        except (OSError, ValueError):
            return None

    def close(self) -> None:
        """Coast, silence the servo, LED off, disconnect. Safe to call twice.

        Every step is individually guarded: this runs from a signal handler and from a
        ``finally`` block, and a failure partway through must not leave the bridge
        energised because an exception skipped the rest.
        """
        if self._closed:
            return
        self._closed = True
        for action in (
            lambda: self.coast(),
            lambda: self.pi.set_servo_pulsewidth(config.SERVO, 0),
            lambda: self.pi.write(config.LED, 0),
        ):
            try:
                action()
            except Exception:
                pass
        try:
            self.pi.stop()
        except Exception:
            pass

    #: ``stop()`` reads better at the call site in main.py; same thing.
    stop = close


def _levels(forward: bool) -> tuple[int, int, int, int]:
    """(IN1, IN2, IN3, IN4) for the requested travel direction.

    The per-side invert flags exist because the motors are mounted mirror-image: the same
    polarity on both bridges makes them fight each other rather than drive the car.
    """
    left = (0, 1) if forward == config.INVERT_LEFT else (1, 0)
    right = (0, 1) if forward == config.INVERT_RIGHT else (1, 0)
    return left[0], left[1], right[0], right[1]


def _duty_byte(duty: float) -> int:
    """0.0-1.0 -> 0-255, clamped. pigpio wants an int in the configured PWM range."""
    if duty <= 0.0:
        return 0
    if duty >= 1.0:
        return config.PWM_RANGE
    return int(round(duty * config.PWM_RANGE))
