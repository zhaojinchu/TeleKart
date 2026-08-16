"""A PID that expects to be given most of the answer.

This regulator is not the primary source of duty. The feedforward table from
calibration carries the load and the PID only trims it, which is what makes the
loop tunable at all on a plant whose gain is uncertain by roughly a factor of
two between a fresh pack and a flat one, and again between wheels in the air
and wheels on carpet.

Three deliberate choices:

**Conditional integration.** The integrator freezes whenever the output is
saturated *and* the error would push it further into the stop. Clamping the
integral alone is not enough -- the classic failure is a car held against a kerb
at full throttle, where the integral sits at its clamp and the wheels then take
half a second to respond when the kerb is cleared. Freezing costs nothing and
removes the failure entirely.

**Derivative on measurement, filtered.** Differentiating the error makes the
output spike every time the setpoint steps; differentiating the measurement does
not. And encoder velocity is quantised and noisy even after M/T, so an unfiltered
derivative is an amplifier for exactly the wrong signal. ``kd`` defaults to zero
in the parameter registry for that reason; the filter is here so that raising it
is a survivable experiment.

**The feedforward is inside the saturation arithmetic.** Limiting the PID's own
output and then adding a feedforward on top would let the sum leave the range
the caller asked for, and the integrator would never learn that it had.
"""

from __future__ import annotations

import math


class PID:
    """Single-axis regulator with feedforward, in whatever units the caller uses.

    On this car the input is RPM error and the output is duty, so ``kp`` is in
    duty per RPM. Nothing in the class knows that; it is dimensionless and the
    units live in the parameter registry.
    """

    __slots__ = (
        "_kp", "_ki", "_kd", "_i_clamp", "_lo", "_hi", "_derivative_tau",
        "_integral", "_prev_measured", "_have_prev", "_derivative",
        "saturated", "output",
    )

    def __init__(
        self,
        kp: float,
        ki: float,
        kd: float,
        *,
        i_clamp: float = 0.4,
        output_limits: tuple[float, float] = (-1.0, 1.0),
        derivative_lpf_hz: float = 10.0,
    ) -> None:
        lo, hi = output_limits
        if lo >= hi:
            raise ValueError(f"output_limits must be (lo, hi) with lo < hi, got {output_limits}")
        if i_clamp < 0.0:
            raise ValueError(f"i_clamp must not be negative, got {i_clamp}")
        if derivative_lpf_hz <= 0.0:
            raise ValueError(f"derivative_lpf_hz must be positive, got {derivative_lpf_hz}")

        self._kp = kp
        self._ki = ki
        self._kd = kd
        self._i_clamp = i_clamp
        self._lo = lo
        self._hi = hi
        #: Time constant of the derivative filter, kept as a period so the
        #: per-tick coefficient is one division instead of an exp().
        self._derivative_tau = 1.0 / (math.tau * derivative_lpf_hz)

        self._integral = 0.0
        self._prev_measured = 0.0
        self._have_prev = False
        self._derivative = 0.0
        #: True when the last output hit a limit. The caller uses it for the
        #: LIMITER_ACTIVE telemetry bit and for its own anti-windup decisions.
        self.saturated = False
        self.output = 0.0

    # -- regulation ---------------------------------------------------------

    def update(
        self,
        setpoint: float,
        measured: float,
        dt: float,
        *,
        feedforward: float = 0.0,
    ) -> float:
        """One step. Returns the clamped output; never raises.

        ``dt`` of zero or less returns the previous output unchanged rather than
        dividing by it. That happens on the first tick after a re-anchor and
        must not be allowed to inject an impulse.
        """
        if dt <= 0.0:
            return self.output
        if setpoint != setpoint or measured != measured:
            # A NaN from a divide-by-zero upstream must not poison the
            # integrator, which would then never recover for the rest of the run.
            self.saturated = False
            return self.output
        if feedforward != feedforward:
            feedforward = 0.0

        error = setpoint - measured

        # Derivative on the measurement, low-pass filtered, and negated because
        # d(error)/dt == -d(measured)/dt when the setpoint is steady.
        if self._kd != 0.0:
            if self._have_prev:
                raw = (measured - self._prev_measured) / dt
                alpha = dt / (self._derivative_tau + dt)
                self._derivative += (raw - self._derivative) * alpha
            else:
                self._derivative = 0.0
        else:
            self._derivative = 0.0
        self._prev_measured = measured
        self._have_prev = True

        proportional = self._kp * error
        derivative = -self._kd * self._derivative

        # Trial integration, then decide whether to keep it. The integral is
        # clamped twice: by i_clamp, which bounds the authority the operator
        # granted it, and by the room the feedforward leaves inside the output
        # limits, which is what stops it accumulating against a ceiling it can
        # never move.
        candidate = self._integral + self._ki * error * dt
        clamp_value = self._i_clamp
        if candidate > clamp_value:
            candidate = clamp_value
        elif candidate < -clamp_value:
            candidate = -clamp_value
        headroom_hi = self._hi - feedforward
        headroom_lo = self._lo - feedforward
        if candidate > headroom_hi:
            candidate = headroom_hi
        elif candidate < headroom_lo:
            candidate = headroom_lo

        unclamped = feedforward + proportional + candidate + derivative
        if unclamped > self._hi:
            output = self._hi
            saturated = True
        elif unclamped < self._lo:
            output = self._lo
            saturated = True
        else:
            output = unclamped
            saturated = False

        # Conditional integration: accept the new integral unless we are pinned
        # against a limit and the error is still pushing into it. Testing the
        # limit that was actually hit, rather than the sign of the output, keeps
        # this correct for asymmetric limits -- which is what a duty ceiling
        # imposed after a brownout produces.
        pushing_into_stop = (unclamped > self._hi and error > 0.0) or (
            unclamped < self._lo and error < 0.0
        )
        if not (saturated and pushing_into_stop):
            self._integral = candidate

        self.saturated = saturated
        self.output = output
        return output

    def reset(self) -> None:
        """Forget everything. Called on arm, on direction change, and on disarm.

        On arm because whatever the integrator learned while stationary is about
        a plant that was not connected. On direction change because the
        feedforward table for the other direction is a different curve. On
        disarm so that re-arming is not a surprise.
        """
        self._integral = 0.0
        self._derivative = 0.0
        self._prev_measured = 0.0
        self._have_prev = False
        self.saturated = False
        self.output = 0.0

    def set_gains(self, kp: float, ki: float, kd: float) -> None:
        """Retune live. Safe while running: the integral is a stored *output*
        contribution, not an accumulated error, so changing ``ki`` does not
        retroactively rescale history."""
        self._kp = kp
        self._ki = ki
        self._kd = kd
        if kd == 0.0:
            self._derivative = 0.0

    def set_output_limits(self, lo: float, hi: float) -> None:
        """Narrow or widen the output range, e.g. when the safety layer drops
        the duty ceiling after a brownout."""
        if lo >= hi:
            raise ValueError(f"output limits must satisfy lo < hi, got ({lo}, {hi})")
        self._lo = lo
        self._hi = hi
        if self._integral > hi:
            self._integral = hi
        elif self._integral < lo:
            self._integral = lo

    # -- introspection ------------------------------------------------------

    @property
    def integral(self) -> float:
        return self._integral

    @property
    def derivative(self) -> float:
        """Filtered derivative of the *measurement*, before ``kd``."""
        return self._derivative

    @property
    def gains(self) -> tuple[float, float, float]:
        return (self._kp, self._ki, self._kd)

    def __repr__(self) -> str:
        return (
            f"PID(kp={self._kp:g}, ki={self._ki:g}, kd={self._kd:g}, "
            f"i={self._integral:+.3f}, out={self.output:+.3f}, "
            f"sat={self.saturated})"
        )
