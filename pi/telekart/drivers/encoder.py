"""Wheel speed from Hall encoders, decoded x2 with the direction inferred.

The decoding choice here is a budget decision and worth stating plainly, because
it looks like a shortcut and is not.

True quadrature needs channel B's level at the instant channel A moves. With
pigpio that means a socket round trip to the daemon -- 50 to 100 microseconds --
inside every edge callback. At 660 counts per output revolution and 200 RPM
that is 2200 edges a second per wheel, 4400 for the car: between one and two
entire A53 cores spent asking a daemon what a pin was doing. Caching B with a
second EITHER_EDGE callback does not help either, because B fires just as often
as A, so the event rate becomes the same as full x4 decoding.

There is no cheap route to hardware direction, so we buy resolution instead and
infer the sign: EITHER_EDGE on channel A only, direction from the commanded
H-bridge state. The car cannot roll backwards under forward power at any speed
this drivetrain reaches, and the one genuinely ambiguous case -- coasting with
no command -- is reported as ``direction_uncertain`` rather than guessed
silently. Channel B is still wired, and it is polled as a disagreement check.

The callback body does nothing but increment a counter and stamp a tick. Not
"almost nothing": literally two stores. It runs on pigpio's notification thread
thousands of times a second, and anything else in it -- a lock, a lookup, a
branch on state the control thread also writes -- becomes jitter on the loop.
"""

from __future__ import annotations

from dataclasses import dataclass

from ..constants import ENCODER_GLITCH_US, ENCODER_STALE_WINDOW_S
from ..hal.base import CallbackHandle, Edge, GpioBackend, Pull, tick_diff

#: Output filter time constant. Counts-per-tick is unusable at low speed -- at
#: 100 Hz and 660 cpr, 20 RPM is 2.2 counts per tick, so quantisation alone is
#: +/-45 % -- and M/T fixes the estimate but not the residual noise. 25 ms of
#: first-order filtering is what makes the number usable, and it is also what
#: caps closed-loop bandwidth at roughly 7 Hz. Do not try to tune past it: the
#: gain margin you think you are finding is the filter's phase lag.
VELOCITY_FILTER_TAU_S = 0.025

#: Most edges a window may contain before the polled channel-B reads are
#: skipped entirely. Polling B samples the quadrant, and one A edge already
#: advances the phase by a whole count -- two quadrant steps -- so past this the
#: sequence aliases and nothing can be concluded from it. Skipping is the right
#: way round for cost too: the two pigpio round trips are avoided exactly when
#: the loop is busiest.
B_CHECK_MAX_EDGES = 1

#: The direction cross-check is stricter still: it runs only on windows with no
#: A edge at all. Then the phase has not crossed an integer count, B can have
#: toggled at most once, and a one-step quadrant walk is unambiguous. With one
#: A edge in the window the quadrant may have moved one, two or three steps
#: depending on where in the count the samples fell, and a three-step move reads
#: as the opposite direction -- which is how a correct check reports 64 %
#: agreement on a perfectly healthy encoder.
B_DIRECTION_MAX_EDGES = 0

#: Consecutive low-speed samples where channel A moved and channel B did not
#: before B is called stuck. A disconnected B line reads a constant level; a
#: working one cannot stay put across four counts of wheel rotation.
B_STUCK_SAMPLES = 8

_MINUTE_US = 60.0 * 1_000_000.0


@dataclass(frozen=True, slots=True)
class EncoderSample:
    """One control tick's worth of encoder state, in signed counts and RPM."""

    #: Signed count delta since the previous sample.
    counts: int
    #: Signed lifetime total since the last :meth:`QuadratureEncoder.reset`.
    total: int
    #: Filtered M/T estimate, signed. What the control loop closes around.
    rpm: float
    #: Unfiltered M/T estimate, signed. Useful when tuning; too noisy to control on.
    raw_rpm: float
    #: Unsigned edge count in this window.
    edges: int
    #: No edge for longer than the stall window, so the wheel is stopped rather
    #: than merely slow.
    stale: bool

    @property
    def moving(self) -> bool:
        return not self.stale and self.rpm != 0.0


class MTVelocity:
    """M/T velocity estimation: the window closes on an EDGE, not on the tick.

    Counting edges inside a fixed 10 ms window quantises brutally at low speed.
    M/T instead measures the time between the *first and last edge* of the
    window, so the count is exact by construction and only the timing carries
    error -- and the timing comes from the backend's microsecond ticks, which is
    the whole reason :meth:`GpioBackend.ticks_us` exists.

    When no edge closes the window at all, the estimate is not frozen: speed
    cannot exceed one count in the time since the last edge, so it is capped by
    that ceiling and decays smoothly toward zero. Freezing instead is what makes
    a decelerating wheel read full speed until the stall timer expires, which
    then makes the PID slam the duty to zero a quarter second late.
    """

    __slots__ = ("_scale", "_tau", "_stale_after_us", "rpm", "raw_rpm", "stale", "anomalies")

    def __init__(
        self,
        cpr: int,
        *,
        filter_tau_s: float = VELOCITY_FILTER_TAU_S,
        stale_after_s: float = ENCODER_STALE_WINDOW_S,
    ) -> None:
        if cpr <= 0:
            raise ValueError(f"cpr must be positive, got {cpr}")
        if filter_tau_s < 0.0:
            raise ValueError("filter_tau_s must not be negative")
        if stale_after_s <= 0.0:
            raise ValueError("stale_after_s must be positive")
        #: counts * _scale / microseconds == RPM
        self._scale = _MINUTE_US / cpr
        self._tau = filter_tau_s
        self._stale_after_us = int(stale_after_s * 1_000_000.0)
        self.rpm = 0.0
        self.raw_rpm = 0.0
        self.stale = True
        #: Windows discarded because the span was zero or negative, which means
        #: two callbacks arrived out of order. Non-zero here is worth knowing.
        self.anomalies = 0

    def update(self, edges: int, span_us: int, gap_us: int, dt: float) -> float:
        """Fold one window into the estimate and return the filtered RPM.

        ``span_us`` is the time between the previous window's last edge and this
        window's last edge; pass a non-positive value when there is no earlier
        edge to measure from. ``gap_us`` is the time since the most recent edge.
        Both are unsigned magnitudes; the sign is applied by the caller from the
        commanded direction.
        """
        if gap_us >= self._stale_after_us:
            self.stale = True
            self.raw_rpm = 0.0
            self.rpm = 0.0
            return 0.0

        self.stale = False
        if edges > 0 and span_us > 0:
            self.raw_rpm = edges * self._scale / span_us
        elif edges > 0:
            # First edge after a reset: a count with nothing to measure against.
            self.anomalies += 1
        else:
            if gap_us > 0:
                ceiling = self._scale / gap_us
                if self.raw_rpm > ceiling:
                    self.raw_rpm = ceiling

        if dt > 0.0 and self._tau > 0.0:
            # dt / (tau + dt) rather than 1 - exp(-dt/tau): unconditionally
            # stable for any dt, indistinguishable at dt << tau, and no exp()
            # in a path that runs 200 times a second.
            alpha = dt / (self._tau + dt)
        else:
            alpha = 1.0
        self.rpm += (self.raw_rpm - self.rpm) * alpha
        return self.rpm

    def reset(self) -> None:
        self.rpm = 0.0
        self.raw_rpm = 0.0
        self.stale = True

    @property
    def stale_after_us(self) -> int:
        return self._stale_after_us

    def __repr__(self) -> str:
        return f"MTVelocity(rpm={self.rpm:.1f}, raw={self.raw_rpm:.1f}, stale={self.stale})"


class QuadratureEncoder:
    """One wheel's encoder: x2 on channel A, direction from the command.

    Owns its callback registration and nothing else. The control loop tells it
    which way the bridge is driving via :meth:`set_direction_hint` and reads one
    :class:`EncoderSample` per tick from :meth:`sample`.
    """

    __slots__ = (
        "_gpio", "_pin_a", "_pin_b", "_cpr", "_invert", "_glitch_us",
        "_edges", "_last_edge_us",
        "_edges_seen", "_prev_edge_us", "_have_reference",
        "_hint", "_last_sign", "_total", "_velocity", "_handle",
        "_uncertain", "_b_index", "_b_have_index", "_b_level", "_b_static",
        "b_checks", "b_disagreements",
    )

    def __init__(
        self,
        gpio: GpioBackend,
        pin_a: int,
        pin_b: int,
        *,
        cpr: int,
        invert: bool = False,
        glitch_us: int = ENCODER_GLITCH_US,
    ) -> None:
        if cpr <= 0:
            raise ValueError(f"cpr must be positive, got {cpr}")
        if pin_a == pin_b:
            raise ValueError(f"encoder channels A and B are both GPIO{pin_a}")
        if glitch_us < 0:
            raise ValueError("glitch_us must not be negative")

        self._gpio = gpio
        self._pin_a = pin_a
        self._pin_b = pin_b
        self._cpr = cpr
        self._invert = bool(invert)
        self._glitch_us = glitch_us

        # --- written by the backend's notification thread, read by the loop ---
        self._edges = 0
        self._last_edge_us = 0
        # --- owned exclusively by the control thread -------------------------
        self._edges_seen = 0
        self._prev_edge_us = 0
        self._have_reference = False
        self._hint = 0
        self._last_sign = 0
        self._total = 0
        self._uncertain = False
        self._b_index = 0
        self._b_have_index = False
        self._b_level = False
        self._b_static = 0
        self.b_checks = 0
        self.b_disagreements = 0

        self._velocity = MTVelocity(cpr)

        # Pull-ups because a Hall encoder with an open-collector output needs
        # one and a push-pull output does not care. The glitch filter runs in
        # the daemon, not in Python: at 2200 edges a second, rejecting ringing
        # on this side of the socket would cost more than the ringing does.
        gpio.setup_input(pin_a, Pull.UP, glitch_us)
        gpio.setup_input(pin_b, Pull.UP, glitch_us)
        self._handle: CallbackHandle = gpio.add_edge_callback(pin_a, Edge.BOTH, self._on_edge)

    # -- callback (notification thread) -------------------------------------

    def _on_edge(self, pin: int, level: int, tick_us: int) -> None:
        """Two stores. Nothing else belongs here.

        There is exactly one writer (this thread) and one reader (the control
        thread), and neither ever writes what the other writes, so no lock is
        needed and adding one would only serialise the loop against the daemon.
        """
        self._edges += 1
        self._last_edge_us = tick_us

    # -- control thread -----------------------------------------------------

    def set_direction_hint(self, sign: int) -> None:
        """-1, 0 or +1 from the commanded H-bridge state.

        Zero means "not commanded": the wheel may still be turning, and the
        counts it produces are attributed to the last commanded direction while
        :attr:`direction_uncertain` says the sign is a guess.
        """
        if sign > 0:
            resolved = -1 if self._invert else 1
        elif sign < 0:
            resolved = 1 if self._invert else -1
        else:
            resolved = 0
        self._hint = resolved
        if resolved != 0:
            self._last_sign = resolved

    def sample(self, dt: float = 0.0, now_us: int = -1) -> EncoderSample:
        """Fold everything since the last call into one signed sample.

        ``dt`` and ``now_us`` are optional so the interface's zero-argument form
        still works; the control loop passes both because it already has ``dt``
        and can read the backend tick once for both wheels instead of twice.
        """
        if now_us < 0:
            now_us = self._gpio.ticks_us()

        # Read the counter first and the timestamp second. A callback landing
        # between the two attributes one-count-shifts this window and shifts the
        # next one back by the same count, because both windows share the same
        # saved reference. The error is bounded at one count out of 660 and it
        # cancels; a lock to prevent it would cost far more.
        edges_now = self._edges
        last_edge_us = self._last_edge_us

        edges = edges_now - self._edges_seen
        if edges < 0:  # cannot happen: the counter only grows
            edges = 0
        self._edges_seen = edges_now

        if edges > 0 and self._have_reference:
            span_us = tick_diff(self._prev_edge_us, last_edge_us)
        else:
            span_us = 0
        if edges > 0:
            self._prev_edge_us = last_edge_us
            self._have_reference = True

        if self._have_reference:
            gap_us = tick_diff(self._prev_edge_us, now_us)
            if gap_us < 0:
                gap_us = 0
        else:
            # Nothing has ever counted. Report stale rather than inventing a
            # gap, so a car that boots with a dead encoder says so immediately.
            gap_us = self._velocity.stale_after_us

        magnitude = self._velocity.update(edges, span_us, gap_us, dt)

        direction = self._hint if self._hint != 0 else self._last_sign
        self._uncertain = edges > 0 and self._hint == 0
        counts = edges * direction
        self._total += counts

        if edges <= B_CHECK_MAX_EDGES:
            self._check_channel_b(direction, edges)

        return EncoderSample(
            counts=counts,
            total=self._total,
            rpm=magnitude * direction,
            raw_rpm=self._velocity.raw_rpm * direction,
            edges=edges,
            stale=self._velocity.stale,
        )

    def reset(self) -> None:
        """Zero the accumulated distance without touching the free-running
        counter the callback owns -- that is what keeps this race-free."""
        self._total = 0
        self._edges_seen = self._edges
        self._prev_edge_us = self._last_edge_us
        self._have_reference = False
        self._uncertain = False
        self._b_have_index = False
        self._b_static = 0
        self._velocity.reset()

    def close(self) -> None:
        """Stop delivering edges. Idempotent -- cleanup paths run twice."""
        self._handle.cancel()

    # -- state --------------------------------------------------------------

    @property
    def total_counts(self) -> int:
        return self._total

    @property
    def direction_uncertain(self) -> bool:
        """Counts arrived while nothing was commanded, so the sign is inferred
        from the last commanded direction rather than measured."""
        return self._uncertain

    @property
    def rpm(self) -> float:
        direction = self._hint if self._hint != 0 else self._last_sign
        return self._velocity.rpm * direction

    @property
    def total_edges(self) -> int:
        """Lifetime unsigned edge count, unaffected by :meth:`reset`. The safety
        layer uses it to tell "this encoder has never counted" -- a wiring
        fault -- from "this encoder has stopped counting" -- a stall."""
        return self._edges

    @property
    def cpr(self) -> int:
        return self._cpr

    @property
    def channel_b_stuck(self) -> bool:
        """Channel B has not moved across several counts of wheel rotation.

        A broken or disconnected B wire. Reported rather than acted on: the
        firmware decodes x2 off channel A alone, so a dead B costs nothing but
        the cross-check itself.
        """
        return self._b_static >= B_STUCK_SAMPLES

    @property
    def channel_b_agreement(self) -> float:
        """Fraction of B-channel checks that agreed with the commanded sign.

        Purely diagnostic and deliberately not fed back into the control path.
        A value that settles well below 1.0 means channel B is swapped relative
        to A, or the motor's inversion flag disagrees with how it is wired --
        both of which are bring-up mistakes, not driving conditions.
        """
        if self.b_checks == 0:
            return 1.0
        return 1.0 - (self.b_disagreements / self.b_checks)

    # -- internals ----------------------------------------------------------

    def _check_channel_b(self, direction: int, edges: int) -> None:
        """Cross-check the polled A/B quadrant against the assumed direction.

        Two separate things come out of one pair of reads. A single-step walk
        around the quadrant says which way the wheel is actually going, and
        disagreeing with the commanded sign means A and B are swapped at the
        connector. And a B line that never moves while A does is a broken wire,
        which is the failure this whole check is really here to catch -- a
        stuck level cannot be distinguished from a slow wheel by any amount of
        looking at A alone.
        """
        level_a = self._gpio.read(self._pin_a)
        level_b = self._gpio.read(self._pin_b)

        # Stuck-wire half: needs A to have actually moved, so it runs on windows
        # that carry an edge.
        if edges > 0:
            if level_b == self._b_level:
                self._b_static += 1
            else:
                self._b_static = 0
        self._b_level = level_b
        # Gray code around the cycle: (0,0) (0,1) (1,1) (1,0).
        if level_a:
            index = 2 if level_b else 3
        else:
            index = 1 if level_b else 0

        if self._b_have_index and direction != 0 and edges <= B_DIRECTION_MAX_EDGES:
            # Which way round the quadrant walk runs for "forwards" is a wiring
            # fact, not a mathematical one. The polarity here matches the
            # reference plant model (channel B a quarter cycle behind A), which
            # is what the simulator and every firmware test are built on. A car
            # whose loom has A and B swapped therefore reports a persistently
            # low agreement -- which is exactly the fault this check exists to
            # surface, and it is a connector to reseat, not a constant to edit.
            step = (index - self._b_index) & 3
            if step == 3:
                observed = -1 if self._invert else 1
            elif step == 1:
                observed = 1 if self._invert else -1
            else:
                observed = 0
            if observed != 0:
                self.b_checks += 1
                if observed != direction:
                    self.b_disagreements += 1

        self._b_index = index
        self._b_have_index = True

    def __repr__(self) -> str:
        return (
            f"QuadratureEncoder(A=GPIO{self._pin_a}, B=GPIO{self._pin_b}, "
            f"cpr={self._cpr}, total={self._total}, rpm={self.rpm:.1f})"
        )
