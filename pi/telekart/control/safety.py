"""Everything that decides whether the wheels are allowed to turn.

The control loop computes what the driver asked for; this decides whether the
car does it. Keeping the two apart means the arming rules, the failsafe
schedule and the hardware protections can be reasoned about -- and tested --
without a plant model, and it means there is exactly one place to look when the
car refuses to move.

Four things live here that are worth reading before changing anything:

**Arming needs all four conditions.** An explicit ARM, throttle held at neutral,
a valid session, and no active critical fault. Any one of them alone is a car
that lurches when a laptop reconnects.

**The failsafe is staged: coast, brake, coast, disarm.** Coasting first, because
the most likely cause of a 200 ms gap is a WiFi hiccup and braking through every
one of those is miserable to drive. Braking second, because an unbraked car
keeps rolling for metres and the whole point is that it stops. Coasting again
before the disarm so the bridge is not holding a short when the enable finally
drops.

**Stall detection is hardware protection.** Two stalled motors put roughly 7 W
into a bridge whose stock heatsink handles two or three. This is not a nicety
and it is not about the motors.

**A brownout is not a stall.** Both encoders reaching zero *at the same moment*
while both are commanded is the boost regulator folding back, not two
simultaneous mechanical jams. It gets its own fault code, its own response --
back off and let the converter restart, because continuing to demand current
keeps it in hiccup mode -- and it must never be diagnosed as a stall, because
the fix for one is nothing like the fix for the other.
"""

from __future__ import annotations

from dataclasses import dataclass

from telekart_protocol import (
    CRITICAL_FAULTS,
    FAILSAFE_BRAKE_AT_MS,
    FAILSAFE_COAST_AT_MS,
    FAILSAFE_DISARM_AT_MS,
    Fault,
    VehicleState,
)

from ..config import VehicleConfig
from ..log import RateLimiter, get_logger
from ..util.clock import Clock

_log = get_logger(__name__)

#: Applied duty above which "commanded but not turning" means something. The
#: Darlington bridge does not conduct at all below roughly 0.22 duty, so a
#: threshold under that would report a stall every time the loop asked for a
#: gentle creep. 0.30 leaves headroom above the deadband on a sagging rail.
STALL_DUTY_THRESHOLD = 0.30

#: Throttle at or below this counts as neutral for the arming hold.
NEUTRAL_THROTTLE = 0.02

#: Both wheels must reach zero within this of each other to read as a supply
#: collapse rather than as two independent stalls.
BROWNOUT_SIMULTANEITY_S = 0.12

#: ...and must have *been turning* this recently. Without it, every launch is a
#: brownout: at the moment the duty first clears the conduction threshold both
#: wheels are commanded and both are stationary, which is the same evidence a
#: collapse produces. A supply collapse stops wheels that were already moving.
BROWNOUT_MOTION_MEMORY_S = 0.60

#: ...and the pair must stay down this long. One tick of both wheels reading
#: zero is a quantisation artefact at low speed, not a converter folding back.
BROWNOUT_CONFIRM_S = 0.06

#: The fast brownout detector: both wheels losing this fraction of their speed
#: inside `BROWNOUT_DROP_WINDOW_S` while both are commanded. Catches the
#: converter folding back before either wheel actually reaches zero, which
#: matters because a hiccup cycle is only ~350 ms long and the wheels do not get
#: all the way to a standstill inside one.
#:
#: A quarter of the speed in an eighth of a second, on BOTH wheels, while both
#: are commanded and neither is braking, is not something a surface change does:
#: two wheels a track-width apart do not meet the same patch of floor at the
#: same instant. The matched-rate test below is what makes that argument hold.
BROWNOUT_DROP_FRACTION = 0.25
BROWNOUT_DROP_WINDOW_S = 0.12
#: How closely the two wheels' fractional losses must agree. A supply collapse
#: takes both by the same proportion; anything mechanical does not.
BROWNOUT_DROP_MATCH = 0.20
#: Speed a wheel must have been doing for a collapse to be meaningful.
BROWNOUT_MIN_RPM = 25.0

#: Hold everything off this long after a brownout. The converter needs its
#: output to come back up with no load on it; demanding current through the
#: recovery is what turns one hiccup into a continuous oscillation.
BROWNOUT_RECOVERY_S = 0.50
#: Then run reduced for this long, so the driver cannot immediately re-trip it.
BROWNOUT_DERATE_S = 2.00
BROWNOUT_DERATE_CEILING = 0.55

#: After a stall trips, the bridge is held off until the throttle has come back
#: to neutral AND this much time has passed. Sticky-forever would strand the car
#: on the far side of the track over one kerb strike; auto-clearing without the
#: neutral requirement would let a driver hold the throttle into an obstruction
#: and cook the bridge in repeated `stall_detect_ms` bursts.
STALL_COOLDOWN_S = 2.0

#: A wheel whose partner is turning this fast while it has produced no edges at
#: all since arming is reporting a wiring fault, not a jam.
ENCODER_FAIL_PEER_RPM = 30.0

#: Below this pack voltage the duty ceiling is derated. Sensing is optional
#: hardware; a reported 0 V means "not fitted" and disables every battery rule.
LOW_BATTERY_DERATE = 0.70

_FAULT_LOG_INTERVAL_S = 2.0


@dataclass(slots=True)
class FeedbackSnapshot:
    """What the loop measured this tick, as the safety layer needs to see it.

    Mutable and preallocated by the control loop, then refilled in place every
    tick with :meth:`set`. A frozen dataclass would be tidier to reason about
    and would allocate a hundred objects a second inside the one code path that
    is not allowed to allocate.
    """

    rpm_l: float = 0.0
    rpm_r: float = 0.0
    #: Duty as *applied* by the driver, not as requested by the controller.
    #: Diagnosing a stall from a command the dead-time sequencer suppressed
    #: would report a stall every direction change.
    duty_l: float = 0.0
    duty_r: float = 0.0
    #: Unsigned lifetime edge counts, used to tell "never counted" from
    #: "stopped counting".
    total_edges_l: int = 0
    total_edges_r: int = 0
    stale_l: bool = False
    stale_r: bool = False
    braking: bool = False
    #: 0.0 when no battery sensing is fitted, which disables the battery rules.
    pack_volts: float = 0.0
    cpu_temp_c: float = 0.0
    #: Raw vcgencmd get_throttled bitmask, 0 when unknown.
    throttled: int = 0
    #: Non-None when the backend recorded a failure it could not raise.
    gpio_error: str | None = None

    def set(
        self,
        *,
        rpm_l: float,
        rpm_r: float,
        duty_l: float,
        duty_r: float,
        total_edges_l: int,
        total_edges_r: int,
        stale_l: bool,
        stale_r: bool,
        braking: bool,
    ) -> None:
        """Refill the drivetrain half in place. Health fields are written by the
        slower monitor thread and deliberately not touched here."""
        self.rpm_l = rpm_l
        self.rpm_r = rpm_r
        self.duty_l = duty_l
        self.duty_r = duty_r
        self.total_edges_l = total_edges_l
        self.total_edges_r = total_edges_r
        self.stale_l = stale_l
        self.stale_r = stale_r
        self.braking = braking


@dataclass(frozen=True, slots=True)
class SafetyOutput:
    """The verdict. Immutable, and reused when nothing changed."""

    allow_drive: bool
    #: 0.0 means no forced brake. Anything higher is applied instead of the
    #: driver's command, not on top of it.
    force_brake: float
    force_coast: bool
    #: Hard ceiling on any single motor's duty for this tick.
    duty_ceiling: float


class SafetyStateMachine:
    """Owns :class:`VehicleState` and the sticky fault word.

    The car is the authority on both. The desktop app never asserts that it is
    armed -- it displays what telemetry reports -- which means every path into
    ARMED has to go through this object.
    """

    __slots__ = (
        "_config", "_clock", "_state", "_faults", "_session_valid",
        "_last_packet_at", "_neutral_since", "_armed_at", "_uptime",
        "_stalled_since_l", "_stalled_since_r", "_stall_hold_until", "_stall_hold",
        "_edges_at_arm_l", "_edges_at_arm_r", "_last_edges_l", "_last_edges_r",
        "_history_rpm_l", "_history_rpm_r", "_history_at",
        "_moving_at_l", "_moving_at_r",
        "_brownout_until", "_derate_until", "_last_output", "_fault_limiter",
        "_failsafe_stage", "brownout_count", "stall_count",
        "failsafe_count",
    )

    def __init__(self, config: VehicleConfig, clock: Clock) -> None:
        self._config = config
        self._clock = clock
        self._state = VehicleState.BOOT
        self._faults = Fault.NONE
        self._session_valid = False

        now = clock.monotonic()
        # Anchored at construction so a car that has never had a packet is not
        # instantly "stale by however long the process has been up".
        self._last_packet_at = now
        self._neutral_since: float | None = now
        self._armed_at = 0.0
        self._uptime = 0.0

        self._stalled_since_l: float | None = None
        self._stalled_since_r: float | None = None
        self._stall_hold_until = 0.0
        self._stall_hold = False
        self._edges_at_arm_l = 0
        self._edges_at_arm_r = 0
        self._last_edges_l = 0
        self._last_edges_r = 0

        # One-sample history for the fast brownout detector. Two floats and a
        # timestamp rather than a ring buffer: the test is "was it much faster
        # a moment ago", and a moment is all that needs remembering.
        self._history_rpm_l = 0.0
        self._history_rpm_r = 0.0
        self._history_at = now
        # Last time each wheel was demonstrably turning. Far enough in the past
        # at construction that a car that has never moved cannot brown out.
        self._moving_at_l = now - BROWNOUT_MOTION_MEMORY_S * 10.0
        self._moving_at_r = now - BROWNOUT_MOTION_MEMORY_S * 10.0

        self._brownout_until = 0.0
        self._derate_until = 0.0
        self._failsafe_stage = 0
        self._last_output = SafetyOutput(False, 0.0, True, config.max_duty)
        self._fault_limiter = RateLimiter(_FAULT_LOG_INTERVAL_S)

        self.brownout_count = 0
        self.stall_count = 0
        self.failsafe_count = 0

    # -- requests -----------------------------------------------------------

    def request_arm(self) -> tuple[bool, str]:
        """Arm, or say why not. All four conditions, checked in a fixed order
        so the operator sees the most fundamental problem first."""
        if self._state is VehicleState.ESTOP:
            return (False, "emergency stop is latched; clear it first")
        if self._state is VehicleState.ARMED:
            return (True, "")
        critical = self._faults & CRITICAL_FAULTS
        if critical:
            return (False, f"critical fault active: {_fault_names(critical)}")
        if not self._session_valid:
            return (False, "no valid session")
        if self._neutral_since is None:
            return (False, "throttle is not at neutral")
        held = self._clock.monotonic() - self._neutral_since
        required = self._config.arm_neutral_s
        if held < required:
            return (
                False,
                f"hold the throttle at neutral for {required - held:.1f}s more",
            )

        now = self._clock.monotonic()
        self._state = VehicleState.ARMED
        self._armed_at = now
        self._failsafe_stage = 0
        # The link is only known to be alive if a packet has arrived recently;
        # arming does not fabricate one. But the failsafe clock starts now, so
        # a car armed over TCP with no UDP stream disarms itself in a second
        # instead of sitting armed and unreachable.
        if now - self._last_packet_at > self._config.control_timeout_s:
            self._last_packet_at = now
        self._stalled_since_l = None
        self._stalled_since_r = None
        self._stall_hold_until = 0.0
        self._stall_hold = False
        self._edges_at_arm_l = self._last_edges_l
        self._edges_at_arm_r = self._last_edges_r
        _log.info("armed", faults=int(self._faults))
        return (True, "")

    def request_disarm(self) -> None:
        """Always accepted. There is never a reason to refuse a disarm."""
        if self._state in (VehicleState.ARMED, VehicleState.FAILSAFE):
            _log.info("disarmed", armed_for=self._clock.monotonic() - self._armed_at)
        if self._state is not VehicleState.ESTOP:
            self._state = self._resting_state()
        self._failsafe_stage = 0
        self._stalled_since_l = None
        self._stalled_since_r = None

    def request_estop(self) -> None:
        """Latch the emergency stop. Reachable from every state, including BOOT."""
        if self._state is not VehicleState.ESTOP:
            _log.warning("emergency stop latched", state=self._state.name)
        self._state = VehicleState.ESTOP
        self._faults |= Fault.ESTOP_LATCHED
        self._failsafe_stage = 0

    def clear_estop(self) -> tuple[bool, str]:
        """Release the latch. Requires the throttle to be at neutral first,
        because the alternative is a car that leaps when the button is reset."""
        if self._state is not VehicleState.ESTOP:
            return (True, "")
        if self._neutral_since is None:
            return (False, "return the throttle to neutral before clearing")
        self._faults &= ~Fault.ESTOP_LATCHED
        self._state = self._resting_state()
        _log.info("emergency stop cleared", state=self._state.name)
        return (True, "")

    def clear_faults(self) -> None:
        """Acknowledge every fault the operator can acknowledge.

        The E-stop latch is not one of them while the E-stop is engaged -- that
        has its own release, deliberately, so that "clear faults" can never be
        the thing that un-presses an emergency stop.
        """
        if self._state is VehicleState.ESTOP:
            keep = self._faults & Fault.ESTOP_LATCHED
        else:
            keep = Fault.NONE
        if self._faults != keep:
            _log.info("faults cleared", was=_fault_names(self._faults))
        self._faults = keep
        self._stalled_since_l = None
        self._stalled_since_r = None
        self._stall_hold_until = 0.0
        self._stall_hold = False
        self._derate_until = 0.0
        self._brownout_until = 0.0
        if self._state is VehicleState.FAULT and not (self._faults & CRITICAL_FAULTS):
            self._state = VehicleState.SAFE

    def raise_fault(self, fault: Fault, detail: str = "") -> None:
        """Record a fault. Sticky until cleared; critical ones force a disarm."""
        if fault is Fault.NONE:
            return
        fresh = fault & ~self._faults
        self._faults |= fault
        if fresh:
            _log.warning("fault raised", fault=_fault_names(fresh), detail=detail)
        elif self._fault_limiter.allow(self._clock.monotonic()):
            _log.debug(
                "fault still active",
                fault=_fault_names(fault),
                detail=detail,
                suppressed=self._fault_limiter.take_suppressed(),
            )
        if fault & CRITICAL_FAULTS and self._state not in (
            VehicleState.ESTOP,
            VehicleState.FAULT,
        ):
            self._state = VehicleState.FAULT
            _log.error("critical fault forced a disarm", fault=_fault_names(fault))

    def _resting_state(self) -> VehicleState:
        """Where a car that is not driving belongs: SAFE, or FAULT if something
        critical is still outstanding."""
        return VehicleState.FAULT if (self._faults & CRITICAL_FAULTS) else VehicleState.SAFE

    def note_control_packet(self) -> None:
        """A valid, authenticated control packet arrived. Resets the failsafe
        clock, and nothing else -- the recovery transition happens in the loop
        so it is visible in one place."""
        self._last_packet_at = self._clock.monotonic()

    def set_session_valid(self, valid: bool) -> None:
        """Told by the session layer. One of the four arming conditions, and a
        disarm trigger on the way down: a car whose operator has gone away must
        not stay armed waiting for the failsafe to notice."""
        if valid == self._session_valid:
            return
        self._session_valid = valid
        if not valid and self._state in (VehicleState.ARMED, VehicleState.FAILSAFE):
            _log.warning("session ended while armed; disarming")
            self.request_disarm()

    # -- per-tick -----------------------------------------------------------

    def update(
        self, dt: float, *, throttle: float, measured: FeedbackSnapshot
    ) -> SafetyOutput:
        """One tick. Returns the verdict; never raises.

        Order matters: neutral tracking first because arming depends on it,
        then health, then the drivetrain diagnosis, then the link. The link
        check is last so that a stale link cannot mask a fault the operator
        needs to see when the link comes back.
        """
        if dt > 0.0:
            self._uptime += dt
        now = self._clock.monotonic()

        if self._state is VehicleState.BOOT:
            # BOOT exists so telemetry can distinguish "starting up" from
            # "ready and refusing to arm". One tick is all it lasts.
            self._state = VehicleState.SAFE

        if throttle <= NEUTRAL_THROTTLE and throttle == throttle:
            if self._neutral_since is None:
                self._neutral_since = now
        else:
            self._neutral_since = None

        self._last_edges_l = measured.total_edges_l
        self._last_edges_r = measured.total_edges_r

        self._check_health(measured)
        self._check_drivetrain(now, measured)
        stale_for = self._check_link(now)

        return self._compose(now, stale_for)

    # -- diagnosis ----------------------------------------------------------

    def _check_health(self, measured: FeedbackSnapshot) -> None:
        detail = measured.gpio_error
        if detail is not None:
            self.raise_fault(Fault.GPIO_ERROR, detail)

        volts = measured.pack_volts
        if volts > 0.0:
            config = self._config
            if config.critical_battery_v > 0.0 and volts < config.critical_battery_v:
                self.raise_fault(Fault.CRITICAL_BATTERY, "pack below cutoff")
            elif config.low_battery_v > 0.0 and volts < config.low_battery_v:
                self.raise_fault(Fault.LOW_BATTERY, "pack low")

    def _check_drivetrain(self, now: float, measured: FeedbackSnapshot) -> None:
        """Stall, brownout and encoder failure, in the order they must be tried.

        Brownout first, because both of the others would otherwise claim the
        same evidence and one of them would be wrong.
        """
        config = self._config
        stall_rpm = config.stall_rpm_threshold

        rpm_l = measured.rpm_l
        rpm_r = measured.rpm_r
        magnitude_l = rpm_l if rpm_l >= 0.0 else -rpm_l
        magnitude_r = rpm_r if rpm_r >= 0.0 else -rpm_r
        duty_l = measured.duty_l
        duty_r = measured.duty_r
        commanded_l = (duty_l if duty_l >= 0.0 else -duty_l) >= STALL_DUTY_THRESHOLD
        commanded_r = (duty_r if duty_r >= 0.0 else -duty_r) >= STALL_DUTY_THRESHOLD

        if magnitude_l >= BROWNOUT_MIN_RPM:
            self._moving_at_l = now
        if magnitude_r >= BROWNOUT_MIN_RPM:
            self._moving_at_r = now

        if commanded_l and magnitude_l <= stall_rpm:
            if self._stalled_since_l is None:
                self._stalled_since_l = now
        else:
            self._stalled_since_l = None
        if commanded_r and magnitude_r <= stall_rpm:
            if self._stalled_since_r is None:
                self._stalled_since_r = now
        else:
            self._stalled_since_r = None

        if self._detect_brownout(now, measured, commanded_l, commanded_r,
                                 magnitude_l, magnitude_r):
            return

        window = config.stall_detect_s
        peer_moving_r = magnitude_r >= ENCODER_FAIL_PEER_RPM
        peer_moving_l = magnitude_l >= ENCODER_FAIL_PEER_RPM
        if self._stalled_since_l is not None and now - self._stalled_since_l >= window:
            self._diagnose_stalled_wheel(
                now,
                left=True,
                never_counted=measured.total_edges_l <= self._edges_at_arm_l,
                peer_moving=peer_moving_r,
            )
        if self._stalled_since_r is not None and now - self._stalled_since_r >= window:
            self._diagnose_stalled_wheel(
                now,
                left=False,
                never_counted=measured.total_edges_r <= self._edges_at_arm_r,
                peer_moving=peer_moving_l,
            )

    def _detect_brownout(
        self,
        now: float,
        measured: FeedbackSnapshot,
        commanded_l: bool,
        commanded_r: bool,
        magnitude_l: float,
        magnitude_r: float,
    ) -> bool:
        """True when this looks like the converter folding back.

        Two detectors. The slow one is the specified signature -- both wheels at
        zero within `BROWNOUT_SIMULTANEITY_S` of each other while commanded.
        The fast one catches the collapse on the way down, roughly a quarter
        second earlier, which is the difference between backing off in time and
        riding out a full hiccup cycle. Both require *both* wheels: one wheel
        doing something odd is never a supply problem.
        """
        if not (commanded_l and commanded_r) or measured.braking:
            self._history_at = now
            self._history_rpm_l = magnitude_l
            self._history_rpm_r = magnitude_r
            return False

        collapsed = False
        left_at = self._stalled_since_l
        right_at = self._stalled_since_r
        if left_at is not None and right_at is not None:
            skew = left_at - right_at
            if (
                -BROWNOUT_SIMULTANEITY_S <= skew <= BROWNOUT_SIMULTANEITY_S
                and now - left_at >= BROWNOUT_CONFIRM_S
                and now - right_at >= BROWNOUT_CONFIRM_S
                and now - self._moving_at_l <= BROWNOUT_MOTION_MEMORY_S
                and now - self._moving_at_r <= BROWNOUT_MOTION_MEMORY_S
            ):
                collapsed = True

        elapsed = now - self._history_at
        if elapsed >= BROWNOUT_DROP_WINDOW_S:
            was_l = self._history_rpm_l
            was_r = self._history_rpm_r
            if was_l >= BROWNOUT_MIN_RPM and was_r >= BROWNOUT_MIN_RPM:
                lost_l = (was_l - magnitude_l) / was_l
                lost_r = (was_r - magnitude_r) / was_r
                mismatch = lost_l - lost_r
                if mismatch < 0.0:
                    mismatch = -mismatch
                if (
                    lost_l >= BROWNOUT_DROP_FRACTION
                    and lost_r >= BROWNOUT_DROP_FRACTION
                    and mismatch <= BROWNOUT_DROP_MATCH
                ):
                    collapsed = True
            self._history_at = now
            self._history_rpm_l = magnitude_l
            self._history_rpm_r = magnitude_r

        if not collapsed:
            return False

        if now >= self._brownout_until:
            self.brownout_count += 1
            self.raise_fault(
                Fault.BROWNOUT,
                "both wheels collapsed together under load",
            )
        self._brownout_until = now + BROWNOUT_RECOVERY_S
        self._derate_until = now + BROWNOUT_RECOVERY_S + BROWNOUT_DERATE_S
        # Clear the stall timers: this evidence has been spent on the brownout
        # and must not be counted twice.
        self._stalled_since_l = None
        self._stalled_since_r = None
        return True

    def _diagnose_stalled_wheel(
        self, now: float, *, left: bool, never_counted: bool, peer_moving: bool
    ) -> None:
        """A wheel commanded but not turning. Wiring fault, or a jam?

        There is no current sensor on this car, so the discriminator is history:
        an encoder that has produced no edges *at all* since arming while its
        partner is spinning is almost always a disconnected wire, and an encoder
        that was counting a moment ago and has stopped is almost always a jam.
        The heuristic is not perfect -- a wheel that jams on the very first
        command reads as a wiring fault -- and it is stated here rather than
        hidden so the failure mode is at least discoverable.
        """
        if never_counted and peer_moving:
            fault = Fault.ENCODER_FAIL_L if left else Fault.ENCODER_FAIL_R
            if not (self._faults & fault):
                self.raise_fault(
                    fault,
                    "no counts since arming while the other wheel turns",
                )
            # Deliberately no stop and no ceiling change. The control loop drops
            # this wheel to open-loop feedforward and keeps driving: losing an
            # encoder halfway around a track is a reason to finish the lap, not
            # a reason to stop dead in front of whatever is behind you.
            if left:
                self._stalled_since_l = None
            else:
                self._stalled_since_r = None
            return

        fault = Fault.STALL_L if left else Fault.STALL_R
        if not (self._faults & fault):
            self.stall_count += 1
        self.raise_fault(fault, "commanded but not turning")
        self._stall_hold = True
        self._stall_hold_until = now + STALL_COOLDOWN_S
        if left:
            self._stalled_since_l = None
        else:
            self._stalled_since_r = None

    def _check_link(self, now: float) -> float:
        """Seconds the link has been stale. Zero or negative means healthy."""
        stale_for = (now - self._last_packet_at) - self._config.control_timeout_s
        if self._state not in (VehicleState.ARMED, VehicleState.FAILSAFE):
            return stale_for

        if stale_for <= 0.0:
            if self._state is VehicleState.FAILSAFE:
                _log.info("control link recovered", stage=self._failsafe_stage)
                self._state = VehicleState.ARMED
                self._failsafe_stage = 0
            return stale_for

        if self._state is VehicleState.ARMED:
            self._state = VehicleState.FAILSAFE
            self.failsafe_count += 1
            _log.warning("control link stale; entering failsafe")

        if stale_for * 1000.0 >= FAILSAFE_DISARM_AT_MS:
            self.raise_fault(Fault.CONTROL_TIMEOUT, "link never recovered")
            self.request_disarm()
        return stale_for

    # -- verdict ------------------------------------------------------------

    def _compose(self, now: float, stale_for: float) -> SafetyOutput:
        config = self._config
        ceiling = config.max_duty
        allow = False
        brake = 0.0
        coast = True

        if self._state is VehicleState.ARMED:
            allow = True
            coast = False
        elif self._state is VehicleState.FAILSAFE:
            # Staged from the moment the link went stale, not from the last
            # packet: the timeout itself is already a coast, and stacking the
            # two would make the brake arrive a fifth of a second late.
            elapsed_ms = stale_for * 1000.0
            if elapsed_ms >= FAILSAFE_COAST_AT_MS:
                stage = 3
            elif elapsed_ms >= FAILSAFE_BRAKE_AT_MS:
                stage = 2
                brake = config.failsafe_brake_duty
            else:
                stage = 1
            if stage != self._failsafe_stage:
                self._failsafe_stage = stage
            coast = brake <= 0.0

        # Protections override the state, in increasing order of severity, so
        # the strictest one wins regardless of the order they were detected in.
        if now < self._brownout_until:
            allow = False
            brake = 0.0
            coast = True
        elif now < self._derate_until and ceiling > BROWNOUT_DERATE_CEILING:
            ceiling = BROWNOUT_DERATE_CEILING

        if self._stall_hold:
            # Released once the bridge has had its cooldown *and* the driver has
            # come off the throttle. Both, because a hold that never released
            # would strand the car over one kerb strike, and a hold that
            # released on time alone would let a driver lean on the throttle and
            # cook the bridge in repeated `stall_detect_ms` bursts. The fault
            # bit stays set either way -- it is diagnosis, not enforcement.
            if now >= self._stall_hold_until and self._neutral_since is not None:
                self._stall_hold = False
            else:
                allow = False
                brake = 0.0
                coast = True

        if self._faults & Fault.LOW_BATTERY and not (self._faults & Fault.CRITICAL_BATTERY):
            derated = config.max_duty * LOW_BATTERY_DERATE
            if derated < ceiling:
                ceiling = derated

        if self._faults & CRITICAL_FAULTS:
            allow = False
            brake = 0.0
            coast = True
            ceiling = 0.0

        previous = self._last_output
        if (
            previous.allow_drive == allow
            and previous.force_brake == brake
            and previous.force_coast == coast
            and previous.duty_ceiling == ceiling
        ):
            # Immutable, so handing back the same object is safe and skips an
            # allocation on the ~99 % of ticks where nothing changed.
            return previous
        output = SafetyOutput(allow, brake, coast, ceiling)
        self._last_output = output
        return output

    # -- state --------------------------------------------------------------

    @property
    def state(self) -> VehicleState:
        return self._state

    @property
    def faults(self) -> Fault:
        return self._faults

    @property
    def session_valid(self) -> bool:
        return self._session_valid

    @property
    def armed(self) -> bool:
        return self._state is VehicleState.ARMED

    @property
    def failsafe_stage(self) -> int:
        """0 healthy, 1 coasting, 2 braking, 3 coasting before the disarm."""
        return self._failsafe_stage

    @property
    def encoder_failed_l(self) -> bool:
        return bool(self._faults & Fault.ENCODER_FAIL_L)

    @property
    def encoder_failed_r(self) -> bool:
        return bool(self._faults & Fault.ENCODER_FAIL_R)

    @property
    def uptime(self) -> float:
        """Seconds of loop time accumulated from the ``dt`` handed to update."""
        return self._uptime

    def neutral_held(self) -> float:
        """How long the throttle has been at neutral, in seconds. Zero when it
        is not. The desktop app shows this as an arming progress bar."""
        if self._neutral_since is None:
            return 0.0
        return self._clock.monotonic() - self._neutral_since

    def __repr__(self) -> str:
        return (
            f"SafetyStateMachine(state={self._state.name}, "
            f"faults={_fault_names(self._faults)}, "
            f"session={self._session_valid})"
        )


def _fault_names(faults: Fault) -> str:
    """Readable fault list. Only ever called from a log path, never per tick."""
    if faults is Fault.NONE or not faults:
        return "none"
    return "|".join(flag.name or str(flag) for flag in Fault if flag and (faults & flag))
