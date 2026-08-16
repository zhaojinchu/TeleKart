#!/usr/bin/env python3
"""End-to-end smoke test: a whole car, on MockBackend + FakeClock, in virtual time.

This is the assembly check the unit suites cannot be. Each of them exercises one
seam against fixtures; this one builds the real object graph the way ``app.py``
does -- backend, both encoders, the motor pair, the servo, the safety machine,
the odometer, the calibration and the controller -- arms it, drives a throttle
step, and then stops feeding it and watches the failsafe ladder run.

It asserts two things that only appear once everything is connected:

1. **The loop genuinely closes.** RPM converges on the target the controller
   itself reports, which it can only do if the encoder decode, the feedforward
   table, the PID and the H-bridge truth table all agree about which way is
   forward and how fast that is.
2. **The failsafe ladder runs on the protocol's schedule** -- coast, brake,
   coast, disarm -- at ``FAILSAFE_*_AT_MS``, measured from the moment the link
   went stale rather than from the last packet.

Nothing here sleeps and nothing needs hardware: a simulated drive of several
seconds runs in milliseconds and returns identical numbers on every machine.
Run it with ``TELEKART_BACKEND=mock python pi/scripts/smoke_drive.py``.

Exit status is 0 when every check passed, 1 otherwise.
"""

from __future__ import annotations

import sys
from pathlib import Path

# The shared protocol package lives in a sibling directory that no amount of
# rootdir inference will find, so make a plain `python pi/scripts/smoke_drive.py`
# work from a fresh checkout. A proper editable install makes this a no-op.
_PI_ROOT = Path(__file__).resolve().parents[1]
_REPO_ROOT = _PI_ROOT.parent
for _candidate in (_PI_ROOT, _REPO_ROOT / "packages" / "telekart_protocol"):
    _text = str(_candidate)
    if _candidate.is_dir() and _text not in sys.path:
        sys.path.insert(0, _text)

from telekart.calibration import DriveCalibration  # noqa: E402
from telekart.config import VehicleConfig  # noqa: E402
from telekart.constants import CONTROL_PERIOD_S, ENCODER_GLITCH_US  # noqa: E402
from telekart.control.drive import DriveCommand, DriveController  # noqa: E402
from telekart.control.safety import SafetyStateMachine  # noqa: E402
from telekart.drivers.encoder import QuadratureEncoder  # noqa: E402
from telekart.drivers.motor import MotorPair  # noqa: E402
from telekart.drivers.servo import SteeringServo  # noqa: E402
from telekart.hal.mock_backend import MockBackend, MockPins, PlantParams  # noqa: E402
from telekart.odometry import BicycleOdometry  # noqa: E402
from telekart.util.clock import FakeClock  # noqa: E402

from telekart_protocol import ControlFlags, VehicleState  # noqa: E402
from telekart_protocol.constants import (  # noqa: E402
    FAILSAFE_BRAKE_AT_MS,
    FAILSAFE_COAST_AT_MS,
    FAILSAFE_DISARM_AT_MS,
)

WHEEL_KEYS = ("left_fwd", "left_rev", "right_fwd", "right_rev")

#: What the mock drivetrain actually reaches at ``max_duty`` on one wheel, and
#: the duty at which it breaks away. Both fall out of the plant model rather
#: than being chosen, and the synthetic calibration below has to agree with them
#: or the loop is running a feedforward table that does not describe its plant.
MOCK_PLANT_MAX_RPM = 139.0
MOCK_PLANT_DEADBAND = 0.22


# --------------------------------------------------------------------------
# Assembly
# --------------------------------------------------------------------------


def synthetic_calibration() -> DriveCalibration:
    """A calibration that describes the mock plant.

    The feedforward table is expressed *relative to the deadband*, because the
    pipeline applies deadband compensation after ``feedforward + PID``. Baking
    the deadband into the table as well would count it twice and put a step at
    the bottom of the throttle that no PID gain can remove.
    """
    span = MOCK_PLANT_MAX_RPM
    slope = 0.63 / span  # 0.63 of duty above the deadband spans rest -> ceiling
    lut = [
        (round(f * span, 3), round(f * span * slope, 5))
        for f in (0.0, 0.2, 0.4, 0.6, 0.8, 1.0)
    ]
    return DriveCalibration(
        max_rpm={key: MOCK_PLANT_MAX_RPM for key in WHEEL_KEYS},
        deadband={key: MOCK_PLANT_DEADBAND for key in WHEEL_KEYS},
        ff_lut={key: list(lut) for key in WHEEL_KEYS},
        measured_at="2026-01-01T00:00:00Z",
        on_ground=False,
    )


class Car:
    """The full object graph, plus the loop that drives it in virtual time."""

    def __init__(self) -> None:
        # Virtual time starts non-zero on purpose: a clock starting at 0.0 makes
        # an uninitialised `last_seen = 0.0` look like "seen just now", and that
        # bug then hides until the car has been up a while.
        self.clock = FakeClock(start=1000.0)
        self.config = VehicleConfig()
        self.gpio = MockBackend(
            pins=MockPins.from_hardware_pins(self.config.pins),
            params=PlantParams(cpr=self.config.encoder_cpr),
            clock=self.clock,
            seed=1,
            strict=True,  # rejects a single-channel PWM write outright
        )
        self.gpio.plant.cpr = self.config.encoder_cpr

        self.motors = MotorPair(
            self.gpio, self.config.pins.motors, self.config, self.clock
        )
        self.servo = SteeringServo(
            self.gpio, self.config.pins.servo, self.config, self.clock
        )
        encoders = self.config.pins.encoders
        self.encoder_l = QuadratureEncoder(
            self.gpio, encoders.left_a, encoders.left_b,
            cpr=self.config.encoder_cpr,
            invert=self.config.encoder_invert_left,
            glitch_us=ENCODER_GLITCH_US,
        )
        self.encoder_r = QuadratureEncoder(
            self.gpio, encoders.right_a, encoders.right_b,
            cpr=self.config.encoder_cpr,
            invert=self.config.encoder_invert_right,
            glitch_us=ENCODER_GLITCH_US,
        )
        self.safety = SafetyStateMachine(self.config, self.clock)
        self.odometry = BicycleOdometry(self.config)
        self.calibration = synthetic_calibration()
        self.controller = DriveController(
            gpio=self.gpio,
            config=self.config,
            clock=self.clock,
            motors=self.motors,
            servo=self.servo,
            encoder_l=self.encoder_l,
            encoder_r=self.encoder_r,
            safety=self.safety,
            odometry=self.odometry,
            calibration=self.calibration,
        )
        self.last_state = None

    # -- driving ------------------------------------------------------------

    def submit(self, **cmd: object) -> None:
        self.controller.submit_command(
            DriveCommand(
                steering=float(cmd.get("steering", 0.0)),
                throttle=float(cmd.get("throttle", 0.0)),
                brake=float(cmd.get("brake", 0.0)),
                flags=cmd.get("flags", ControlFlags.NONE),  # type: ignore[arg-type]
                received_at=self.clock.monotonic(),
            )
        )
        self.safety.note_control_packet()

    def step(self, dt: float = CONTROL_PERIOD_S):
        """Time passes, the plant integrates *last* tick's command and emits the
        edges that motion produced, and only then does the controller read them.

        Stepping before ticking is what preserves the one-tick delay that
        closed-loop stability actually depends on.
        """
        self.clock.advance(dt)
        self.gpio.step(dt)
        self.last_state = self.controller.tick(dt)
        return self.last_state

    def run(self, seconds: float, command: dict | None = None):
        remaining = seconds
        while remaining > 1e-12:
            slice_s = CONTROL_PERIOD_S if remaining >= CONTROL_PERIOD_S else remaining
            if command is not None:
                self.submit(**command)
            self.step(slice_s)
            remaining -= slice_s
        return self.last_state

    def arm(self) -> tuple[bool, str]:
        """Satisfy all four preconditions, then ask.

        The session flag is owned by the TCP layer on the real car, so nothing
        in the control loop can set it for itself. Neutral throttle has to have
        been held for `arm_neutral_ms` *before* the request, which is the whole
        point of that parameter -- so hold neutral first and ask afterwards.
        """
        self.safety.set_session_valid(True)
        self.run(self.config.arm_neutral_s + 0.25, command={"throttle": 0.0})
        accepted, reason = self.safety.request_arm()
        if accepted:
            self.run(2 * CONTROL_PERIOD_S, command={"throttle": 0.0})
        return accepted, reason

    # -- views --------------------------------------------------------------

    @property
    def plant_rpm(self) -> tuple[float, float]:
        """Plant truth, not the estimate."""
        return (self.gpio.wheel_rpm("left"), self.gpio.wheel_rpm("right"))

    @property
    def duty(self) -> tuple[float, float]:
        pins = self.config.pins.motors
        return (self.gpio.pwm_duty(pins.ena), self.gpio.pwm_duty(pins.enb))


# --------------------------------------------------------------------------
# Checks
# --------------------------------------------------------------------------

_failures: list[str] = []


def check(label: str, ok: bool, detail: str) -> None:
    mark = "PASS" if ok else "FAIL"
    print(f"  [{mark}] {label}: {detail}")
    if not ok:
        _failures.append(f"{label}: {detail}")


def main() -> int:
    car = Car()
    config = car.config

    print("=" * 74)
    print("TeleKart firmware smoke test -- MockBackend + FakeClock, virtual time")
    print("=" * 74)
    print(f"  backend            {type(car.gpio).__name__} (strict PWM pairing)")
    print(f"  control period     {CONTROL_PERIOD_S * 1000:.1f} ms")
    print(f"  max_rpm_measured   {car.calibration.max_rpm_measured:.1f} RPM")
    print(f"  v_max              {car.calibration.max_speed_mps(config.wheel_diameter_m):.3f} m/s")
    print(f"  control_timeout_s  {config.control_timeout_s:.3f} s")

    # -- arm ---------------------------------------------------------------
    print("\n[1] Arming")
    accepted, reason = car.arm()
    check("arm accepted", accepted, reason or "all four preconditions met")
    check(
        "state is ARMED",
        car.safety.state is VehicleState.ARMED,
        car.safety.state.name,
    )
    if not accepted:
        print("\nCannot continue: the car never armed.")
        return 1

    # -- throttle step ------------------------------------------------------
    throttle = 0.70
    settle_s = 3.0
    print(f"\n[2] Throttle step to {throttle:.2f}, {settle_s:.1f} s of virtual time")
    state = car.run(settle_s, command={"throttle": throttle})

    target = (state.rpm_target_l + state.rpm_target_r) / 2.0
    measured = (state.rpm_l + state.rpm_r) / 2.0
    plant_l, plant_r = car.plant_rpm
    duty_l, duty_r = car.duty
    error = measured - target
    error_pct = (error / target * 100.0) if target > 0.0 else float("inf")

    print(f"      target      {target:8.2f} RPM  (axle, from the controller)")
    print(f"      measured    {measured:8.2f} RPM  (encoder estimate)")
    print(f"      plant truth {plant_l:8.2f} / {plant_r:.2f} RPM  (left / right)")
    print(f"      duty        {duty_l:8.3f} / {duty_r:.3f}       (ENA / ENB)")
    print(f"      speed       {state.speed:8.3f} m/s   v_max {state.v_max:.3f} m/s")
    print(f"      error       {error:+8.2f} RPM  ({error_pct:+.1f} %)")

    check("target scales off measured max_rpm", target > 0.0,
          f"{target:.1f} RPM from max_rpm_measured={car.calibration.max_rpm_measured:.0f}")
    check("RPM converged within 10 % of target", abs(error_pct) <= 10.0,
          f"{error_pct:+.1f} % error")
    check("both wheels turning", plant_l > 5.0 and plant_r > 5.0,
          f"{plant_l:.1f} / {plant_r:.1f} RPM")
    check("bridge enabled while armed", duty_l > 0.0 and duty_r > 0.0,
          f"ENA={duty_l:.3f} ENB={duty_r:.3f}")
    check("combined duty inside the budget",
          duty_l + duty_r <= config.duty_sum_max + 1e-9,
          f"{duty_l + duty_r:.3f} <= {config.duty_sum_max:.3f}")

    # Convergence has to be *held*, not merely touched on the way past.
    peak_error = 0.0
    for _ in range(100):
        s = car.run(CONTROL_PERIOD_S, command={"throttle": throttle})
        e = abs((s.rpm_l + s.rpm_r) / 2.0 - target)
        peak_error = max(peak_error, e)
    check("stays converged over a further 1.0 s",
          target > 0.0 and peak_error / target <= 0.10,
          f"peak excursion {peak_error:.2f} RPM ({peak_error / target * 100:.1f} %)")

    # The feedback path has to be carrying this, not a feedforward table that
    # happens to be right. Two independent pieces of evidence.
    edges_l = car.encoder_l.total_edges
    edges_r = car.encoder_r.total_edges
    print(f"      encoder edges {edges_l} / {edges_r} (left / right)")
    check("encoders are actually counting", edges_l > 0 and edges_r > 0,
          f"{edges_l} / {edges_r} edges")

    ff_only = car.calibration.feedforward("left", target)
    print(f"      feedforward alone would be {ff_only:.3f} duty; "
          f"applied is {duty_l:.3f} -- the PID is carrying {duty_l - ff_only:+.3f}")

    # -- closed-loop disturbance rejection ----------------------------------
    # The real proof the loop is closed: add drag the feedforward table cannot
    # know about and watch the regulator find the duty that holds the speed.
    print("\n[3] Rail sag disturbance -- does the loop reject it?")
    before_duty = car.duty[0]
    before_rail = car.gpio.rail_voltage
    # Below the converter's knee the rail can no longer hold 9 V, so the same
    # duty now buys less speed. The feedforward table has no way to know this;
    # only the encoders can see it, so recovery is proof the loop is closed.
    car.gpio.set_pack_voltage(5.4)
    sag = car.run(0.30, command={"throttle": throttle})
    sag_rpm = (sag.rpm_l + sag.rpm_r) / 2.0
    recovered = car.run(3.0, command={"throttle": throttle})
    rec_rpm = (recovered.rpm_l + recovered.rpm_r) / 2.0
    after_duty = car.duty[0]
    rec_err_pct = (rec_rpm - target) / target * 100.0 if target > 0.0 else float("inf")

    print(f"      rail               {before_rail:.2f} V -> {car.gpio.rail_voltage:.2f} V")
    print(f"      immediately after  {sag_rpm:7.2f} RPM  (dipped {sag_rpm - target:+.2f})")
    print(f"      3 s later          {rec_rpm:7.2f} RPM  ({rec_err_pct:+.1f} % of target)")
    print(f"      duty               {before_duty:.3f} -> {after_duty:.3f} "
          f"({after_duty - before_duty:+.3f})")
    check("the disturbance actually bit", sag_rpm < target - 0.5,
          f"dipped to {sag_rpm:.2f} RPM from {target:.2f}")
    check("duty rose to reject it", after_duty > before_duty + 1e-3,
          f"{before_duty:.3f} -> {after_duty:.3f}")
    check("RPM recovered to target", abs(rec_err_pct) <= 10.0,
          f"{rec_err_pct:+.1f} % error after the disturbance")

    # Put it back so the failsafe stage sees the drivetrain it was tuned for.
    car.gpio.set_pack_voltage(car.gpio.plant.params.pack_nominal_v)
    car.run(1.0, command={"throttle": throttle})

    # -- failsafe ladder ----------------------------------------------------
    print("\n[4] Link goes stale -- failsafe ladder")
    print(f"      schedule: brake@{FAILSAFE_BRAKE_AT_MS} ms  "
          f"coast@{FAILSAFE_COAST_AT_MS} ms  disarm@{FAILSAFE_DISARM_AT_MS} ms "
          f"(measured from stale, i.e. {config.control_timeout_s * 1000:.0f} ms after "
          f"the last packet)")

    entry_rpm = (car.last_state.rpm_l + car.last_state.rpm_r) / 2.0
    stale_at = car.clock.monotonic()
    timeout_s = config.control_timeout_s

    # Sampled at the midpoint of each rung, so a one-tick boundary error cannot
    # make a wrong implementation look right.
    probes = [
        ("coast (pre-brake)", FAILSAFE_BRAKE_AT_MS * 0.4, "coast"),
        ("brake", (FAILSAFE_BRAKE_AT_MS + FAILSAFE_COAST_AT_MS) / 2.0, "brake"),
        ("coast (post-brake)", (FAILSAFE_COAST_AT_MS + FAILSAFE_DISARM_AT_MS) / 2.0, "coast"),
    ]

    seen_failsafe = False
    for label, at_ms, expected in probes:
        deadline = stale_at + timeout_s + at_ms / 1000.0
        while car.clock.monotonic() < deadline - 1e-9:
            car.step()  # no submit(): the link is dead
        st = car.safety.state
        dl, dr = car.duty
        braking = car.motors.braking
        since_ms = (car.clock.monotonic() - stale_at - timeout_s) * 1000.0
        seen_failsafe = seen_failsafe or st is VehicleState.FAILSAFE
        actual = "brake" if braking else ("coast" if dl == 0.0 and dr == 0.0 else "drive")
        print(f"      t+{since_ms:6.1f} ms  state={st.name:8s} "
              f"ENA={dl:.3f} ENB={dr:.3f} braking={braking!s:5s} -> {actual}")
        check(f"failsafe rung at {at_ms:.0f} ms is {expected}",
              actual == expected, f"got {actual}")
        check(f"state is FAILSAFE at {at_ms:.0f} ms",
              st is VehicleState.FAILSAFE, st.name)

    # Past the disarm deadline.
    deadline = stale_at + timeout_s + (FAILSAFE_DISARM_AT_MS + 100) / 1000.0
    while car.clock.monotonic() < deadline - 1e-9:
        car.step()
    st = car.safety.state
    dl, dr = car.duty
    since_ms = (car.clock.monotonic() - stale_at - timeout_s) * 1000.0
    print(f"      t+{since_ms:6.1f} ms  state={st.name:8s} ENA={dl:.3f} ENB={dr:.3f}")

    check("entered FAILSAFE at all", seen_failsafe, "observed on the ladder")
    check("disarmed past the deadline",
          st is not VehicleState.ARMED and st is not VehicleState.FAILSAFE, st.name)
    check("bridge is safe after disarm", car.gpio.outputs_safe,
          f"ENA={dl:.3f} ENB={dr:.3f}")

    final_rpm = car.gpio.wheel_rpm("left")
    print(f"      wheel spun down from {entry_rpm:.1f} RPM to {final_rpm:.1f} RPM")
    check("the car actually stopped", final_rpm < 0.25 * max(entry_rpm, 1.0),
          f"{final_rpm:.1f} RPM from {entry_rpm:.1f} RPM")

    # -- summary ------------------------------------------------------------
    print("\n" + "=" * 74)
    if _failures:
        print(f"FAILED -- {len(_failures)} check(s):")
        for failure in _failures:
            print(f"  - {failure}")
        return 1
    print("All checks passed.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
