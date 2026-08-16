"""Fixtures for the firmware suite: a whole car, in virtual time, with no hardware.

Two decisions shape everything here.

**Every fixture is built on ``FakeClock`` + ``MockBackend``.** A sixty-second
drive test therefore runs in milliseconds and returns the same numbers on every
machine. Nothing in this suite sleeps, and nothing in it needs a Pi.

**The firmware modules are imported inside the fixtures, not at module scope.**
A ``conftest.py`` that fails to import takes the *entire* session down with a
collection error, including the tests that had no dependency on the missing
module. Importing late means a half-built tree still runs everything that is
finished, and the tests that genuinely need the missing piece are the only ones
that fail.
"""

from __future__ import annotations

import dataclasses
import sys
from pathlib import Path
from typing import Any, Iterator

import pytest

# --------------------------------------------------------------------------
# Import path
# --------------------------------------------------------------------------
# `make setup-pi` installs both packages into pi/.venv and this block does
# nothing. It exists so that a fresh checkout can run `pytest pi/tests` with a
# system interpreter: the shared protocol package lives in a sibling directory
# that no amount of rootdir inference will find.

_PI_ROOT = Path(__file__).resolve().parents[1]
_REPO_ROOT = _PI_ROOT.parent
_PROTOCOL_ROOT = _REPO_ROOT / "packages" / "telekart_protocol"

for _candidate in (_PI_ROOT, _PROTOCOL_ROOT):
    _text = str(_candidate)
    if _candidate.is_dir() and _text not in sys.path:
        sys.path.insert(0, _text)

from telekart.config import VehicleConfig  # noqa: E402
from telekart.constants import CONTROL_PERIOD_S  # noqa: E402
from telekart.hal.mock_backend import MockBackend, MockPins, PlantParams  # noqa: E402
from telekart.util.clock import FakeClock  # noqa: E402

from telekart_protocol import ControlFlags, VehicleState  # noqa: E402

#: What the mock drivetrain actually reaches at ``max_duty`` on one wheel, and
#: the duty at which it breaks away. Both fall out of the plant model rather
#: than being chosen: 0.85 duty on a 9 V rail through a 1.4 V Darlington drop
#: gives ~139 RPM, and the stiction plus that drop gives ~0.217 of deadband.
#: The synthetic calibration below has to agree with them or every closed-loop
#: test is measuring a feedforward table that does not describe its plant.
MOCK_PLANT_MAX_RPM = 139.0
MOCK_PLANT_DEADBAND = 0.22

WHEEL_KEYS: tuple[str, ...] = ("left_fwd", "left_rev", "right_fwd", "right_rev")


# --------------------------------------------------------------------------
# Primitives
# --------------------------------------------------------------------------


@pytest.fixture
def clock() -> FakeClock:
    """Virtual time, starting at a non-zero value.

    Non-zero on purpose: a clock that starts at 0.0 makes an uninitialised
    ``last_seen = 0.0`` look like "seen just now", and that bug then hides until
    the car has been up for a while. Starting at 1000 s exposes it immediately.
    """
    return FakeClock(start=1000.0)


@pytest.fixture
def config() -> VehicleConfig:
    """Registry defaults and the as-wired pin map. No YAML, no filesystem."""
    return VehicleConfig()


@pytest.fixture
def plant_params(config: VehicleConfig) -> PlantParams:
    return PlantParams(cpr=config.encoder_cpr)


@pytest.fixture
def gpio(
    clock: FakeClock, config: VehicleConfig, plant_params: PlantParams
) -> Iterator[MockBackend]:
    backend = MockBackend(
        pins=MockPins.from_hardware_pins(config.pins),
        params=plant_params,
        clock=clock,
        seed=1,
        strict=True,
    )
    # The plant counts encoder phase in the same units the driver decodes, so a
    # test that changes encoder_cpr has to move both or the RPM it measures is
    # not the RPM the plant is turning.
    backend.plant.cpr = config.encoder_cpr
    try:
        yield backend
    finally:
        backend.cleanup()


@pytest.fixture
def plant(gpio: MockBackend) -> Any:
    return gpio.plant


# --------------------------------------------------------------------------
# Drivers
# --------------------------------------------------------------------------


@pytest.fixture
def motors(gpio: MockBackend, config: VehicleConfig, clock: FakeClock) -> Any:
    from telekart.drivers.motor import MotorPair

    return MotorPair(gpio, config.pins.motors, config, clock)


@pytest.fixture
def servo(gpio: MockBackend, config: VehicleConfig, clock: FakeClock) -> Any:
    from telekart.drivers.servo import SteeringServo

    return SteeringServo(gpio, config.pins.servo, config, clock)


@pytest.fixture
def encoder_l(gpio: MockBackend, config: VehicleConfig) -> Any:
    from telekart.constants import ENCODER_GLITCH_US
    from telekart.drivers.encoder import QuadratureEncoder

    return QuadratureEncoder(
        gpio,
        config.pins.encoders.left_a,
        config.pins.encoders.left_b,
        cpr=config.encoder_cpr,
        invert=config.encoder_invert_left,
        glitch_us=ENCODER_GLITCH_US,
    )


@pytest.fixture
def encoder_r(gpio: MockBackend, config: VehicleConfig) -> Any:
    from telekart.constants import ENCODER_GLITCH_US
    from telekart.drivers.encoder import QuadratureEncoder

    return QuadratureEncoder(
        gpio,
        config.pins.encoders.right_a,
        config.pins.encoders.right_b,
        cpr=config.encoder_cpr,
        invert=config.encoder_invert_right,
        glitch_us=ENCODER_GLITCH_US,
    )


# --------------------------------------------------------------------------
# Control
# --------------------------------------------------------------------------


@pytest.fixture
def mixer(config: VehicleConfig) -> Any:
    from telekart.control.mixer import DifferentialMixer

    return DifferentialMixer(config)


@pytest.fixture
def safety(config: VehicleConfig, clock: FakeClock) -> Any:
    from telekart.control.safety import SafetyStateMachine

    return SafetyStateMachine(config, clock)


@pytest.fixture
def odometry(config: VehicleConfig) -> Any:
    from telekart.odometry import BicycleOdometry

    return BicycleOdometry(config)


@pytest.fixture
def calibration() -> Any:
    """A calibration that describes the mock plant.

    The feedforward table is expressed **relative to the deadband**, because the
    pipeline applies deadband compensation after ``feedforward + PID``. A table
    that baked the deadband in as well would be counted twice and produce a step
    at the bottom of the throttle that no PID gain can remove.
    """
    return synthetic_calibration()


def synthetic_calibration(
    max_rpm: float = MOCK_PLANT_MAX_RPM,
    deadband: float = MOCK_PLANT_DEADBAND,
    *,
    on_ground: bool = False,
) -> Any:
    from telekart.calibration import DriveCalibration

    span = max(1.0, max_rpm)
    # 0.63 of duty above the deadband is what carries the mock plant from rest
    # to its ceiling, so the slope is that divided by the RPM span.
    slope = 0.63 / span
    lut = [(round(f * span, 3), round(f * span * slope, 5)) for f in (0.0, 0.2, 0.4, 0.6, 0.8, 1.0)]
    return DriveCalibration(
        max_rpm={key: max_rpm for key in WHEEL_KEYS},
        deadband={key: deadband for key in WHEEL_KEYS},
        ff_lut={key: list(lut) for key in WHEEL_KEYS},
        measured_at="2026-01-01T00:00:00Z",
        on_ground=on_ground,
    )


# --------------------------------------------------------------------------
# The whole car
# --------------------------------------------------------------------------


class Vehicle:
    """Everything assembled, plus the loop that drives it in virtual time.

    ``step`` runs the three operations in the order the real system does them:
    time passes, the plant integrates the commands issued on the previous tick
    and emits the encoder edges that motion produced, and only then does the
    controller read those edges and issue new commands. Ticking before stepping
    would let a command take effect in the same instant it was written, which
    hides exactly the one-tick delay that closed-loop stability depends on.
    """

    __slots__ = (
        "clock", "gpio", "config", "motors", "servo", "encoder_l", "encoder_r",
        "safety", "odometry", "calibration", "controller", "ticks", "last_state",
    )

    def __init__(self, **parts: Any) -> None:
        for name in self.__slots__:
            setattr(self, name, parts.get(name))
        self.ticks = 0
        self.last_state = None

    # -- driving ------------------------------------------------------------

    def submit(
        self,
        *,
        steering: float = 0.0,
        throttle: float = 0.0,
        brake: float = 0.0,
        flags: ControlFlags = ControlFlags.NONE,
    ) -> None:
        """Hand the controller a command, as the UDP receiver would.

        ``note_control_packet`` is called here as well as (presumably) inside
        the controller. It only resets a staleness timer, so calling it twice
        costs nothing, and it keeps this fixture from depending on which of the
        two layers owns that call.
        """
        from telekart.control.drive import DriveCommand

        self.controller.submit_command(
            DriveCommand(
                steering=steering,
                throttle=throttle,
                brake=brake,
                flags=flags,
                received_at=self.clock.monotonic(),
            )
        )
        self.safety.note_control_packet()

    def step(self, dt: float = CONTROL_PERIOD_S) -> Any:
        self.clock.advance(dt)
        self.gpio.step(dt)
        self.last_state = self.controller.tick(dt)
        self.ticks += 1
        return self.last_state

    def run(
        self,
        seconds: float,
        *,
        dt: float = CONTROL_PERIOD_S,
        command: dict[str, Any] | None = None,
    ) -> Any:
        """Step for ``seconds``, re-submitting ``command`` on every tick.

        Re-submitting is what a live link does at 100 Hz; omitting ``command``
        is how a test lets the link go stale and watches the failsafe run.
        """
        remaining = seconds
        while remaining > 1e-12:
            slice_s = dt if remaining >= dt else remaining
            if command is not None:
                self.submit(**command)
            self.step(slice_s)
            remaining -= slice_s
        return self.last_state

    def arm(self, *, settle: float = 0.25) -> tuple[bool, str]:
        """Satisfy all four arming preconditions, then ask.

        The session is asserted here because on the real car it is the TCP
        session layer that owns that flag, and no part of the control loop can
        set it for itself. Neutral throttle then has to have been held for
        ``arm_neutral_ms`` *before* the request, which is the whole point of
        that parameter -- so this holds neutral first and asks afterwards.
        """
        self.safety.set_session_valid(True)
        self.run(self.config.arm_neutral_s + settle, command={"throttle": 0.0})
        accepted, reason = self.safety.request_arm()
        if accepted:
            self.run(2 * CONTROL_PERIOD_S, command={"throttle": 0.0})
        return accepted, reason

    # -- views --------------------------------------------------------------

    @property
    def state(self) -> VehicleState:
        return self.safety.state

    @property
    def rpm(self) -> tuple[float, float]:
        """Plant truth, not the estimate -- for asserting that the wheels really
        turned rather than that the filter says they did."""
        return (self.gpio.wheel_rpm("left"), self.gpio.wheel_rpm("right"))

    @property
    def duty(self) -> tuple[float, float]:
        pins = self.config.pins.motors
        return (self.gpio.pwm_duty(pins.ena), self.gpio.pwm_duty(pins.enb))


@pytest.fixture
def vehicle(
    clock: FakeClock,
    gpio: MockBackend,
    config: VehicleConfig,
    motors: Any,
    servo: Any,
    encoder_l: Any,
    encoder_r: Any,
    safety: Any,
    odometry: Any,
    calibration: Any,
) -> Vehicle:
    from telekart.control.drive import DriveController

    controller = DriveController(
        gpio=gpio,
        config=config,
        clock=clock,
        motors=motors,
        servo=servo,
        encoder_l=encoder_l,
        encoder_r=encoder_r,
        safety=safety,
        odometry=odometry,
        calibration=calibration,
    )
    return Vehicle(
        clock=clock,
        gpio=gpio,
        config=config,
        motors=motors,
        servo=servo,
        encoder_l=encoder_l,
        encoder_r=encoder_r,
        safety=safety,
        odometry=odometry,
        calibration=calibration,
        controller=controller,
    )


# --------------------------------------------------------------------------
# Feedback snapshots
# --------------------------------------------------------------------------

#: Names this suite uses, mapped to the spellings a snapshot might plausibly
#: have chosen. The safety tests assert *behaviour*; the exact field set of
#: FeedbackSnapshot belongs to control/safety.py and is not this file's to fix.
_FEEDBACK_ALIASES: dict[str, tuple[str, ...]] = {
    "rpm_l": ("rpm_l", "rpm_left", "left_rpm"),
    "rpm_r": ("rpm_r", "rpm_right", "right_rpm"),
    "duty_l": ("duty_l", "duty_left", "left_duty"),
    "duty_r": ("duty_r", "duty_right", "right_duty"),
    "edges_l": ("total_edges_l", "edges_l", "edges_left"),
    "edges_r": ("total_edges_r", "edges_r", "edges_right"),
    "stale_l": ("stale_l", "encoder_stale_l", "left_stale"),
    "stale_r": ("stale_r", "encoder_stale_r", "right_stale"),
    "braking": ("braking",),
    "pack_v": ("pack_volts", "pack_v", "pack_voltage", "battery_v"),
    "temp_c": ("cpu_temp_c", "temp_c", "temperature_c"),
    "throttled": ("throttled",),
    "gpio_error": ("gpio_error",),
}

#: Keyed by the *string* spelling of the annotation, because every module in
#: this codebase uses `from __future__ import annotations` and dataclass field
#: types therefore arrive here unevaluated.
_ZERO_BY_TYPE: dict[str, Any] = {
    "float": 0.0,
    "int": 0,
    "bool": False,
    "str": "",
    "Fault": 0,
}


@pytest.fixture
def feedback() -> Any:
    """The snapshot factory, handed over as a fixture.

    A fixture rather than something the test modules import from here: two
    packages in this repository are called ``tests``, so a session that collects
    both resolves ``from .conftest import ...`` to whichever one was imported
    first. Fixtures are injected by pytest and never go through that lookup.
    """
    return make_feedback


def make_feedback(**values: Any) -> Any:
    """Build a ``FeedbackSnapshot`` from whichever fields it actually declares.

    Written reflectively rather than positionally because the snapshot's layout
    is owned by ``control/safety.py`` and is being written in parallel with this
    file. A test that hard-coded the constructor would fail for a reason that
    has nothing to do with the behaviour under test; this fails only if a field
    this suite genuinely depends on is absent.
    """
    from telekart.control.safety import FeedbackSnapshot

    fields = {f.name: f for f in dataclasses.fields(FeedbackSnapshot)}
    kwargs: dict[str, Any] = {}
    for canonical, spellings in _FEEDBACK_ALIASES.items():
        if canonical not in values:
            continue
        for name in spellings:
            if name in fields:
                kwargs[name] = values[canonical]
                break
    for name, field in fields.items():
        if name in kwargs:
            continue
        if field.default is not dataclasses.MISSING:
            continue
        if field.default_factory is not dataclasses.MISSING:  # type: ignore[misc]
            continue
        kwargs[name] = _ZERO_BY_TYPE.get(str(field.type).strip("'\" "), 0.0)
    return FeedbackSnapshot(**kwargs)


# --------------------------------------------------------------------------
# Shared assertions
# --------------------------------------------------------------------------


def assert_outputs_safe(gpio: MockBackend) -> None:
    """Neither bridge can drive. The only end state a panic path may leave."""
    assert gpio.outputs_safe, (
        f"bridge still enabled: ENA={gpio.pwm_duty(gpio.pins.ena):.3f} "
        f"ENB={gpio.pwm_duty(gpio.pins.enb):.3f}"
    )
