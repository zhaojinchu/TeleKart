"""The panic-stop chain -- layer 1, the only layer that lives in this process.

``pigpiod`` retains GPIO state after its client dies, so a crash at 80 % duty
leaves the motors running indefinitely. This module tests the in-process half of
the four-layer answer: that every hook is actually installed, that firing is
idempotent and cannot raise, and that the outputs end up safe on *every* path out
of ``main()`` -- exception, signal, or a plain ``return``.

Two properties here are easy to regress silently and are asserted mechanically
rather than by eye:

* **Allocation-freeness.** The panic path runs from a signal handler and from
  ``sys.excepthook``, which is precisely where a ``MemoryError`` lands. A tuple
  built from attribute loads is a heap allocation that no reviewer will notice,
  so the bytecode is inspected directly.
* **Ordering.** Enables go to zero before the direction pins move. Clearing the
  direction pins under a live enable walks the outputs through ``IN1 == IN2``,
  which on an L298 is a brake -- a hard short across a spinning motor at the
  exact moment things were meant to be calming down.
"""

from __future__ import annotations

import atexit
import dis
import signal
import sys
import threading

import pytest

from telekart.app import HardwarePanic, PanicChain
from telekart.config import VehicleConfig
from telekart.drivers.motor import MotorPair
from telekart.hal.mock_backend import MockBackend, MockPins, PlantParams
from telekart.util.clock import FakeClock


def build() -> tuple[MockBackend, VehicleConfig, HardwarePanic, MotorPair, FakeClock]:
    clock = FakeClock(start=1000.0)
    config = VehicleConfig()
    gpio = MockBackend(
        pins=MockPins.from_hardware_pins(config.pins),
        params=PlantParams(cpr=config.encoder_cpr),
        clock=clock,
        strict=True,
    )
    motors = MotorPair(gpio, config.pins.motors, config, clock)
    panic = HardwarePanic(gpio, config.pins, config.pwm_hz)
    return gpio, config, panic, motors, clock


def spin_up(motors: MotorPair, clock: FakeClock, gpio: MockBackend) -> None:
    """Get the bridge genuinely live, so 'safe' afterwards means something."""
    for _ in range(60):
        motors.drive(0.8, 0.8)
        clock.advance(0.005)
        gpio.step(0.005)
    assert gpio.pwm_duty(gpio.pins.ena) > 0.0, "the bridge never came up"


# --------------------------------------------------------------------------
# HardwarePanic
# --------------------------------------------------------------------------


def test_panic_safes_the_bridge_and_is_idempotent() -> None:
    gpio, _config, panic, motors, clock = build()
    panic.attach(motors)
    spin_up(motors, clock, gpio)

    panic.stop(announce=False)
    assert gpio.outputs_safe
    panic.stop(announce=False)
    panic.stop(announce=False)
    assert gpio.outputs_safe
    assert panic.fired == 3
    assert panic.failures == 0


def test_panic_works_before_any_driver_is_attached() -> None:
    """A failure part way through construction still has to leave the bridge
    coasting -- which is the whole reason the raw pin path exists alongside the
    MotorPair one."""
    gpio, config, panic, motors, clock = build()
    spin_up(motors, clock, gpio)  # something drove the pins; panic never saw it

    panic.stop(announce=False)  # no attach() at all
    assert gpio.outputs_safe
    for pin in (config.pins.motors.in1, config.pins.motors.in2,
                config.pins.motors.in3, config.pins.motors.in4):
        assert gpio.pin_level(pin) is False


def test_panic_never_raises_after_the_backend_is_gone() -> None:
    """It is wired to atexit, so it runs after cleanup at least once in the life
    of every process. It may not raise there."""
    gpio, _config, panic, motors, clock = build()
    panic.attach(motors)
    spin_up(motors, clock, gpio)
    gpio.cleanup()

    panic.stop(announce=False)  # must not raise
    assert gpio.outputs_safe


def test_panic_drops_the_enables_before_the_direction_pins() -> None:
    """Order matters. Clearing IN under a live enable is a brake on an L298."""
    gpio, config, panic, motors, clock = build()
    panic.attach(motors)
    spin_up(motors, clock, gpio)

    before = len(gpio.ops())
    panic.stop(announce=False)
    ops = gpio.ops()[before:]

    pins = config.pins.motors
    ins = {pins.in1, pins.in2, pins.in3, pins.in4}
    first_in = next(
        (i for i, op in enumerate(ops) if op.kind == "write" and op.pin in ins), None
    )
    first_enable_zero = next(
        (
            i
            for i, op in enumerate(ops)
            if op.kind == "pwm" and op.pin in (pins.ena, pins.enb) and op.value == 0.0
        ),
        None,
    )
    assert first_enable_zero is not None, "the enables were never taken to zero"
    # Not `if first_in is not None`: spin_up leaves IN1/IN3 high and the panic
    # path writes the direction pins undeduplicated, so they *must* appear. A
    # missing write means the assertion below stopped testing anything.
    assert first_in is not None, "the direction pins were never cleared at all"
    assert first_enable_zero < first_in, (
        "the direction pins moved before the bridge was disabled, which "
        "walks a spinning motor through IN1 == IN2 -- a brake, not a coast"
    )


@pytest.mark.parametrize(
    "func, name",
    [(HardwarePanic.stop, "HardwarePanic.stop"), (MotorPair.panic_stop, "MotorPair.panic_stop")],
)
def test_the_panic_path_allocates_nothing(func: object, name: str) -> None:
    """Runs from a signal handler and from sys.excepthook -- exactly where a
    MemoryError lands. Spelling a tuple of attribute loads inline would build it
    on the heap every call, and nothing but the bytecode will tell you."""
    allocating = [
        instruction.opname
        for instruction in dis.get_instructions(func)  # type: ignore[arg-type]
        if instruction.opname.startswith(("BUILD_", "FORMAT_"))
        or instruction.opname == "LIST_EXTEND"
    ]
    assert not allocating, f"{name} allocates: {allocating}"


# --------------------------------------------------------------------------
# PanicChain
# --------------------------------------------------------------------------


def test_the_chain_installs_and_removes_every_hook() -> None:
    gpio, _config, panic, _motors, _clock = build()

    sys_before = sys.excepthook
    thread_before = threading.excepthook
    signals = [
        getattr(signal, n) for n in ("SIGTERM", "SIGINT", "SIGHUP")
        if getattr(signal, n, None) is not None
    ]
    assert signals, "no supported signals on this platform"
    handlers_before = {s: signal.getsignal(s) for s in signals}

    chain = PanicChain(panic)
    chain.install()
    try:
        assert sys.excepthook is not sys_before
        assert threading.excepthook is not thread_before
        for s in signals:
            assert signal.getsignal(s) is not handlers_before[s], f"{s!r} not hooked"
    finally:
        chain.remove()

    assert sys.excepthook is sys_before
    assert threading.excepthook is thread_before
    for s in signals:
        assert signal.getsignal(s) is handlers_before[s]


def test_the_chain_registers_and_unregisters_with_atexit() -> None:
    """A chain that left itself registered would fire a panic stop against a
    dead backend at interpreter shutdown, in every process that ever built one."""
    gpio, _config, panic, _motors, _clock = build()
    chain = PanicChain(panic)
    chain.install()
    chain.remove()

    # atexit.unregister is the only observable here; if the bound method were
    # not the same object identity-wise this would silently no-op, so prove it
    # by firing the exit callbacks and checking nothing ran.
    fired_before = panic.fired
    atexit._run_exitfuncs()  # type: ignore[attr-defined]
    assert panic.fired == fired_before, "the chain is still registered with atexit"


def test_the_context_manager_safes_hardware_on_a_plain_return() -> None:
    gpio, _config, panic, motors, clock = build()
    panic.attach(motors)

    with PanicChain(panic):
        spin_up(motors, clock, gpio)
        assert not gpio.outputs_safe  # genuinely driving inside the block

    assert gpio.outputs_safe
    assert panic.fired >= 1


def test_the_context_manager_safes_hardware_on_an_exception() -> None:
    gpio, _config, panic, motors, clock = build()
    panic.attach(motors)

    with pytest.raises(RuntimeError):
        with PanicChain(panic):
            spin_up(motors, clock, gpio)
            raise RuntimeError("boom")

    assert gpio.outputs_safe


def test_a_signal_fires_the_panic_and_forwards_to_the_shutdown_hook() -> None:
    gpio, _config, panic, motors, clock = build()
    panic.attach(motors)
    seen: list[int] = []

    chain = PanicChain(panic, seen.append)
    chain.install()
    try:
        spin_up(motors, clock, gpio)
        chain._handle_signal(int(signal.SIGTERM), None)
    finally:
        chain.remove()

    assert gpio.outputs_safe
    assert seen == [int(signal.SIGTERM)], "the shutdown request was not forwarded"


def test_an_unhandled_exception_fires_the_panic() -> None:
    gpio, _config, panic, motors, clock = build()
    panic.attach(motors)

    chain = PanicChain(panic)
    chain.install()
    try:
        spin_up(motors, clock, gpio)
        try:
            raise RuntimeError("unhandled")
        except RuntimeError:
            sys.excepthook(*sys.exc_info())  # type: ignore[arg-type]
    finally:
        chain.remove()

    assert gpio.outputs_safe


# --------------------------------------------------------------------------
# The bring-up scripts' shape: PanicChain(panic), with no on_signal callback
# --------------------------------------------------------------------------
#
# Every script in pi/scripts/ builds the chain WITHOUT a shutdown callback:
#
#     with PanicChain(panic):
#         try:
#             ...drive the hardware...
#         except KeyboardInterrupt:
#             ...
#         finally:
#             gpio.cleanup()
#
# Installing a SIGINT handler also removes the KeyboardInterrupt the default
# handler would have raised. If the no-callback path returns normally, both the
# `except KeyboardInterrupt` and the `finally` above are dead code, the operator's
# Ctrl-C is swallowed, and the loop commands duty again on its very next
# iteration -- having just been told to stop.


def test_ctrl_c_with_no_callback_raises_keyboardinterrupt() -> None:
    """The scripts' `except KeyboardInterrupt` must be reachable."""
    gpio, _config, panic, motors, clock = build()
    panic.attach(motors)

    chain = PanicChain(panic)  # no on_signal -- exactly what pi/scripts/ does
    chain.install()
    try:
        spin_up(motors, clock, gpio)
        with pytest.raises(KeyboardInterrupt):
            chain._handle_signal(int(signal.SIGINT), None)
    finally:
        chain.remove()

    assert gpio.outputs_safe, "the panic stop must still have run"


def test_sigterm_with_no_callback_raises_systemexit() -> None:
    gpio, _config, panic, motors, clock = build()
    panic.attach(motors)

    chain = PanicChain(panic)
    chain.install()
    try:
        spin_up(motors, clock, gpio)
        with pytest.raises(SystemExit) as caught:
            chain._handle_signal(int(signal.SIGTERM), None)
    finally:
        chain.remove()

    assert caught.value.code == 128 + int(signal.SIGTERM)
    assert gpio.outputs_safe


def test_ctrl_c_stops_a_script_shaped_drive_loop() -> None:
    """The end-to-end property, in the shape the scripts are actually written.

    Without the raise, `interrupted` stays False, the loop runs to completion and
    keeps calling drive() after the operator asked it to stop. The motor driver's
    panic latch means the duty stays at zero either way -- but a script that
    ignores Ctrl-C is a script the operator will reach for the master switch to
    stop, and any later clear_panic() or direct GPIO write would put it live
    again. servo_calibrate.py writes the servo pin directly, so for that one the
    latch does not help at all.
    """
    gpio, _config, panic, motors, clock = build()
    panic.attach(motors)

    interrupted = False
    cleaned_up = False
    ticks_after_signal = 0

    chain = PanicChain(panic)
    chain.install()
    try:
        try:
            for i in range(40):
                motors.drive(0.5, 0.5)
                clock.advance(0.010)
                gpio.step(0.010)
                if i == 20:
                    chain._handle_signal(int(signal.SIGINT), None)
                elif i > 20:
                    ticks_after_signal += 1
        except KeyboardInterrupt:
            interrupted = True
        finally:
            cleaned_up = True
    finally:
        chain.remove()

    assert interrupted, "Ctrl-C did not reach the script's KeyboardInterrupt handler"
    assert cleaned_up, "the script's finally: never ran"
    assert ticks_after_signal == 0, "the loop kept commanding duty after Ctrl-C"
    assert gpio.outputs_safe
