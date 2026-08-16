#!/usr/bin/env python3
"""Bring-up phase 1: prove the pins before anything can move.

Run this with the **motor battery disconnected**. Every check below is designed
so that a total failure of the thing being tested moves nothing at all: the
enables are exercised at zero and 20 % duty into a bridge with no supply, and
the servo is swept 100 us either side of centre with the linkage off.

The three findings that matter, in the order they bite:

* **GPIO5 and GPIO6 idle HIGH.** The SoC pulls GPIO0-8 up at boot, so IN1 and
  IN2 come up high, and IN1 == IN2 with an enable up is a brake. External 10k
  pull-downs are the fix; software cannot solve it, which is why this script
  reads those pins *before* it claims them.
* **snd_bcm2835 owns both PWM channels.** If the onboard audio driver is
  loaded, hardware PWM is simply unavailable and the motors will never run.
  ``dtparam=audio=off`` in /boot/firmware/config.txt, then reboot.
* **pigpiod retains pin state after its client exits.** Nothing here leaves an
  output high, and the panic chain runs on every exit path including Ctrl-C.
"""

from __future__ import annotations

import argparse
import platform
import subprocess
import sys
from dataclasses import dataclass
from pathlib import Path

# Running from a checkout without `make setup-pi` is the normal case for a
# bench script, so make the source tree importable rather than requiring an
# install step before the hardware can be tested.
_PI_ROOT = Path(__file__).resolve().parents[1]
_PROTOCOL_ROOT = _PI_ROOT.parent / "packages" / "telekart_protocol"
for _candidate in (_PI_ROOT, _PROTOCOL_ROOT):
    if _candidate.is_dir() and str(_candidate) not in sys.path:
        sys.path.insert(0, str(_candidate))

from telekart.app import HardwarePanic, PanicChain  # noqa: E402
from telekart.config import ConfigError, VehicleConfig  # noqa: E402
from telekart.constants import (  # noqa: E402
    ENCODER_GLITCH_US,
    LOCAL_CONFIG_PATH,
    SERVO_NOMINAL_CENTER_US,
)
from telekart.hal.base import Edge, GpioBackend, GpioError, Pull, select_backend  # noqa: E402
from telekart.util.clock import RealClock  # noqa: E402

PASS = "PASS"
FAIL = "FAIL"
WARN = "WARN"
SKIP = "SKIP"

#: Low enough to be inaudible in a bridge with no supply, high enough that a
#: scope or a meter on the enable pin sees something.
PROBE_DUTY = 0.20


@dataclass(slots=True)
class Check:
    name: str
    status: str
    detail: str = ""
    fix: str = ""


class Checklist:
    """Collects results so the operator gets one table instead of a scroll."""

    def __init__(self) -> None:
        self.checks: list[Check] = []

    def record(self, name: str, status: str, detail: str = "", fix: str = "") -> Check:
        check = Check(name, status, detail, fix)
        self.checks.append(check)
        marker = {PASS: "  ok  ", FAIL: " FAIL ", WARN: " warn ", SKIP: " skip "}[status]
        print(f"[{marker}] {name}" + (f"  --  {detail}" if detail else ""))
        if status == FAIL and fix:
            print(f"          fix: {fix}")
        return check

    def ok(self, name: str, detail: str = "") -> Check:
        return self.record(name, PASS, detail)

    def fail(self, name: str, detail: str, fix: str = "") -> Check:
        return self.record(name, FAIL, detail, fix)

    def warn(self, name: str, detail: str, fix: str = "") -> Check:
        return self.record(name, WARN, detail, fix)

    def skip(self, name: str, detail: str = "") -> Check:
        return self.record(name, SKIP, detail)

    @property
    def failures(self) -> list[Check]:
        return [check for check in self.checks if check.status == FAIL]

    def summary(self) -> int:
        print()
        print("=" * 68)
        counts = {status: 0 for status in (PASS, FAIL, WARN, SKIP)}
        for check in self.checks:
            counts[check.status] += 1
        print(
            f"  {counts[PASS]} passed   {counts[FAIL]} failed   "
            f"{counts[WARN]} warnings   {counts[SKIP]} skipped"
        )
        if self.failures:
            print()
            print("  Blocking problems:")
            for check in self.failures:
                print(f"    - {check.name}: {check.detail}")
                if check.fix:
                    print(f"      {check.fix}")
            print()
            print("  Do not proceed to phase 2 until these are clear.")
        else:
            print("  GPIO self-test passed. Next: scripts/servo_calibrate.py --sweep-safe")
        print("=" * 68)
        return 1 if self.failures else 0


# --------------------------------------------------------------------------
# Environment
# --------------------------------------------------------------------------


def read_text(path: str) -> str:
    try:
        return Path(path).read_text(encoding="utf-8", errors="replace")
    except OSError:
        return ""


def audio_driver_loaded() -> bool:
    """snd_bcm2835 claims BOTH hardware PWM channels the moment it loads."""
    return "snd_bcm2835" in read_text("/proc/modules")


def boot_config_text() -> str:
    # Bookworm moved the boot partition; check both so the advice printed below
    # names the file that actually exists on this machine.
    for path in ("/boot/firmware/config.txt", "/boot/config.txt"):
        text = read_text(path)
        if text:
            return text
    return ""


def pigpiod_running() -> bool:
    for entry in Path("/proc").iterdir():
        if not entry.name.isdigit():
            continue
        comm = read_text(f"/proc/{entry.name}/comm").strip()
        if comm == "pigpiod":
            return True
    return False


def vcgencmd_throttled() -> str | None:
    try:
        result = subprocess.run(
            ["vcgencmd", "get_throttled"],
            capture_output=True,
            text=True,
            timeout=5.0,
            check=False,
        )
    except (OSError, subprocess.SubprocessError):
        return None
    if result.returncode != 0:
        return None
    return result.stdout.strip()


def check_environment(report: Checklist, backend_name: str) -> None:
    machine = platform.machine()
    version = ".".join(str(part) for part in sys.version_info[:3])
    report.ok("interpreter", f"python {version} on {machine}")

    if backend_name == "mock" or not Path("/proc/modules").exists():
        report.skip("host is a Raspberry Pi", f"machine={machine}")
        return

    if audio_driver_loaded():
        report.fail(
            "snd_bcm2835 not loaded",
            "the onboard audio driver is loaded and owns both PWM channels",
            "add 'dtparam=audio=off' to /boot/firmware/config.txt and reboot",
        )
    else:
        report.ok("snd_bcm2835 not loaded", "both PWM channels are free")

    config_txt = boot_config_text()
    if config_txt and "dtparam=audio=off" not in config_txt:
        report.warn(
            "dtparam=audio=off in config.txt",
            "not present; audio is only unloaded until the next reboot",
            "add 'dtparam=audio=off' to /boot/firmware/config.txt",
        )
    elif config_txt:
        report.ok("dtparam=audio=off in config.txt")

    if pigpiod_running():
        report.ok("pigpiod running")
    else:
        report.fail(
            "pigpiod running",
            "no pigpiod process found",
            "sudo systemctl enable --now pigpiod",
        )

    throttled = vcgencmd_throttled()
    if throttled is None:
        report.skip("vcgencmd get_throttled", "vcgencmd not available")
    elif throttled.endswith("=0x0"):
        report.ok("vcgencmd get_throttled", throttled)
    else:
        report.fail(
            "vcgencmd get_throttled",
            f"{throttled}; the Pi's own supply has been out of spec",
            "fix the 5 V supply first -- every measurement below is suspect until it reads 0x0",
        )


# --------------------------------------------------------------------------
# Pins
# --------------------------------------------------------------------------


def check_boot_levels(report: Checklist, gpio: GpioBackend, config: VehicleConfig) -> None:
    """Read the motor pins before claiming them.

    This is the external pull-down check and it only works *before* anything is
    configured as an output -- afterwards the pin reads whatever we drove it to
    and the resistor could be missing entirely.
    """
    pins = config.pins.motors
    named = (
        ("ENA", pins.ena),
        ("IN1", pins.in1),
        ("IN2", pins.in2),
        ("IN3", pins.in3),
        ("IN4", pins.in4),
        ("ENB", pins.enb),
    )
    high = []
    levels = []
    for label, pin in named:
        level = gpio.read(pin)
        levels.append(f"{label}(GPIO{pin})={1 if level else 0}")
        if level:
            high.append(f"{label}(GPIO{pin})")

    detail = " ".join(levels)
    if not high:
        report.ok("motor pins idle low", detail)
        return
    report.fail(
        "motor pins idle low",
        f"{', '.join(high)} idle HIGH -- {detail}",
        "fit 10k pull-downs to ground on ENA/ENB/IN1-IN4. GPIO5 and GPIO6 are "
        "pulled UP by the SoC at boot, and IN1 == IN2 with an enable up is a "
        "brake, so this is a hardware fix and not a software one",
    )


def check_direction_pins(report: Checklist, gpio: GpioBackend, config: VehicleConfig) -> None:
    """Toggle each direction pin and confirm nothing else moves with it."""
    pins = config.pins.motors
    named = (
        ("IN1", pins.in1),
        ("IN2", pins.in2),
        ("IN3", pins.in3),
        ("IN4", pins.in4),
    )
    for _label, pin in named:
        gpio.setup_output(pin, False)

    problems: list[str] = []
    for label, pin in named:
        gpio.write(pin, True)
        if not gpio.read(pin):
            problems.append(f"{label}(GPIO{pin}) would not go high")
        for other_label, other_pin in named:
            if other_pin == pin:
                continue
            if gpio.read(other_pin):
                problems.append(f"{label} high also raised {other_label}")
        gpio.write(pin, False)
        if gpio.read(pin):
            problems.append(f"{label}(GPIO{pin}) would not go low")

    if problems:
        report.fail(
            "direction pins toggle independently",
            "; ".join(problems),
            "a pin that follows another is a solder bridge or a swapped ribbon "
            "conductor; check continuity against wiring.md before powering the bridge",
        )
    else:
        report.ok("direction pins toggle independently", "IN1-IN4 read back cleanly")


def check_hardware_pwm(report: Checklist, gpio: GpioBackend, config: VehicleConfig) -> None:
    """The one check that most often fails, and the one with a specific cause."""
    pins = config.pins.motors
    clock = RealClock()
    try:
        gpio.set_pwm_pair(pins.ena, pins.enb, config.pwm_hz, 0.0, 0.0)
        gpio.set_pwm_pair(pins.ena, pins.enb, config.pwm_hz, PROBE_DUTY, 0.0)
        clock.sleep(0.2)
        gpio.set_pwm_pair(pins.ena, pins.enb, config.pwm_hz, 0.0, PROBE_DUTY)
        clock.sleep(0.2)
        gpio.set_pwm_pair(pins.ena, pins.enb, config.pwm_hz, 0.0, 0.0)
    except GpioError as exc:
        report.fail(
            "hardware PWM on both channels",
            str(exc),
            "almost always the onboard audio driver: snd_bcm2835 claims BOTH "
            "PWM channels at boot. Add 'dtparam=audio=off' to "
            "/boot/firmware/config.txt and reboot. Also confirm pigpiod runs as "
            "root, which hardware PWM requires",
        )
        return

    error = gpio.pop_gpio_error()
    if error is not None:
        report.fail(
            "hardware PWM on both channels",
            error,
            "add 'dtparam=audio=off' to /boot/firmware/config.txt and reboot",
        )
        return
    report.ok(
        "hardware PWM on both channels",
        f"GPIO{pins.ena} and GPIO{pins.enb} accepted {config.pwm_hz} Hz as a pair",
    )


def check_servo(
    report: Checklist, gpio: GpioBackend, config: VehicleConfig, interactive: bool
) -> None:
    """A 100 us nudge either side of centre. Nothing near the end stops.

    Anything wider belongs in servo_calibrate, which prompts at every step --
    a servo driven into a bound linkage stalls at 700-800 mA on the same 5 V
    rail as the SoC.
    """
    pin = config.pins.servo
    clock = RealClock()
    centre = SERVO_NOMINAL_CENTER_US
    reader = getattr(gpio, "pi", None)

    for pulse in (centre, centre - 100, centre + 100, centre):
        gpio.set_servo_pulse(pin, pulse)
        clock.sleep(0.4)
        if reader is not None:
            try:
                echoed = int(reader.get_servo_pulsewidth(pin))
            except Exception as exc:  # noqa: BLE001 - diagnostics only
                report.warn("servo pulse readback", f"pigpio refused: {exc}")
                reader = None
            else:
                if echoed != pulse:
                    report.fail(
                        "servo pulse readback",
                        f"asked for {pulse} us, daemon reports {echoed} us",
                        "check that GPIO18 is not claimed by an I2S overlay in config.txt",
                    )
                    reader = None

    error = gpio.pop_gpio_error()
    if error is not None:
        report.fail("servo pulse train", error, "check pigpiod and the GPIO18 mux")
    else:
        report.ok("servo pulse train", f"GPIO{pin} accepted 1400-1600 us")

    # Zero stops the train and the servo goes limp. That is the mechanism behind
    # servo_relax_when_disarmed, and it is worth proving by hand once.
    gpio.set_servo_pulse(pin, 0)
    if not interactive:
        report.skip("servo moved and then went limp", "no operator to confirm")
        return
    answer = ask("Did the horn move to each position and is it now free to turn by hand?")
    if answer:
        report.ok("servo moved and then went limp")
    else:
        report.fail(
            "servo moved and then went limp",
            "operator reports the servo did not move or is still holding",
            "check the 5 V feed and the signal wire; a servo that holds after "
            "pulse 0 is not being driven by this pin at all",
        )


def check_encoders(
    report: Checklist, gpio: GpioBackend, config: VehicleConfig, seconds: float
) -> None:
    """Configure the four encoder channels and watch them while nothing moves.

    Counts at rest are noise pickup, and noise pickup corrupts odometry silently
    -- there is no other symptom until a lap does not close.
    """
    pins = config.pins.encoders
    named = (
        ("L_A", pins.left_a),
        ("L_B", pins.left_b),
        ("R_A", pins.right_a),
        ("R_B", pins.right_b),
    )
    counts = {pin: 0 for _label, pin in named}

    def on_edge(pin: int, _level: int, _tick_us: int) -> None:
        counts[pin] += 1

    handles = []
    for _label, pin in named:
        gpio.setup_input(pin, Pull.UP, ENCODER_GLITCH_US)
        handles.append(gpio.add_edge_callback(pin, Edge.BOTH, on_edge))

    levels = " ".join(f"{label}={1 if gpio.read(pin) else 0}" for label, pin in named)
    report.ok("encoder channels readable", levels)

    print(f"       watching the encoders for {seconds:.0f} s -- do not touch the wheels")
    RealClock().sleep(seconds)
    for handle in handles:
        handle.cancel()

    noisy = [f"{label}={counts[pin]}" for label, pin in named if counts[pin] > 0]
    if noisy:
        report.fail(
            "no encoder counts at rest",
            f"edges arrived while stationary: {', '.join(noisy)}",
            "either a wheel moved, or this is noise pickup: check the encoder "
            "ground return and the 100 nF decoupling cap at the encoder end "
            "(wiring.md sections 5 and 6)",
        )
    else:
        report.ok("no encoder counts at rest", f"zero edges in {seconds:.0f} s")


def check_status_led(
    report: Checklist, gpio: GpioBackend, config: VehicleConfig, interactive: bool
) -> None:
    pin = config.pins.status_led
    if pin is None:
        report.skip("status LED", "not fitted in this config")
        return
    gpio.setup_output(pin, False)
    clock = RealClock()
    for _ in range(3):
        gpio.write(pin, True)
        clock.sleep(0.15)
        gpio.write(pin, False)
        clock.sleep(0.15)
    if not interactive:
        report.skip("status LED blinked", "no operator to confirm")
        return
    if ask("Did the status LED blink three times?"):
        report.ok("status LED blinked")
    else:
        report.warn(
            "status LED blinked",
            "operator saw nothing",
            "check the LED polarity and its series resistor; this is cosmetic "
            "and does not block the next phase",
        )


def check_estop_button(report: Checklist, gpio: GpioBackend, config: VehicleConfig) -> None:
    pin = config.pins.estop_button
    if pin is None:
        report.skip("E-stop button", "not fitted in this config")
        return
    gpio.setup_input(pin, Pull.UP, 5000)
    if gpio.read(pin):
        report.ok("E-stop button idle high", f"GPIO{pin} reads 1 with the internal pull-up")
    else:
        report.fail(
            "E-stop button idle high",
            f"GPIO{pin} reads 0 with nothing pressed",
            "the button is wired to ground and should read HIGH at rest. A stuck "
            "LOW means it is held, miswired, or shorted -- and the firmware would "
            "read it as a permanently pressed emergency stop",
        )


# --------------------------------------------------------------------------
# Operator interaction
# --------------------------------------------------------------------------


def ask(question: str) -> bool:
    try:
        answer = input(f"       {question} [y/N] ").strip().lower()
    except (EOFError, KeyboardInterrupt):
        print()
        return False
    return answer in ("y", "yes")


def confirm_battery_disconnected(interactive: bool) -> bool:
    banner = (
        "\n"
        "  ====================================================================\n"
        "   The MOTOR BATTERY must be DISCONNECTED for this test.\n"
        "   Every pin below gets toggled. With a live bridge that is motion.\n"
        "  ====================================================================\n"
    )
    print(banner)
    if not interactive:
        print("  --yes given: assuming the battery is disconnected.\n")
        return True
    try:
        answer = input("  Type DISCONNECTED to continue: ").strip()
    except (EOFError, KeyboardInterrupt):
        print()
        return False
    return answer == "DISCONNECTED"


# --------------------------------------------------------------------------
# Entry point
# --------------------------------------------------------------------------


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Toggle and verify every pin with the motor battery disconnected.",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument("--backend", default="auto", help="auto, pigpio or mock")
    parser.add_argument("--config", type=Path, default=None, help="vehicle config YAML")
    parser.add_argument(
        "--seconds", type=float, default=3.0, help="how long to watch the encoders at rest"
    )
    parser.add_argument(
        "--yes", action="store_true", help="skip every operator prompt (for CI)"
    )
    parser.add_argument(
        "--skip-servo", action="store_true", help="leave the servo pin alone entirely"
    )
    return parser


def load_config(path: Path | None) -> VehicleConfig:
    if path is not None:
        return VehicleConfig.load(path, LOCAL_CONFIG_PATH)
    return VehicleConfig.load_default()


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    interactive = sys.stdin.isatty() and not args.yes

    if not confirm_battery_disconnected(interactive):
        print("  Aborted. Nothing was touched.")
        return 2

    try:
        config = load_config(args.config)
    except ConfigError as exc:
        print(f"config error: {exc}", file=sys.stderr)
        return 2

    report = Checklist()
    print()
    print(f"TeleKart GPIO self-test -- {config.summary()}")
    print()

    check_environment(report, args.backend)

    try:
        gpio = select_backend(args.backend)
    except GpioError as exc:
        report.fail("GPIO backend", str(exc), "sudo systemctl enable --now pigpiod")
        return report.summary()

    panic = HardwarePanic(gpio, config.pins, config.pwm_hz)
    with PanicChain(panic):
        try:
            check_boot_levels(report, gpio, config)
            check_direction_pins(report, gpio, config)
            check_hardware_pwm(report, gpio, config)
            if args.skip_servo:
                report.skip("servo pulse train", "--skip-servo")
            else:
                check_servo(report, gpio, config, interactive)
            check_encoders(report, gpio, config, args.seconds)
            check_status_led(report, gpio, config, interactive)
            check_estop_button(report, gpio, config)
        except KeyboardInterrupt:
            print("\n  interrupted -- outputs are being safed")
            report.fail("test completed", "aborted by the operator")
        finally:
            gpio.cleanup()

    return report.summary()


if __name__ == "__main__":
    raise SystemExit(main())
