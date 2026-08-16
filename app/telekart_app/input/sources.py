"""Where driver input comes from.

Four implementations of one three-method protocol, and the point of the protocol
is that everything downstream -- the chain, the TX thread, the HUD -- cannot tell
them apart:

* `JoystickSource` reads the SDL axis cache. The real one.
* `KeyboardSource` turns arrow keys into the same numbers, so the app is usable
  on a train with no wheel in the bag.
* `ScriptSource` drives a complete lap from a table of times. No hardware, no
  clock skew, no flakiness -- an integration test can run a full drive in
  milliseconds and assert exact numbers.
* `ReplaySource` plays back a recording of a real drive, which is how a bug that
  only happens on the third corner gets debugged more than once.

`DeviceSnapshot` also lives here rather than in `sdl_backend`, so the calibration
state machine and every test can build one without pygame installed.
"""

from __future__ import annotations

import json
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Callable, Iterable, Protocol, Sequence

from .mapping import (
    Action,
    ActionBinding,
    Control,
    ControlBinding,
    EdgeTracker,
    InputMap,
    InputRef,
    RefKind,
    action_bit,
    resolve_actions,
    resolve_binding,
)

Clock = Callable[[], float]

#: Buttons index a bitmask, so this is the number of them we can represent.
MAX_BUTTONS = 64


@dataclass(frozen=True, slots=True)
class DeviceSnapshot:
    """A coherent instant of one device's state.

    Published by the SDL pump as a single immutable object rather than as
    mutable per-axis fields. The reason is concurrency: the pump runs on the Qt
    main thread and the input thread reads at 250 Hz, and swapping one reference
    is atomic under the GIL while updating six floats in place is not. Without
    this, a reader could get steering from one instant and throttle from the
    next -- rare, silent, and exactly the kind of thing that shows up once a
    fortnight as an unexplained twitch.
    """

    t: float = 0.0
    #: Increments once per pump. Consumers use it to spot a stalled pump and to
    #: avoid double-counting the latched press edges.
    generation: int = 0
    axes: tuple[float, ...] = ()
    #: Level bitmask, bit i = button i.
    buttons: int = 0
    button_count: int = 0
    hats: tuple[tuple[int, int], ...] = ()
    #: Buttons that saw a JOYBUTTONDOWN since the previous snapshot, even if they
    #: were released again before this one was taken.
    pressed_edges: int = 0
    connected: bool = False


EMPTY_SNAPSHOT = DeviceSnapshot()


@dataclass(frozen=True, slots=True)
class RawSample:
    """One poll of one source, before any shaping.

    `steer`/`throttle`/`brake` are in raw device units -- whatever the hardware
    reports. Calibration is the chain's first stage, not the source's job, so
    that the wizard can recalibrate a live device without restarting anything.
    """

    t: float = 0.0
    steer: float = 0.0
    throttle: float = 0.0
    brake: float = 0.0
    #: Action bitmasks (`1 << Action`). `pressed` is the rising edges observed
    #: since the previous poll, including taps too short to appear in `held`.
    held: int = 0
    pressed: int = 0
    released: int = 0
    connected: bool = False
    source: str = ""

    def is_held(self, action: Action) -> bool:
        return bool(self.held & action_bit(action))

    def was_pressed(self, action: Action) -> bool:
        return bool(self.pressed & action_bit(action))

    def to_dict(self) -> dict[str, Any]:
        return {
            "t": round(self.t, 6),
            "s": round(self.steer, 6),
            "th": round(self.throttle, 6),
            "b": round(self.brake, 6),
            "h": self.held,
            "p": self.pressed,
            "r": self.released,
            "c": self.connected,
        }

    @classmethod
    def from_dict(cls, data: dict[str, Any], *, source: str = "replay") -> "RawSample":
        return cls(
            t=float(data.get("t", 0.0)),
            steer=float(data.get("s", 0.0)),
            throttle=float(data.get("th", 0.0)),
            brake=float(data.get("b", 0.0)),
            held=int(data.get("h", 0)),
            pressed=int(data.get("p", 0)),
            released=int(data.get("r", 0)),
            connected=bool(data.get("c", True)),
            source=source,
        )


NEUTRAL_SAMPLE = RawSample()


class InputSource(Protocol):
    """Everything the input thread is allowed to know about its input."""

    @property
    def name(self) -> str:
        """Driver-facing device name."""
        ...

    @property
    def source_id(self) -> str:
        """Stable identity for profile lookup: a joystick GUID, or a fixed
        string for the synthetic sources."""
        ...

    @property
    def connected(self) -> bool: ...

    def poll(self) -> RawSample:
        """Read the current state. Must never block and must never raise."""
        ...

    def close(self) -> None: ...


class DeviceReader(Protocol):
    """The slice of `sdl_backend.SdlDevice` that `JoystickSource` needs.

    Structural, so sources.py -- and its tests -- never import pygame.
    """

    @property
    def guid(self) -> str: ...

    @property
    def name(self) -> str: ...

    @property
    def connected(self) -> bool: ...

    @property
    def snapshot(self) -> DeviceSnapshot: ...


# --------------------------------------------------------------------------
# Joystick
# --------------------------------------------------------------------------


class JoystickSource:
    """A wheel, a gamepad, or anything else SDL enumerates.

    Does no SDL work itself. It reads the snapshot the pump published, which is
    the whole trick behind keeping SDL on the Qt main thread while the input
    thread runs free: this side is three tuple indexes and some integer maths.
    """

    __slots__ = ("_device", "_map", "_clock", "_edges", "_last_generation", "_lost")

    def __init__(
        self,
        device: DeviceReader,
        mapping: InputMap,
        *,
        clock: Clock = time.perf_counter,
    ) -> None:
        self._device = device
        self._map = mapping
        self._clock = clock
        self._edges = EdgeTracker()
        self._last_generation = -1
        self._lost = False

    @property
    def name(self) -> str:
        return self._device.name

    @property
    def source_id(self) -> str:
        return self._device.guid

    @property
    def device(self) -> DeviceReader:
        return self._device

    @property
    def connected(self) -> bool:
        return self._device.connected

    @property
    def mapping(self) -> InputMap:
        return self._map

    def set_mapping(self, mapping: InputMap) -> None:
        """Rebind live. Safe mid-drive: the next poll simply reads different
        indices, and the chain's rate limiter absorbs any resulting step."""
        self._map = mapping

    def poll(self) -> RawSample:
        snapshot = self._device.snapshot
        if not snapshot.connected:
            if not self._lost:
                self._lost = True
                self._edges.reset()
                self._last_generation = -1
            return RawSample(
                t=self._clock(), connected=False, source=self._device.name
            )
        self._lost = False

        mapping = self._map
        axes = snapshot.axes
        buttons = snapshot.buttons
        hats = snapshot.hats

        held = resolve_actions(mapping.actions, axes, buttons, hats, 0)
        self._edges.update(held)
        pressed = self._edges.pressed

        # Merge the taps the level snapshot could not show. Only once per
        # snapshot: the input thread polls faster than the pump publishes, so
        # re-reading the same latched edges would fire an action several times.
        if snapshot.generation != self._last_generation:
            self._last_generation = snapshot.generation
            if snapshot.pressed_edges:
                pressed |= resolve_actions(
                    mapping.actions, (), snapshot.pressed_edges, (), 0
                )

        return RawSample(
            t=snapshot.t,
            steer=resolve_binding(mapping.steer, axes, buttons, hats, 0),
            throttle=resolve_binding(mapping.throttle, axes, buttons, hats, 0),
            brake=resolve_binding(mapping.brake, axes, buttons, hats, 0),
            held=held,
            pressed=pressed,
            released=self._edges.released,
            connected=True,
            source=self._device.name,
        )

    def close(self) -> None:
        self._edges.reset()


# --------------------------------------------------------------------------
# Keyboard
# --------------------------------------------------------------------------


class KeyboardSource:
    """Arrow keys as a wheel. Digital by nature, so the chain's rate limiter is
    what makes it drivable at all -- see `default_chain_config`.

    Threading: the GUI thread calls `press`/`release`, the input thread calls
    `poll`. The shared state is one integer. There is exactly one writer, so the
    read-modify-write in `press` cannot race with another writer, and the reader
    observes either the old value or the new one -- never a partial. This is the
    same single-writer argument the firmware's control mailbox rests on; do not
    "fix" it into a lock.
    """

    __slots__ = ("_map", "_resolved", "_bits", "_mask", "_clock", "_edges", "_name")

    def __init__(
        self,
        mapping: InputMap,
        *,
        clock: Clock = time.perf_counter,
        name: str = "Keyboard",
    ) -> None:
        self._clock = clock
        self._name = name
        self._edges = EdgeTracker()
        self._mask = 0
        self._bits: dict[int, int] = {}
        self._map = mapping
        self._resolved = self._compact(mapping)

    def _compact(self, mapping: InputMap) -> InputMap:
        """Rewrite Qt key codes into bit positions.

        Key codes are large and sparse (Qt.Key_Left is 0x01000012), so they
        cannot index a bitmask directly. Every KEY ref gets a compact bit here,
        and `poll` resolves against that. The original mapping is left untouched
        for display and persistence.
        """
        self._bits = {}
        remapped = mapping
        for control in Control:
            binding = mapping.binding(control)
            remapped = remapped.with_binding(
                control,
                ControlBinding(
                    self._compact_ref(binding.positive),
                    self._compact_ref(binding.negative),
                ),
            )
        actions = tuple(
            ActionBinding(entry.action, self._compact_ref(entry.ref))
            for entry in mapping.actions
        )
        return InputMap(remapped.steer, remapped.throttle, remapped.brake, actions)

    def _compact_ref(self, ref: InputRef) -> InputRef:
        if ref.kind is not RefKind.KEY:
            return ref
        bit = self._bits.get(ref.index)
        if bit is None:
            if len(self._bits) >= MAX_BUTTONS:
                # Out of bits. Dropping the binding is better than corrupting a
                # neighbouring one, and it is visible in the bindings screen.
                return InputRef()
            bit = len(self._bits)
            self._bits[ref.index] = bit
        return InputRef(RefKind.KEY, bit)

    @property
    def name(self) -> str:
        return self._name

    @property
    def source_id(self) -> str:
        return "keyboard"

    @property
    def connected(self) -> bool:
        return True

    @property
    def mapping(self) -> InputMap:
        return self._map

    def set_mapping(self, mapping: InputMap) -> None:
        self._map = mapping
        self._resolved = self._compact(mapping)
        self._mask = 0

    def bound_keys(self) -> tuple[int, ...]:
        """Qt key codes this source cares about. The window filters on these so
        that typing in a text field does not steer the car."""
        return tuple(self._bits)

    def press(self, key_code: int) -> None:
        bit = self._bits.get(key_code)
        if bit is not None:
            self._mask |= 1 << bit

    def release(self, key_code: int) -> None:
        bit = self._bits.get(key_code)
        if bit is not None:
            self._mask &= ~(1 << bit)

    def release_all(self) -> None:
        """Call on focus-out. A window that loses focus stops delivering key
        releases, and a throttle key stuck down because the driver alt-tabbed is
        the worst possible way to learn that."""
        self._mask = 0

    def poll(self) -> RawSample:
        mask = self._mask
        mapping = self._resolved
        held = resolve_actions(mapping.actions, (), 0, (), mask)
        self._edges.update(held)
        return RawSample(
            t=self._clock(),
            steer=resolve_binding(mapping.steer, (), 0, (), mask),
            throttle=resolve_binding(mapping.throttle, (), 0, (), mask),
            brake=resolve_binding(mapping.brake, (), 0, (), mask),
            held=held,
            pressed=self._edges.pressed,
            released=self._edges.released,
            connected=True,
            source=self._name,
        )

    def close(self) -> None:
        self._mask = 0
        self._edges.reset()


# --------------------------------------------------------------------------
# Script
# --------------------------------------------------------------------------


@dataclass(frozen=True, slots=True)
class ScriptStep:
    """A held or ramped input over a span of time."""

    duration: float
    steer: float = 0.0
    throttle: float = 0.0
    brake: float = 0.0
    held: int = 0
    #: Ramp linearly from the previous step's endpoint instead of stepping. A
    #: step change is legal input and worth testing, but a whole lap of them
    #: tests the rate limiter and nothing else.
    ramp: bool = True
    label: str = ""

    def __post_init__(self) -> None:
        if not (self.duration > 0.0):
            raise ValueError(f"script step duration must be positive: {self.duration}")


class ScriptSource:
    """A pre-programmed drive. Deterministic, hardware-free, clock-injectable.

    Emits values in *normalized* units, so pair it with `profile.identity_profile`
    -- calibrating a script against a wheel's raw range would be nonsense.
    """

    __slots__ = ("_steps", "_clock", "_loop", "_t0", "_total", "_name", "_edges", "_finished")

    def __init__(
        self,
        steps: Sequence[ScriptStep],
        *,
        clock: Clock = time.perf_counter,
        loop: bool = False,
        name: str = "Script",
    ) -> None:
        if not steps:
            raise ValueError("a script needs at least one step")
        self._steps = tuple(steps)
        self._clock = clock
        self._loop = loop
        self._name = name
        self._t0 = clock()
        self._total = sum(s.duration for s in self._steps)
        self._edges = EdgeTracker()
        self._finished = False

    # -- lifecycle ----------------------------------------------------------

    @property
    def name(self) -> str:
        return self._name

    @property
    def source_id(self) -> str:
        return "script"

    @property
    def connected(self) -> bool:
        return True

    @property
    def duration(self) -> float:
        return self._total

    @property
    def elapsed(self) -> float:
        return self._clock() - self._t0

    @property
    def progress(self) -> float:
        if self._total <= 0.0:
            return 1.0
        value = self.elapsed / self._total
        return 1.0 if value > 1.0 else 0.0 if value < 0.0 else value

    @property
    def finished(self) -> bool:
        return self._finished

    def rewind(self) -> None:
        self._t0 = self._clock()
        self._finished = False
        self._edges.reset()

    # -- evaluation ---------------------------------------------------------

    def sample_at(self, elapsed: float) -> RawSample:
        """The scripted state at a given offset. Pure; used by tests directly."""
        total = self._total
        if elapsed >= total:
            if self._loop and total > 0.0:
                elapsed = elapsed % total
            else:
                last = self._steps[-1]
                return RawSample(
                    t=elapsed,
                    steer=last.steer,
                    throttle=last.throttle,
                    brake=last.brake,
                    held=last.held,
                    connected=True,
                    source=self._name,
                )
        if elapsed < 0.0:
            elapsed = 0.0

        start = 0.0
        prev_steer = 0.0
        prev_throttle = 0.0
        prev_brake = 0.0
        for step in self._steps:
            end = start + step.duration
            if elapsed < end:
                if step.ramp:
                    frac = (elapsed - start) / step.duration
                    steer = prev_steer + (step.steer - prev_steer) * frac
                    throttle = prev_throttle + (step.throttle - prev_throttle) * frac
                    brake = prev_brake + (step.brake - prev_brake) * frac
                else:
                    steer, throttle, brake = step.steer, step.throttle, step.brake
                return RawSample(
                    t=elapsed,
                    steer=steer,
                    throttle=throttle,
                    brake=brake,
                    held=step.held,
                    connected=True,
                    source=self._name,
                )
            start = end
            prev_steer, prev_throttle, prev_brake = step.steer, step.throttle, step.brake

        last = self._steps[-1]
        return RawSample(
            t=elapsed,
            steer=last.steer,
            throttle=last.throttle,
            brake=last.brake,
            held=last.held,
            connected=True,
            source=self._name,
        )

    def poll(self) -> RawSample:
        elapsed = self._clock() - self._t0
        if elapsed >= self._total and not self._loop:
            self._finished = True
        sample = self.sample_at(elapsed)
        self._edges.update(sample.held)
        return RawSample(
            t=sample.t,
            steer=sample.steer,
            throttle=sample.throttle,
            brake=sample.brake,
            held=sample.held,
            pressed=self._edges.pressed,
            released=self._edges.released,
            connected=True,
            source=self._name,
        )

    def close(self) -> None:
        self._finished = True

    # -- canned programmes --------------------------------------------------

    @classmethod
    def full_lap(
        cls, *, clock: Clock = time.perf_counter, loop: bool = False
    ) -> "ScriptSource":
        """One lap of a small circuit: launch, two corners, a chicane, a hairpin.

        Sized for a car whose real top speed is 150-200 RPM, so the straights are
        short and the corners are slow. Every stage of the chain gets exercised:
        full lock both ways, full and partial throttle, trail braking, a
        simultaneous throttle-and-brake overlap, and a stop at the end.
        """
        arm = action_bit(Action.ARM)
        return cls(
            [
                ScriptStep(0.5, label="grid", ramp=False),
                ScriptStep(0.2, held=arm, label="arm", ramp=False),
                ScriptStep(2.0, throttle=0.85, label="launch"),
                ScriptStep(2.0, throttle=1.00, label="main straight"),
                ScriptStep(0.8, throttle=0.0, brake=0.90, label="brake for turn 1"),
                ScriptStep(1.2, steer=-0.95, throttle=0.25, brake=0.15, label="turn 1 entry"),
                ScriptStep(1.0, steer=-0.60, throttle=0.55, label="turn 1 exit"),
                ScriptStep(1.5, steer=0.0, throttle=0.90, label="short straight"),
                ScriptStep(0.6, steer=0.70, throttle=0.40, label="chicane left"),
                ScriptStep(0.6, steer=-0.70, throttle=0.40, label="chicane right"),
                ScriptStep(1.0, steer=0.0, throttle=1.00, label="back straight"),
                ScriptStep(0.9, throttle=0.0, brake=1.00, label="brake for hairpin"),
                ScriptStep(2.0, steer=1.00, throttle=0.30, label="hairpin"),
                ScriptStep(1.5, steer=0.20, throttle=0.80, label="hairpin exit"),
                ScriptStep(2.0, steer=0.0, throttle=1.00, label="start-finish"),
                ScriptStep(1.5, throttle=0.0, brake=0.60, label="slow down"),
                ScriptStep(1.0, label="stopped", ramp=False),
            ],
            clock=clock,
            loop=loop,
            name="Scripted lap",
        )

    @classmethod
    def sweep(
        cls, *, clock: Clock = time.perf_counter, loop: bool = True
    ) -> "ScriptSource":
        """Lock to lock and pedal to floor, slowly. For rig checks and for
        watching the HUD track the command."""
        return cls(
            [
                ScriptStep(2.0, steer=1.0, label="steer right"),
                ScriptStep(4.0, steer=-1.0, label="steer left"),
                ScriptStep(2.0, steer=0.0, label="centre"),
                ScriptStep(2.0, throttle=1.0, label="throttle up"),
                ScriptStep(2.0, throttle=0.0, label="throttle down"),
                ScriptStep(2.0, brake=1.0, label="brake on"),
                ScriptStep(2.0, brake=0.0, label="brake off"),
            ],
            clock=clock,
            loop=loop,
            name="Sweep",
        )


# --------------------------------------------------------------------------
# Replay
# --------------------------------------------------------------------------


class ReplaySource:
    """Plays recorded samples back against a clock.

    Timestamps in the recording are treated as relative to its first sample, so a
    recording made three days ago replays from zero.
    """

    __slots__ = ("_samples", "_clock", "_loop", "_speed", "_t0", "_index", "_name", "_span")

    def __init__(
        self,
        samples: Sequence[RawSample],
        *,
        clock: Clock = time.perf_counter,
        loop: bool = False,
        speed: float = 1.0,
        name: str = "Replay",
    ) -> None:
        if not samples:
            raise ValueError("a replay needs at least one sample")
        if not (speed > 0.0):
            raise ValueError(f"replay speed must be positive: {speed}")
        base = samples[0].t
        # Normalize once at construction so `poll` stays a comparison and an
        # index bump.
        self._samples = tuple(
            RawSample(
                t=s.t - base,
                steer=s.steer,
                throttle=s.throttle,
                brake=s.brake,
                held=s.held,
                pressed=s.pressed,
                released=s.released,
                connected=s.connected,
                source=name,
            )
            for s in samples
        )
        self._clock = clock
        self._loop = loop
        self._speed = speed
        self._name = name
        self._t0 = clock()
        self._index = 0
        self._span = self._samples[-1].t

    @property
    def name(self) -> str:
        return self._name

    @property
    def source_id(self) -> str:
        return "replay"

    @property
    def connected(self) -> bool:
        return True

    @property
    def duration(self) -> float:
        return self._span

    @property
    def finished(self) -> bool:
        return not self._loop and self._index >= len(self._samples) - 1

    def rewind(self) -> None:
        self._t0 = self._clock()
        self._index = 0

    def poll(self) -> RawSample:
        elapsed = (self._clock() - self._t0) * self._speed
        if self._loop and self._span > 0.0 and elapsed > self._span:
            elapsed = elapsed % self._span
            if elapsed < self._samples[self._index].t:
                self._index = 0
        samples = self._samples
        limit = len(samples) - 1
        index = self._index
        while index < limit and samples[index + 1].t <= elapsed:
            index += 1
        self._index = index
        return samples[index]

    def close(self) -> None:
        self._index = len(self._samples) - 1

    @classmethod
    def from_jsonl(
        cls,
        path: str | Path,
        *,
        clock: Clock = time.perf_counter,
        loop: bool = False,
        speed: float = 1.0,
    ) -> "ReplaySource":
        samples: list[RawSample] = []
        name = Path(path).stem
        with open(path, "r", encoding="utf-8") as handle:
            for line_no, line in enumerate(handle, 1):
                line = line.strip()
                if not line:
                    continue
                try:
                    obj = json.loads(line)
                except json.JSONDecodeError as exc:
                    raise ValueError(f"{path}:{line_no}: {exc}") from exc
                if not isinstance(obj, dict):
                    raise ValueError(f"{path}:{line_no}: expected a JSON object")
                samples.append(RawSample.from_dict(obj, source=name))
        if not samples:
            raise ValueError(f"{path} contains no samples")
        return cls(samples, clock=clock, loop=loop, speed=speed, name=name)


def write_jsonl(path: str | Path, samples: Iterable[RawSample]) -> int:
    """Record samples for later replay. Returns the number written."""
    count = 0
    with open(path, "w", encoding="utf-8") as handle:
        for sample in samples:
            handle.write(json.dumps(sample.to_dict(), separators=(",", ":")))
            handle.write("\n")
            count += 1
    return count


# --------------------------------------------------------------------------
# Nothing at all
# --------------------------------------------------------------------------


class NullSource:
    """No device. Always neutral, always disconnected.

    The app starts with this rather than with `None`, so that every downstream
    consumer has one code path instead of two and the "no wheel plugged in" case
    is the same code that handles "wheel unplugged mid-drive".
    """

    __slots__ = ("_clock",)

    def __init__(self, *, clock: Clock = time.perf_counter) -> None:
        self._clock = clock

    @property
    def name(self) -> str:
        return "No input device"

    @property
    def source_id(self) -> str:
        return "null"

    @property
    def connected(self) -> bool:
        return False

    def poll(self) -> RawSample:
        return RawSample(t=self._clock(), connected=False, source="null")

    def close(self) -> None:
        return None
