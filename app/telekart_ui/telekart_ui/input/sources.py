"""Where driver input comes from.

Three implementations of one three-method protocol, and the point of the protocol
is that everything downstream -- the chain, the TX thread, the HUD -- cannot tell
them apart:

* `JoystickSource` reads the SDL axis cache. The real one.
* `KeyboardSource` turns WASD and the arrow keys into the same numbers, so the
  app is usable with no wheel in the bag.
* `NullSource` is neutral and disconnected, so "nothing plugged in" is the same
  code path as "unplugged mid-drive" rather than a `None` check everywhere.

`DeviceSnapshot` lives here rather than in `sdl_backend` so a test can build one
with pygame not installed.
"""

from __future__ import annotations

import time
from dataclasses import dataclass
from typing import Callable, Protocol

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
