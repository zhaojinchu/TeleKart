"""SDL joystick access, arranged so that SDL and Qt can share one macOS process.

This is the highest-risk integration point in the desktop app. SDL2 and Qt both
want to own the Cocoa application object and the main run loop, and when they
fight over it the symptom is not a clean error -- it is a beachball, or a wheel
that stops reporting the moment a dialog opens, or a crash inside AppKit with a
stack that mentions neither library. The mitigations below are ordered, and each
one is load-bearing.

**1. Never create an SDL window.** `pygame.display.set_mode()` is never called
anywhere in this codebase. Only the display *module* is initialised, and only
with the dummy video driver, which touches no Cocoa API at all. No NSWindow, no
NSApplication, no app-delegate contention. The reason the display module has to
be initialised at all is unglamorous: pygame's event functions are guarded by
`VIDEO_INIT_CHECK()`, so `pygame.event.pump()` raises "video system not
initialized" without it. The dummy driver satisfies the check and costs nothing.

**2. Pump on the Qt main thread; read the cache from the input thread.**
`SdlHub.pump()` must be called from a QTimer on the GUI thread -- see
`install_qt_pump`. macOS HID hotplug rides CFRunLoop, and SDL's Darwin joystick
backend runs the run loop from whichever thread pumps; pumping off the main
thread is not supported and fails in exactly the intermittent way that is
hardest to diagnose. The 250 Hz input thread never calls into SDL; it reads
`SdlDevice.snapshot`, which is a single attribute load of an immutable object.
`SDL_JoystickGetAxis` is roughly a 20 us array read, but it is not thread-safe
against `SDL_JoystickUpdate`, so caching is what makes the split legal as well
as fast.

**3. The failure mode of (2) is benign, and that is the point.** If the GUI
thread stalls -- a slow repaint, a modal menu tracking loop, a hitch on the
video decoder -- the pump stops and the input thread reads axis values a few
milliseconds stale. The TX thread keeps sending on time with those values. The
car sees a continuous, valid control stream and does not trip its failsafe. A
few milliseconds of stale steering during a UI hitch is a far better outcome
than the alternative design, where a stalled UI stops the control stream and the
car begins a failsafe brake in the middle of a corner. `SdlHub.stale_for()`
exposes the staleness so the HUD can say so out loud.
"""

from __future__ import annotations

import os
import time
from typing import Any, Callable, Iterable

from .sources import MAX_BUTTONS, DeviceSnapshot

# --------------------------------------------------------------------------
# Import guard
# --------------------------------------------------------------------------

try:
    import pygame
except ImportError as exc:  # pragma: no cover - environment problem, not logic
    raise ImportError(
        "TeleKart needs pygame-ce for wheel and gamepad input.\n"
        "    pip install pygame-ce\n"
        "Do not install plain 'pygame' as well: the two projects claim the same "
        "module name and having both in one environment breaks the import."
    ) from exc

if not getattr(pygame, "IS_CE", 0):
    # Upstream pygame and pygame-ce are API-compatible enough that this would
    # mostly work, and "mostly" is the problem: the wheels lag by months on
    # macOS arm64 and there is no cp314 build. Failing here with instructions
    # beats failing later with a segfault in a wheel built for another ABI.
    raise ImportError(
        "The installed 'pygame' is upstream pygame, not pygame-ce.\n"
        "    pip uninstall -y pygame pygame-ce && pip install pygame-ce\n"
        "Both projects install a module named 'pygame'; only one can win, and "
        "TeleKart is built and tested against pygame-ce."
    )


#: How long the input thread may go without a fresh pump before the UI is
#: considered stalled. Two 60 Hz frames plus slack.
DEFAULT_STALE_S = 0.050

#: Rescan interval for the hotplug fallback.
DEFAULT_HOTPLUG_POLL_S = 2.0


class SdlDevice:
    """One joystick, and the snapshot the input thread reads.

    Everything that calls into SDL happens on the pump thread. The only member
    the input thread touches is `snapshot`, and it is replaced wholesale rather
    than mutated.
    """

    __slots__ = (
        "_js",
        "_guid",
        "_name",
        "_instance_id",
        "_num_axes",
        "_num_buttons",
        "_num_hats",
        "_connected",
        "_pending_edges",
        "snapshot",
    )

    def __init__(self, js: Any) -> None:
        self._js = js
        self._name = str(js.get_name())
        self._instance_id = int(js.get_instance_id())
        self._num_axes = int(js.get_numaxes())
        self._num_buttons = min(int(js.get_numbuttons()), MAX_BUTTONS)
        self._num_hats = int(js.get_numhats())
        self._guid = _device_guid(js, self._name, self._num_axes, self._num_buttons)
        self._connected = True
        self._pending_edges = 0
        self.snapshot: DeviceSnapshot = DeviceSnapshot(
            t=time.perf_counter(),
            generation=0,
            axes=(0.0,) * self._num_axes,
            buttons=0,
            button_count=self._num_buttons,
            hats=((0, 0),) * self._num_hats,
            pressed_edges=0,
            connected=True,
        )

    # -- identity -----------------------------------------------------------

    @property
    def guid(self) -> str:
        return self._guid

    @property
    def name(self) -> str:
        return self._name

    @property
    def instance_id(self) -> int:
        return self._instance_id

    @property
    def num_axes(self) -> int:
        return self._num_axes

    @property
    def num_buttons(self) -> int:
        return self._num_buttons

    @property
    def num_hats(self) -> int:
        return self._num_hats

    @property
    def connected(self) -> bool:
        return self._connected

    # -- pump side ----------------------------------------------------------

    def note_button_down(self, button: int) -> None:
        """Latch a press so a tap between two refreshes is not lost."""
        if 0 <= button < MAX_BUTTONS:
            self._pending_edges |= 1 << button

    def refresh(self, now: float, generation: int) -> None:
        """Re-read every control and publish a new snapshot. Pump thread only."""
        js = self._js
        axes = tuple(js.get_axis(i) for i in range(self._num_axes))
        mask = 0
        for i in range(self._num_buttons):
            if js.get_button(i):
                mask |= 1 << i
        hats = tuple(js.get_hat(i) for i in range(self._num_hats))
        # Consume the latched presses: they belong to exactly one snapshot, and
        # the consumer de-duplicates on `generation`.
        edges = self._pending_edges
        self._pending_edges = 0
        self.snapshot = DeviceSnapshot(
            t=now,
            generation=generation,
            axes=axes,
            buttons=mask,
            button_count=self._num_buttons,
            hats=hats,
            pressed_edges=edges,
            connected=True,
        )

    def mark_disconnected(self, now: float, generation: int) -> None:
        self._connected = False
        self._pending_edges = 0
        self.snapshot = DeviceSnapshot(
            t=now,
            generation=generation,
            axes=(0.0,) * self._num_axes,
            buttons=0,
            button_count=self._num_buttons,
            hats=((0, 0),) * self._num_hats,
            pressed_edges=0,
            connected=False,
        )

    def close(self) -> None:
        if self._js is not None:
            try:
                self._js.quit()
            except Exception:
                # A device yanked out of the port throws from quit() on some
                # SDL builds. There is nothing useful to do about it and it must
                # not take the app down.
                pass
            self._js = None
        self._connected = False

    def __repr__(self) -> str:
        return (
            f"SdlDevice({self._name!r}, axes={self._num_axes}, "
            f"buttons={self._num_buttons}, hats={self._num_hats})"
        )


class SdlHub:
    """Owns SDL's joystick subsystem and the pump.

    Construct exactly one, on the Qt main thread, and hand `pump` to a QTimer.
    """

    __slots__ = (
        "_devices",
        "_generation",
        "_last_pump",
        "_last_device_event",
        "_last_rescan",
        "_hotplug_poll_s",
        "_started",
        "_video_driver",
        "_on_added",
        "_on_removed",
        "_pumps",
        "_worst_gap",
    )

    def __init__(
        self,
        *,
        hotplug_poll_s: float = DEFAULT_HOTPLUG_POLL_S,
        on_added: Callable[[SdlDevice], None] | None = None,
        on_removed: Callable[[SdlDevice], None] | None = None,
    ) -> None:
        self._devices: dict[int, SdlDevice] = {}
        self._generation = 0
        self._last_pump = 0.0
        self._last_device_event = 0.0
        self._last_rescan = 0.0
        self._hotplug_poll_s = hotplug_poll_s
        self._started = False
        self._video_driver = ""
        self._on_added = on_added
        self._on_removed = on_removed
        self._pumps = 0
        self._worst_gap = 0.0

    # -- lifecycle ----------------------------------------------------------

    def start(self) -> None:
        """Initialise SDL. Call once, from the Qt main thread, before any pump."""
        if self._started:
            return
        _init_sdl()
        self._video_driver = str(pygame.display.get_driver())
        now = time.perf_counter()
        self._last_pump = now
        self._last_device_event = now
        self._last_rescan = now
        self._started = True
        self._rescan(now)

    def stop(self) -> None:
        for device in self._devices.values():
            device.close()
        self._devices.clear()
        if self._started:
            try:
                pygame.joystick.quit()
            except Exception:
                pass
            self._started = False

    def __enter__(self) -> "SdlHub":
        self.start()
        return self

    def __exit__(self, *exc: object) -> None:
        self.stop()

    # -- introspection ------------------------------------------------------

    @property
    def started(self) -> bool:
        return self._started

    @property
    def video_driver(self) -> str:
        """Which SDL video driver backs the event queue. Expected: 'dummy'."""
        return self._video_driver

    @property
    def generation(self) -> int:
        return self._generation

    @property
    def pumps(self) -> int:
        return self._pumps

    @property
    def worst_gap(self) -> float:
        """Longest observed interval between pumps, in seconds. This is a direct
        measure of how badly the GUI thread has been stalling."""
        return self._worst_gap

    def reset_stats(self) -> None:
        self._worst_gap = 0.0
        self._pumps = 0

    def devices(self) -> tuple[SdlDevice, ...]:
        return tuple(self._devices.values())

    def first_device(self) -> SdlDevice | None:
        for device in self._devices.values():
            if device.connected:
                return device
        return None

    def device_by_guid(self, guid: str) -> SdlDevice | None:
        for device in self._devices.values():
            if device.guid == guid and device.connected:
                return device
        return None

    def stale_for(self, now: float | None = None) -> float:
        """Seconds since the last pump. See mitigation (3) in the module note:
        a large value means the UI is stalled, not that the link is bad."""
        if not self._started:
            return 0.0
        return (time.perf_counter() if now is None else now) - self._last_pump

    def is_stalled(self, threshold: float = DEFAULT_STALE_S) -> bool:
        return self.stale_for() > threshold

    # -- the pump -----------------------------------------------------------

    def pump(self) -> None:
        """Drain SDL events and refresh every device cache.

        **Qt main thread only.** Never raises: an exception here would kill the
        QTimer that drives it and silently freeze all input.
        """
        if not self._started:
            return
        now = time.perf_counter()
        gap = now - self._last_pump
        if gap > self._worst_gap:
            self._worst_gap = gap
        self._last_pump = now
        self._pumps += 1
        self._generation += 1
        generation = self._generation

        try:
            self._drain_events(now)
        except Exception:
            # Losing one pump's events is survivable; losing the timer is not.
            pass

        for device in self._devices.values():
            if not device.connected:
                continue
            try:
                device.refresh(now, generation)
            except Exception:
                # The device went away between the event drain and this read.
                device.mark_disconnected(now, generation)
                self._notify_removed(device)

        if now - self._last_device_event > self._hotplug_poll_s:
            self._poll_hotplug(now)

    def _drain_events(self, now: float) -> None:
        for event in pygame.event.get():
            kind = event.type
            if kind == pygame.JOYDEVICEADDED:
                self._last_device_event = now
                self._open(int(event.device_index), now)
            elif kind == pygame.JOYDEVICEREMOVED:
                self._last_device_event = now
                self._close(int(event.instance_id), now)
            elif kind == pygame.JOYBUTTONDOWN:
                device = self._devices.get(int(event.instance_id))
                if device is not None:
                    device.note_button_down(int(event.button))

    def _poll_hotplug(self, now: float) -> None:
        """Fallback for when device events never arrive.

        SDL's hotplug notifications depend on the platform HID backend, and with
        no video window in play there are configurations -- remote sessions,
        certain SDL builds, a USB hub that re-enumerates -- where the events
        simply do not come. Comparing the device count every couple of seconds
        costs nothing and turns "the wheel never appears until you restart the
        app" into a two-second delay nobody notices.
        """
        self._last_rescan = now
        try:
            count = pygame.joystick.get_count()
        except Exception:
            return
        live = sum(1 for d in self._devices.values() if d.connected)
        if count != live:
            self._rescan(now)

    def _rescan(self, now: float) -> None:
        try:
            count = pygame.joystick.get_count()
        except Exception:
            return
        seen: set[int] = set()
        for index in range(count):
            try:
                js = pygame.joystick.Joystick(index)
                instance_id = int(js.get_instance_id())
            except Exception:
                continue
            seen.add(instance_id)
            if instance_id not in self._devices:
                self._adopt(js, now)
        for instance_id in [i for i in self._devices if i not in seen]:
            self._close(instance_id, now)

    def _open(self, device_index: int, now: float) -> None:
        try:
            js = pygame.joystick.Joystick(device_index)
        except Exception:
            return
        self._adopt(js, now)

    def _adopt(self, js: Any, now: float) -> None:
        try:
            js.init()
        except Exception:
            # pygame-ce initialises on construction; older builds need this and
            # newer ones treat it as a no-op. Either way a failure here means
            # the device is unusable, not that the app is broken.
            return
        try:
            device = SdlDevice(js)
        except Exception:
            return
        existing = self._devices.get(device.instance_id)
        if existing is not None and existing.connected:
            return
        self._devices[device.instance_id] = device
        device.refresh(now, self._generation)
        if self._on_added is not None:
            try:
                self._on_added(device)
            except Exception:
                pass

    def _close(self, instance_id: int, now: float) -> None:
        device = self._devices.pop(instance_id, None)
        if device is None:
            return
        device.mark_disconnected(now, self._generation)
        device.close()
        self._notify_removed(device)

    def _notify_removed(self, device: SdlDevice) -> None:
        if self._on_removed is not None:
            try:
                self._on_removed(device)
            except Exception:
                pass


# --------------------------------------------------------------------------
# Initialisation
# --------------------------------------------------------------------------


def _init_sdl() -> None:
    """Bring SDL up in the only configuration that coexists with Qt on macOS."""
    # Background events are not optional here. SDL suppresses joystick updates
    # when it believes the application is unfocused, and with no SDL window it
    # never believes otherwise -- the wheel would read as dead centre forever.
    os.environ.setdefault("SDL_JOYSTICK_ALLOW_BACKGROUND_EVENTS", "1")
    # An explicit override is honoured, because someone debugging SDL itself
    # will want to point it at a real driver.
    os.environ.setdefault("SDL_VIDEODRIVER", "dummy")

    # Note what is NOT called: pygame.init(), which would initialise every
    # module including mixer and font, and pygame.display.set_mode(), which is
    # what actually creates a window and an NSApplication.
    pygame.display.init()
    pygame.joystick.init()

    try:
        pygame.event.pump()
    except pygame.error as exc:  # pragma: no cover - depends on the SDL build
        raise RuntimeError(
            "SDL's event queue is unavailable, so joystick input cannot work. "
            f"Video driver is {pygame.display.get_driver()!r}. Original error: "
            f"{exc}"
        ) from exc


def install_qt_pump(
    hub: SdlHub, parent: Any = None, *, interval_ms: int = 8
) -> Any:
    """Drive `hub.pump` from a QTimer on the Qt main thread.

    Returns the timer. **Keep the reference alive** -- a QTimer with no parent
    and no Python reference is garbage-collected and input stops.

    8 ms is deliberately faster than the 60 Hz UI tick and slower than the
    250 Hz input thread. Faster than the UI because a pump that lands on the
    same cadence as repainting inherits every repaint hitch; slower than the
    input thread because SDL's own device state only changes as fast as the HID
    reports arrive, typically 100-1000 Hz, and re-reading it more often buys
    nothing.
    """
    # Imported lazily so this module stays usable from headless tests and from
    # tools that have no Qt installed at all.
    from PySide6.QtCore import Qt, QTimer

    timer = QTimer(parent)
    timer.setTimerType(Qt.TimerType.PreciseTimer)
    timer.setInterval(interval_ms)
    timer.timeout.connect(hub.pump)
    timer.start()
    return timer


def _device_guid(js: Any, name: str, num_axes: int, num_buttons: int) -> str:
    """Stable per-model identity, used as the profile key.

    SDL's GUID encodes vendor, product and version, so two identical wheels share
    one profile -- which is the behaviour people expect -- and a different wheel
    gets its own. When SDL cannot provide one, a synthesised key from the name
    and control counts is stable enough to be useful and clearly marked so it is
    never mistaken for a real GUID.
    """
    try:
        guid = str(js.get_guid())
    except Exception:
        guid = ""
    if guid and guid.strip("0"):
        return guid
    return "synth:" + name.strip().lower().replace(" ", "-") + f":{num_axes}:{num_buttons}"


def describe_devices(devices: Iterable[SdlDevice]) -> list[str]:
    """One readable line per device. For the log at startup and the device picker."""
    return [
        f"{d.name} [{d.guid}] axes={d.num_axes} buttons={d.num_buttons} hats={d.num_hats}"
        for d in devices
    ]
