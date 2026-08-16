"""Composition root: the object graph behind ``AppModel``.

``AppModel`` is the only thing widgets talk to, which means it has to answer
"connect to this car" and "which controller is plugged in" without a widget ever
learning that a ``LinkManager`` or an ``SdlHub`` exists. This module is what it
delegates to.

It is kept separate from ``app_model.py`` on purpose. ``AppModel`` remains
constructible from five bare ``LatestBox`` objects with no sockets, no SDL and
no PyAV -- the property that makes the window testable -- and this class is the
production wiring that a test simply does not build.

Nothing here is Qt. The SDL pump needs a QTimer on the GUI thread and that one
call is made from :meth:`start`, which the application already guarantees to run
there; everything else is plain threads.
"""

from __future__ import annotations

from typing import Any

from telekart_protocol import VideoCodec

from ..config import Settings
from ..core.log import get_logger
from ..input.chain import InputChain
from ..input.defaults import Profile, keyboard_profile, profile_for_device
from ..input.mapping import Action
from ..input.sources import JoystickSource, KeyboardSource
from ..input.thread import InputThread, telemetry_speed_source
from ..net.link_manager import LinkManager

_log = get_logger(__name__)


class AppRuntime:
    """Owns the link and the input stack.

    Constructed once per run. ``start`` and ``stop`` are idempotent, because the
    application's shutdown is wired to ``aboutToQuit`` and Qt is entitled to
    deliver that after a window close has already torn things down.
    """

    def __init__(
        self,
        settings: Settings,
        *,
        codec: VideoCodec = VideoCodec.H264,
        enable_input: bool = True,
    ) -> None:
        self._settings = settings
        self._enable_input = enable_input
        self._started = False

        self.link = LinkManager(settings, codec=codec)

        # Input. The chain instance belongs to the input thread from the moment
        # that thread starts; reconfiguration goes through InputThread.set_config.
        self._profile: Profile = keyboard_profile()
        self._chain = InputChain(self._profile.chain)
        self.input = InputThread(
            self.link.command_box,
            chain=self._chain,
            speed_source=telemetry_speed_source(self.link.telemetry_box),
        )
        self._keyboard = KeyboardSource(self._profile.mapping)
        self.input.set_source(self._keyboard)

        self._hub: Any = None
        self._pump: Any = None
        self._joystick: JoystickSource | None = None

    # -- lifecycle ----------------------------------------------------------

    def start(self) -> None:
        """Start the input thread and, if possible, SDL. Call on the GUI thread."""
        if self._started:
            return
        self._started = True
        if self._enable_input:
            self._start_sdl()
            self.input.start()
        self.link.set_input_device(self.input.device_name, self.input.device_connected)

    def stop(self) -> None:
        if not self._started:
            return
        self._started = False
        self.input.stop()
        if self._pump is not None:
            self._pump.stop()
            self._pump = None
        if self._hub is not None:
            try:
                self._hub.stop()
            except Exception:  # noqa: BLE001 - SDL teardown is best-effort
                _log.debug("SDL teardown raised", exc_info=True)
            self._hub = None
        self.link.close()

    def _start_sdl(self) -> None:
        """Bring SDL up and hang its pump off a QTimer on this thread.

        Failure is not fatal and never a dialog: video and telemetry work
        without a wheel, and the keyboard drives the car perfectly well. A
        driving station that refuses to open because no controller is plugged in
        would be useless at exactly the moment somebody is setting one up.
        """
        try:
            from ..input.sdl_backend import SdlHub, install_qt_pump
        except Exception as exc:  # noqa: BLE001 - pygame-ce may be absent
            _log.warning("no SDL backend (%s); keyboard input only", exc)
            return
        try:
            hub = SdlHub(on_added=self._on_device_added, on_removed=self._on_device_removed)
            hub.start()
        except Exception as exc:  # noqa: BLE001 - a headless SDL build raises broadly
            _log.warning("SDL did not start (%s); keyboard input only", exc)
            return
        self._hub = hub
        # Keep the returned timer alive. A QTimer that goes out of scope is
        # garbage-collected and SDL simply stops being pumped, which looks
        # exactly like a wheel that stopped working.
        self._pump = install_qt_pump(hub)
        device = hub.first_device()
        if device is not None:
            self._adopt(device)
        else:
            _log.info("no controller found; the keyboard is the active input source")

    # -- devices ------------------------------------------------------------

    def _on_device_added(self, device: Any) -> None:
        if self._joystick is None:
            self._adopt(device)

    def _on_device_removed(self, device: Any) -> None:
        joystick = self._joystick
        if joystick is None or joystick.source_id != device.guid:
            return
        _log.warning("controller %s disconnected; falling back to the keyboard", device.name)
        self._joystick = None
        self._use_profile(keyboard_profile(), self._keyboard)

    def _adopt(self, device: Any) -> None:
        profile = profile_for_device(
            name=device.name,
            num_axes=device.num_axes,
            num_buttons=device.num_buttons,
            axes=self._settings.axes,
        )
        source = JoystickSource(device, profile.mapping)
        self._joystick = source
        self._use_profile(profile, source)
        _log.info(
            "controller %s (%d axes, %d buttons) -> %s pedals",
            device.name,
            device.num_axes,
            device.num_buttons,
            "digital" if profile.digital_pedals else "analog",
        )

    def _use_profile(self, profile: Profile, source: Any) -> None:
        self._profile = profile
        self.input.set_config(profile.chain)
        if hasattr(source, "set_mapping"):
            source.set_mapping(profile.mapping)
        self.input.set_source(source)
        self.link.set_input_device(self.input.device_name, self.input.device_connected)

    # -- the seam AppModel forwards to --------------------------------------

    @property
    def profile(self) -> Profile:
        return self._profile

    @property
    def device_name(self) -> str:
        return self.input.device_name

    def bound_keys(self) -> frozenset[int]:
        """Qt key codes the keyboard profile binds, for the window's filter."""
        return frozenset(self._keyboard.bound_keys())

    def keyboard_press(self, key_code: int) -> None:
        self._keyboard.press(key_code)

    def keyboard_release(self, key_code: int) -> None:
        self._keyboard.release(key_code)

    def keyboard_release_all(self) -> None:
        self._keyboard.release_all()

    def take_input_events(self) -> tuple[Action, ...]:
        return self.input.take_events()


__all__ = ["AppRuntime"]
