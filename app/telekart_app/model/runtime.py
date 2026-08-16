"""Composition root: the object graph behind ``AppModel``.

``AppModel`` is the only thing widgets talk to (``docs/INTERFACES.md`` section
9), which means it has to be able to answer "connect to this car" and "which
controller is plugged in" without a widget ever learning that a ``LinkManager``
or an ``SdlHub`` exists. This module is what it delegates to.

It is kept separate from ``app_model.py`` on purpose. ``AppModel`` remains
constructible from five bare ``LatestBox`` objects with no sockets, no SDL and
no PyAV -- the property that makes every screen testable -- and this class is
the production wiring that a test simply does not build.

Nothing here is Qt. The SDL pump needs a QTimer on the GUI thread and that one
call is made from :meth:`start`, which the application already guarantees to
run there; everything else is plain threads.
"""

from __future__ import annotations

from typing import Any

from ..config.settings import Settings
from ..core.log import get_logger
from ..input.chain import InputChain
from ..input.mapping import Action
from ..input.profile import DeviceProfile, ProfileStore, preset_keyboard
from ..input.sources import DeviceSnapshot, EMPTY_SNAPSHOT, JoystickSource, KeyboardSource
from ..input.thread import InputThread, telemetry_speed_source
from ..net.link_manager import LinkManager
from ..race.director import RaceDirector, RaceMode
from ..race.lap_source import make_lap_source
from ..storage.db import Database
from ..storage.recorder import SessionRecorder

_log = get_logger(__name__)


class AppRuntime:
    """Owns the link, the recorder, the race director and the input stack.

    Constructed once per run. ``start`` and ``stop`` are idempotent, because the
    application's shutdown is wired to ``aboutToQuit`` and Qt is entitled to
    deliver that after a window close has already torn things down.
    """

    def __init__(
        self,
        settings: Settings,
        *,
        profiles: ProfileStore | None = None,
        enable_input: bool = True,
    ) -> None:
        self._settings = settings
        self._profiles = profiles if profiles is not None else ProfileStore()
        self._enable_input = enable_input
        self._started = False

        self._db: Database | None = None
        self._recorder: SessionRecorder | None = None
        if settings.recording.enabled:
            try:
                self._db = Database()
                self._recorder = SessionRecorder(
                    self._db,
                    telemetry_hz=settings.recording.telemetry_hz,
                    record_inputs=settings.recording.inputs,
                )
            except Exception as exc:  # noqa: BLE001 - a bad disk must not stop driving
                _log.error("session recording unavailable (%s); driving is unaffected", exc)
                self._db = None
                self._recorder = None

        self.link = LinkManager(settings, recorder=self._recorder)

        self.race = RaceDirector(
            make_lap_source(settings.race.lap_source),
            mode=RaceMode.parse(settings.race.mode),
            target_laps=settings.race.target_laps,
            min_lap_time=settings.race.min_lap_time,
        )

        # Input. The chain instance belongs to the input thread from the moment
        # that thread starts; reconfiguration goes through InputThread.set_config.
        self._profile: DeviceProfile = preset_keyboard()
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
        if self._recorder is not None and self._recorder.active:
            self._recorder.stop()
        if self._db is not None:
            self._db.close()

    def _start_sdl(self) -> None:
        """Bring SDL up and hang its pump off a QTimer on this thread.

        Failure is not fatal and never a dialog: telemetry, video and tuning all
        work without a wheel, and the connect screen already says what is
        missing. A driving station that refuses to open because no controller is
        plugged in would be useless at exactly the moment somebody is setting one
        up.
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
        self._use_profile(preset_keyboard(), self._keyboard)

    def _adopt(self, device: Any) -> None:
        profile = self._profiles.resolve(
            guid=device.guid,
            name=device.name,
            num_axes=device.num_axes,
            num_buttons=device.num_buttons,
            num_hats=device.num_hats,
        )
        source = JoystickSource(device, profile.mapping)
        self._joystick = source
        self._use_profile(profile, source)
        _log.info(
            "controller %s (%d axes, %d buttons) -> profile %r",
            device.name,
            device.num_axes,
            device.num_buttons,
            profile.name or profile.profile_id,
        )

    def _use_profile(self, profile: DeviceProfile, source: Any) -> None:
        self._profile = profile
        self.input.set_config(profile.chain)
        if hasattr(source, "set_mapping"):
            source.set_mapping(profile.mapping)
        self.input.set_source(source)
        self.link.set_input_device(self.input.device_name, self.input.device_connected)

    # -- the seam AppModel forwards to --------------------------------------

    @property
    def recorder(self) -> SessionRecorder | None:
        return self._recorder

    @property
    def profile(self) -> DeviceProfile:
        return self._profile

    def set_profile(self, profile: DeviceProfile) -> None:
        """Apply a re-calibrated profile to the live device."""
        source = self._joystick if self._joystick is not None else self._keyboard
        self._use_profile(profile, source)

    @property
    def device_snapshot(self) -> DeviceSnapshot:
        """The live device state, for the calibration wizard's preview."""
        joystick = self._joystick
        if joystick is None:
            return EMPTY_SNAPSHOT
        return joystick.device.snapshot

    def keyboard_press(self, key_code: int) -> None:
        self._keyboard.press(key_code)

    def keyboard_release(self, key_code: int) -> None:
        self._keyboard.release(key_code)

    def take_input_events(self) -> tuple[Action, ...]:
        return self.input.take_events()


__all__ = ["AppRuntime"]
