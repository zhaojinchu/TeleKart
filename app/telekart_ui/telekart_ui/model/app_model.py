"""AppModel: the single object every widget talks to."""

from __future__ import annotations

import time
from dataclasses import replace
from typing import Any, Protocol

from PySide6.QtCore import QObject, Qt, QTimer, Signal

from telekart_protocol import Fault, TelemetryPacket, VehicleState

from ..config import Settings
from ..core.latest_box import LatestBox
from ..core.log import get_logger
from ..input.mapping import Action
from .snapshots import (
    EMPTY_INPUT,
    EMPTY_LINK,
    EMPTY_SESSION,
    EMPTY_VEHICLE,
    TELEMETRY_STALE_S,
    InputSnapshot,
    LinkSnapshot,
    SessionSnapshot,
    VehicleSnapshot,
    differs,
)
from .units import fault_text, iter_fault_bits

_log = get_logger(__name__)

#: One drain per display frame. Faster buys nothing -- the compositor will not
#: show it -- and slower makes the steering readout visibly lag the wheel.
DEFAULT_TICK_HZ = 60.0


class SnapshotSources(Protocol):
    """The five boxes the model drains. ``LinkManager`` satisfies this.

    Expressed as a Protocol so the entire UI can be driven from a fixture that
    is nothing but five ``LatestBox`` objects and a script -- no sockets, no
    car. That is the property that makes the window testable.
    """

    telemetry_box: LatestBox[Any]
    link_box: LatestBox[LinkSnapshot]
    input_box: LatestBox[InputSnapshot]
    session_box: LatestBox[SessionSnapshot]
    video_box: LatestBox[Any]


class VehicleController(Protocol):
    """Outbound commands. ``LinkManager`` satisfies this too."""

    def arm(self) -> None: ...
    def disarm(self) -> None: ...
    def estop(self) -> None: ...
    def clear_estop(self) -> None: ...
    def clear_faults(self) -> None: ...
    def reset_odom(self) -> None: ...


class _NullSources:
    """Empty boxes, so an AppModel with no link still ticks and still paints."""

    def __init__(self) -> None:
        self.telemetry_box: LatestBox[Any] = LatestBox()
        self.link_box: LatestBox[LinkSnapshot] = LatestBox()
        self.input_box: LatestBox[InputSnapshot] = LatestBox()
        self.session_box: LatestBox[SessionSnapshot] = LatestBox()
        self.video_box: LatestBox[Any] = LatestBox()


class AppModel(QObject):
    """Drains every LatestBox once per frame and emits only real changes.

    Two rules make the whole UI tractable:

    * **Widgets talk to nothing else.** No widget owns a socket, a thread or a
      decoder. Everything it renders arrives as an immutable snapshot and
      everything it commands goes out through a controller it never inspects.
    * **One drain per frame.** All five boxes are read in one pass, so every
      widget repainted in that frame is looking at the same instant. Emitting
      per packet instead would let the speed readout and the steering arc come
      from different packets, which reads on screen as a fault that is not there.

    Change signals are suppressed when nothing changed, with a per-field
    epsilon for floats (see ``snapshots.differs``). A parked car still reports
    encoder noise, and without the epsilon every "unchanged" packet would wake
    every subscriber fifty times a second.
    """

    vehicleChanged = Signal(VehicleSnapshot)
    linkChanged = Signal(LinkSnapshot)
    inputChanged = Signal(InputSnapshot)
    sessionChanged = Signal(SessionSnapshot)

    #: Edge-triggered, one signal per newly-set bit. Level-triggered faults are
    #: already in `vehicleChanged`; these exist so an alert fires once instead
    #: of fifty times a second for as long as the fault is latched.
    faultRaised = Signal(int, str)
    faultCleared = Signal(int, str)

    #: The car's own report, never an app-side assumption.
    stateChanged = Signal(object)

    #: Carries a ``FrameBundle``. Typed as ``object`` on purpose: naming the
    #: class here would drag PyAV and QtGui into every import of this module.
    #: The receiver must keep the bundle alive while it paints -- see
    #: video.frame for what happens if it does not.
    frameReady = Signal(object)

    #: Emitted once per drain with the current perf_counter. For animations and
    #: for anything that needs a heartbeat without subscribing to data.
    ticked = Signal(float)

    def __init__(
        self,
        sources: SnapshotSources | None = None,
        *,
        controller: VehicleController | None = None,
        settings: Settings | None = None,
        runtime: Any = None,
        codec: Any = None,
        tick_hz: float = DEFAULT_TICK_HZ,
        parent: QObject | None = None,
    ) -> None:
        super().__init__(parent)
        if tick_hz <= 0.0:
            raise ValueError(f"tick rate must be positive, got {tick_hz}")

        # Two ways to build one. A test passes boxes and gets a model with no
        # sockets at all; the application passes Settings and gets the real
        # object graph. Both end up with the same widget-facing surface, which
        # is the point -- widgets talk to this and to nothing else, so the
        # network has to live behind it either way.
        if runtime is None and settings is not None:
            from telekart_protocol import VideoCodec

            from .runtime import AppRuntime

            runtime = AppRuntime(settings, codec=codec or VideoCodec.H264)
        self._runtime = runtime
        if runtime is not None:
            sources = sources if sources is not None else runtime.link
            controller = controller if controller is not None else runtime.link

        self._settings = settings
        self._sources: SnapshotSources = sources if sources is not None else _NullSources()
        self._controller = controller
        self._pending_frame: Any = None

        self._vehicle = EMPTY_VEHICLE
        self._link = EMPTY_LINK
        self._input = EMPTY_INPUT
        self._session = EMPTY_SESSION
        self._frame: Any = None
        self._last_telemetry_t = 0.0
        self._faults = Fault.NONE

        self._timer = QTimer(self)
        # PreciseTimer: the default CoarseTimer can slip 5 % on macOS, which
        # against a 60 Hz display shows up as an occasional dropped update.
        self._timer.setTimerType(Qt.TimerType.PreciseTimer)
        self._timer.setInterval(max(1, round(1000.0 / tick_hz)))
        self._timer.timeout.connect(self.tick_once)

    # -- wiring -------------------------------------------------------------

    def set_sources(self, sources: SnapshotSources) -> None:
        self._sources = sources

    def set_controller(self, controller: VehicleController | None) -> None:
        self._controller = controller

    @property
    def controller(self) -> VehicleController | None:
        return self._controller

    def start(self) -> None:
        """Start the 60 Hz drain, and the runtime if this model owns one."""
        runtime = self._runtime
        if runtime is not None:
            runtime.start()
        self._timer.start()

    def stop(self) -> None:
        self._timer.stop()
        runtime = self._runtime
        if runtime is not None:
            runtime.stop()
        # Whatever the video box last handed over is several megabytes of
        # decoded picture. Nothing is going to paint it now.
        self._frame = None
        self._pending_frame = None

    @property
    def running(self) -> bool:
        return self._timer.isActive()

    # -- current state ------------------------------------------------------

    @property
    def vehicle(self) -> VehicleSnapshot:
        return self._vehicle

    @property
    def link(self) -> LinkSnapshot:
        return self._link

    @property
    def input(self) -> InputSnapshot:
        return self._input

    @property
    def session(self) -> SessionSnapshot:
        return self._session

    @property
    def frame(self) -> Any:
        """The most recent ``FrameBundle``, or None.

        Holding this reference is what keeps the decoded pixels alive. A widget
        that wants to paint must keep its own reference for the duration of the
        paint rather than reading this property twice.
        """
        return self._frame

    # -- the tick -----------------------------------------------------------

    def tick_once(self) -> None:
        """One drain. Public so tests can step the model with no event loop."""
        now = time.perf_counter()
        self._drain_telemetry(now)
        self._drain_link()
        self._drain_input()
        self._drain_session()
        self._drain_video()
        self._drain_input_events()
        self.ticked.emit(now)

    def _drain_input_events(self) -> None:
        """Turn wheel-button presses into the same commands the menu issues.

        The input thread queues them rather than acting: ARM is a session
        message and the session client is not something a 250 Hz thread should
        be reaching into, and routing both paths through here means a button and
        a menu item cannot diverge.
        """
        runtime = self._runtime
        if runtime is None:
            return
        for action in runtime.take_input_events():
            handler = _ACTION_COMMANDS.get(action)
            if handler is not None:
                handler(self)

    def _drain_telemetry(self, now: float) -> None:
        taken = self._sources.telemetry_box.take()
        if taken is not None:
            sample = taken[0]
            self._last_telemetry_t = getattr(sample, "recv_t", now)
            snapshot = _vehicle_from_packet(sample.packet, stale=False)
        elif self._vehicle.valid:
            stale = (
                self._last_telemetry_t > 0.0
                and now - self._last_telemetry_t > TELEMETRY_STALE_S
            )
            if stale == self._vehicle.stale:
                return
            # Everything else is still the last thing the car said; only its
            # trustworthiness changed.
            snapshot = replace(self._vehicle, stale=stale)
        else:
            return

        previous = self._vehicle
        if not differs(previous, snapshot):
            return
        self._vehicle = snapshot
        self.vehicleChanged.emit(snapshot)

        if snapshot.state is not previous.state:
            self.stateChanged.emit(snapshot.state)
        self._emit_fault_edges(snapshot.faults)

    def _emit_fault_edges(self, faults: Fault) -> None:
        if faults == self._faults:
            return
        raised = int(faults) & ~int(self._faults)
        cleared = int(self._faults) & ~int(faults)
        self._faults = faults
        for bit in iter_fault_bits(Fault(raised)):
            text = fault_text(bit)
            _log.warning("fault raised: %s", text)
            self.faultRaised.emit(bit, text)
        for bit in iter_fault_bits(Fault(cleared)):
            self.faultCleared.emit(bit, fault_text(bit))

    def _drain_link(self) -> None:
        taken = self._sources.link_box.take()
        if taken is None:
            return
        snapshot = taken[0]
        if snapshot is None or not differs(self._link, snapshot):
            return
        self._link = snapshot
        self.linkChanged.emit(snapshot)

    def _drain_input(self) -> None:
        taken = self._sources.input_box.take()
        if taken is None:
            return
        snapshot = taken[0]
        if snapshot is None or not differs(self._input, snapshot):
            return
        self._input = snapshot
        self.inputChanged.emit(snapshot)

    def _drain_session(self) -> None:
        taken = self._sources.session_box.take()
        if taken is None:
            return
        snapshot = taken[0]
        if snapshot is None or not differs(self._session, snapshot):
            return
        self._session = snapshot
        self.sessionChanged.emit(snapshot)

    def _drain_video(self) -> None:
        taken = self._sources.video_box.take()
        if taken is None:
            return
        bundle = taken[0]
        # Replacing the reference is what frees the previous frame's pixels.
        # Nothing else in the app may hold one past its repaint.
        self._frame = bundle
        # Offered to `take_frame` exactly once. The model stays the video box's
        # single consumer -- a second `take()` from the video widget would steal
        # updates from anything subscribed to frameReady.
        self._pending_frame = bundle
        if bundle is not None:
            self.frameReady.emit(bundle)

    def take_frame(self) -> Any:
        """The newest ``FrameBundle`` since the last call, or ``None``.

        Take semantics, because ``VideoView`` repaints on a non-None answer:
        peek semantics would make it redraw the same picture sixty times a
        second. The model keeps its own reference in ``self._frame``, so the
        pixels stay alive for exactly as long as the widget is holding the
        bundle it was handed.
        """
        bundle = self._pending_frame
        self._pending_frame = None
        return bundle

    # -- commands -----------------------------------------------------------
    # Thin forwarding on purpose. A widget must never learn what a session is.

    def arm(self) -> None:
        self._command("arm")

    def disarm(self) -> None:
        self._command("disarm")

    def estop(self) -> None:
        self._command("estop")

    def clear_estop(self) -> None:
        self._command("clear_estop")

    def clear_faults(self) -> None:
        self._command("clear_faults")

    def reset_odom(self) -> None:
        self._command("reset_odom")

    def _command(self, name: str) -> None:
        controller = self._controller
        if controller is None:
            _log.info("no controller: ignoring %s", name)
            return
        getattr(controller, name)()

    # -- link lifecycle -----------------------------------------------------
    # Widgets ask the model to connect; they never learn what a session is.

    def connect_to(self, host: str = "", *, driver: str = "") -> None:
        runtime = self._runtime
        if runtime is None:
            _log.info("no runtime: ignoring connect_to(%r)", host)
            return
        runtime.link.connect(host, driver=driver)

    def disconnect(self) -> None:
        runtime = self._runtime
        if runtime is not None:
            runtime.link.disconnect()

    def set_display_size(self, width: int, height: int) -> None:
        """Tell the decoder what size to reformat to. Debounced by the view."""
        runtime = self._runtime
        if runtime is not None:
            runtime.link.set_video_display_size(width, height)

    # -- input --------------------------------------------------------------

    @property
    def device_name(self) -> str:
        runtime = self._runtime
        return "" if runtime is None else runtime.device_name

    def bound_keys(self) -> frozenset[int]:
        """Qt key codes the current profile actually binds.

        The window filters on this before forwarding, so typing a passphrase
        into the connect panel cannot steer the car.
        """
        runtime = self._runtime
        return frozenset() if runtime is None else runtime.bound_keys()

    def key_pressed(self, key_code: int) -> None:
        runtime = self._runtime
        if runtime is not None:
            runtime.keyboard_press(key_code)

    def key_released(self, key_code: int) -> None:
        runtime = self._runtime
        if runtime is not None:
            runtime.keyboard_release(key_code)

    def release_all_keys(self) -> None:
        """Drop every held key. Called when the window loses focus.

        Without it, alt-tabbing with the throttle held leaves the key down: the
        window never sees the release, so the input thread keeps publishing full
        throttle and the car keeps driving. The car's own 200 ms control timeout
        does not save you here, because the app is still transmitting happily.
        """
        runtime = self._runtime
        if runtime is not None:
            runtime.keyboard_release_all()


#: Edge actions the model services on its own, so a wheel button and a keyboard
#: shortcut reach the car by the identical path.
_ACTION_COMMANDS: dict[Action, Any] = {
    Action.ARM: AppModel.arm,
    Action.DISARM: AppModel.disarm,
    Action.CLEAR_FAULTS: AppModel.clear_faults,
}


def _vehicle_from_packet(packet: TelemetryPacket, *, stale: bool) -> VehicleSnapshot:
    """Wire units to SI. The only place in the app that unpacks a telemetry packet."""
    return VehicleSnapshot(
        valid=True,
        stale=stale,
        state=packet.state,
        faults=packet.faults,
        flags=packet.flags,
        speed=packet.speed_mps,
        v_max=packet.v_max_mps,
        steer_angle=_radians(packet.steer_angle_deg),
        pack_volts=packet.pack_volts,
        cpu_temp_c=packet.cpu_temp_c,
        loop_p99=packet.loop_p99_us / 1_000_000.0,
        sequence=packet.sequence,
        car_time_us=packet.car_time_us,
    )


def _radians(degrees: float) -> float:
    return degrees * 0.017453292519943295


def state_is_drivable(state: VehicleState) -> bool:
    """Convenience for the UI. Mirrors the car's own definition, no more."""
    return state in (VehicleState.ARMED, VehicleState.FAILSAFE)


__all__ = [
    "DEFAULT_TICK_HZ",
    "AppModel",
    "SnapshotSources",
    "VehicleController",
    "state_is_drivable",
]
