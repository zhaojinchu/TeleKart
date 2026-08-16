"""The 250 Hz input thread: cached axes in, a shaped command out.

The one thread in ``docs/INTERFACES.md`` section 9 that had no module. It sits
between the SDL pump (which runs on the Qt main thread, because SDL's event
queue belongs to whichever thread initialised the video subsystem) and the
100 Hz ``ControlTxThread``, and it owns exactly one thing: the
:class:`~telekart_app.input.chain.InputChain` instance.

Why 250 Hz when the wire only carries 100:

* The chain's one-euro filter and rate limiter are time-domain filters. Running
  them at 2.5x the transmit rate means the value the TX thread picks up is at
  most 4 ms old rather than a whole transmit period, and the rate limiter's
  step granularity is finer than the thing it is limiting.
* It decouples *feel* from *transmission*. A hitch in the TX thread does not
  distort the input curve, and a hitch here costs at most one repeated command
  rather than a gap in the stream the car would failsafe on.

Threading contract, which is the whole reason this file is short:

* This thread **never touches Qt and never touches a socket.** It reads a
  ``DeviceSnapshot`` reference published by the pump (one atomic attribute read)
  and writes one immutable command into a ``LatestBox``.
* ``speed_frac`` for the speed-sensitive steering assist comes from
  ``peek()`` on the telemetry box, never ``take()``. ``AppModel`` is that box's
  single consumer; taking here would steal frames from the HUD.
* Session-channel actions (ARM, DISARM, CLEAR_FAULTS, MARK_LAP…) are *queued*,
  not dispatched. The GUI thread drains them on its own tick, so a wheel button
  and a menu item end up on exactly the same code path.
"""

from __future__ import annotations

import threading
import time
from typing import Any, Callable, Protocol

from telekart_protocol import ControlFlags

from ..core.latest_box import LatestBox
from ..core.log import get_logger
from ..core.paced_loop import JitterSnapshot, PacedLoop
from ..net.control_tx import ControlCommand
from .chain import ChainConfig, ChainOutput, InputChain, NEUTRAL_OUTPUT
from .mapping import ACTION_FLAGS, ACTION_MODES, Action, ActionMode, ActionState
from .sources import NEUTRAL_SAMPLE, RawSample

_log = get_logger(__name__)

#: Section 9's number. Not a knob on the constructor by accident -- it is, and
#: the default is the documented rate.
DEFAULT_INPUT_HZ = 250.0

#: How long an edge-triggered flag (RESET_ODOM is the only one today) keeps
#: being asserted after its button was tapped.
#:
#: The problem it solves: this thread publishes at 250 Hz into a depth-1 box and
#: the TX thread reads it at 100 Hz, so three of every five commands are
#: overwritten unseen. A flag that rode exactly one command would be dropped
#: three times out of five. 60 ms guarantees at least five transmit ticks carry
#: it, which is also short enough that the car sees a pulse rather than a level.
_EDGE_FLAG_HOLD_S = 0.060

#: Bound on the queue of session actions waiting for the GUI thread. It drains
#: at 60 Hz; anything beyond a handful means the GUI is wedged, and replaying a
#: minute of stale ARM presses when it recovers would be worse than dropping.
_EVENT_QUEUE_LIMIT = 32


class InputSourceLike(Protocol):
    """What this thread needs of a source. ``sources.JoystickSource`` and
    ``sources.KeyboardSource`` both satisfy it structurally."""

    @property
    def name(self) -> str: ...

    @property
    def connected(self) -> bool: ...

    def poll(self) -> RawSample: ...


class InputThread(threading.Thread):
    """Polls a source, runs the chain, publishes one command per tick."""

    def __init__(
        self,
        command_box: LatestBox[ControlCommand],
        *,
        chain: InputChain | None = None,
        source: InputSourceLike | None = None,
        rate_hz: float = DEFAULT_INPUT_HZ,
        speed_source: Callable[[], float] | None = None,
        name: str = "Input",
    ) -> None:
        super().__init__(name=name, daemon=True)
        if rate_hz <= 0.0:
            raise ValueError(f"input rate must be positive, got {rate_hz}")
        self._commands = command_box
        self._chain = chain if chain is not None else InputChain()
        self._loop = PacedLoop(1.0 / rate_hz, name=name)
        self._speed_source = speed_source
        # See ControlTxThread: shadowing Thread._stop breaks join().
        self._shutdown = threading.Event()

        # Single-writer/single-reader reference swaps, the same discipline the
        # firmware's control mailbox uses. Do not "fix" these into locks.
        self._source: InputSourceLike | None = source
        self._actions = ActionState()
        self._events_lock = threading.Lock()
        self._events: list[Action] = []

        self._edge_flags = ControlFlags.NONE
        self._edge_until = 0.0
        self._last_output: ChainOutput = NEUTRAL_OUTPUT
        self._last_sample: RawSample = NEUTRAL_SAMPLE
        self._ticks = 0

    # -- configuration ------------------------------------------------------

    @property
    def chain(self) -> InputChain:
        return self._chain

    @property
    def source(self) -> InputSourceLike | None:
        return self._source

    @property
    def device_name(self) -> str:
        source = self._source
        return source.name if source is not None else ""

    @property
    def device_connected(self) -> bool:
        source = self._source
        return bool(source is not None and source.connected)

    def set_source(self, source: InputSourceLike | None) -> None:
        """Swap the device. Resets the chain, because carrying a half-applied
        throttle across a device change is a genuinely dangerous surprise."""
        self._source = source
        self._chain.reset()
        self._actions.reset()
        _log.info("input source -> %s", source.name if source is not None else "none")

    def set_config(self, cfg: ChainConfig) -> None:
        """Reconfigure from another thread. One reference assignment; the chain
        picks it up at the top of its next tick."""
        self._chain.pending_config = cfg

    def reset(self) -> None:
        self._chain.reset()
        self._actions.reset()

    # -- outputs ------------------------------------------------------------

    @property
    def output(self) -> ChainOutput:
        """The most recent chain result. For the calibration preview only."""
        return self._last_output

    @property
    def sample(self) -> RawSample:
        """The most recent raw poll, before any shaping."""
        return self._last_sample

    def is_latched(self, action: Action) -> bool:
        return self._actions.is_latched(action)

    def take_events(self) -> tuple[Action, ...]:
        """Drain the queued edge actions. Called from the GUI thread."""
        with self._events_lock:
            if not self._events:
                return ()
            events = tuple(self._events)
            self._events.clear()
        return events

    def snapshot(self) -> JitterSnapshot:
        return self._loop.snapshot()

    @property
    def ticks(self) -> int:
        return self._ticks

    # -- loop ---------------------------------------------------------------

    def stop(self) -> None:
        self._shutdown.set()

    def close(self) -> None:
        self.stop()

    def run(self) -> None:
        self._loop.start()
        last = time.perf_counter()
        while not self._shutdown.is_set():
            now = time.perf_counter()
            dt = now - last
            last = now
            try:
                self._tick(now, dt)
            except Exception:
                # A control path must not die. Publishing nothing this tick
                # leaves the TX thread on the previous command, which the car's
                # own 200 ms timeout already covers.
                _log.exception("input tick failed")
            if self._loop.sleep_until_next(self._shutdown):
                break
        _log.info("input thread exiting after %d ticks", self._ticks)

    def _tick(self, now: float, dt: float) -> None:
        self._ticks += 1
        source = self._source
        sample = source.poll() if source is not None else NEUTRAL_SAMPLE
        self._last_sample = sample

        speed_frac = 0.0
        if self._speed_source is not None:
            speed_frac = self._speed_source()

        output = self._chain.update_from(sample, dt, speed_frac=speed_frac)
        self._last_output = output

        self._actions.update(sample.held, sample.pressed)
        flags = self._actions.control_flags()
        # take_events() empties the pending set, so the edge flags that rode
        # this packet have to be latched here or they vanish. See the constant.
        pending = self._actions.take_events()
        if pending:
            self._queue_events(pending)
            edge = flags & (ControlFlags.RESET_ODOM)
            if edge:
                self._edge_flags = edge
                self._edge_until = now + _EDGE_FLAG_HOLD_S
        if self._edge_until > now:
            flags |= self._edge_flags
        elif self._edge_flags:
            self._edge_flags = ControlFlags.NONE

        if output.input_lost:
            # The device vanished mid-drive. The chain has already cut throttle
            # and applied its disconnect brake; nothing else to add, but the
            # latched toggles are no longer backed by a physical control.
            self._actions.reset()

        self._commands.put(
            ControlCommand(
                steering=output.steer,
                throttle=output.throttle,
                brake=output.brake,
                flags=flags,
                t_read=sample.t if sample.t > 0.0 else now,
            )
        )

    def _queue_events(self, pending: int) -> None:
        """Queue the edge actions the GUI thread owns.

        Only actions in EDGE mode reach here, and only the ones that are not
        already carried by a control flag: a flag on the wire is the car's
        business, an event is the app's.
        """
        queued: list[Action] = []
        for action, mode in ACTION_MODES.items():
            if mode is not ActionMode.EDGE or action in ACTION_FLAGS:
                # RESET_ODOM is the only edge action with a wire flag. It rides
                # the control packet; dispatching it through the session channel
                # as well would reset the odometry twice for one button press.
                continue
            if pending & (1 << int(action)):
                queued.append(action)
        if not queued:
            return
        with self._events_lock:
            if len(self._events) >= _EVENT_QUEUE_LIMIT:
                _log.warning("input event queue full; the GUI thread is not draining")
                return
            self._events.extend(queued)


def telemetry_speed_source(box: LatestBox[Any]) -> Callable[[], float]:
    """A ``speed_frac`` reader over the telemetry box.

    ``peek``, never ``take``: ``AppModel`` is the single consumer of that box
    and this must not steal an update from the HUD. Returns 0.0 whenever the
    car has not published a measured ``v_max`` yet, which disables the assist
    rather than scaling against a guess -- nothing in this codebase hardcodes a
    top speed.
    """

    def read() -> float:
        sample = box.peek()
        if sample is None:
            return 0.0
        packet = getattr(sample, "packet", sample)
        try:
            return float(packet.speed_fraction)
        except (AttributeError, TypeError, ValueError):
            return 0.0

    return read


__all__ = [
    "DEFAULT_INPUT_HZ",
    "InputSourceLike",
    "InputThread",
    "telemetry_speed_source",
]
