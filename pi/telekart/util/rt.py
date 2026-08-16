"""Real-time tuning knobs for the control thread.

Every function here is best-effort. The firmware must run identically -- if a
little jitterier -- on a developer's Mac, in a container without ``CAP_SYS_NICE``,
and as an unprivileged user. So each call reports success as a bool, logs *why*
it failed at the level the failure deserves, and never raises. A car that
refuses to start because it could not set a scheduler priority is worse than a
car with 2 ms of extra jitter.

What actually matters here, in order of effect on the 100 Hz loop:

1. ``SCHED_FIFO`` -- stops CFS from preempting the loop for the WiFi stack.
2. ``gc.freeze()`` plus ``gc.disable()`` -- a gen-2 collection over the whole
   heap is tens of milliseconds on an A53 and lands wherever it likes.
3. ``mlockall`` -- the Pi runs with swap on by default; a major fault mid-tick
   is unbounded.
4. CPU affinity -- keeps the loop off core 0, where most IRQ work lands.
"""

from __future__ import annotations

import ctypes
import ctypes.util
import gc
import os
import platform
import sys
from dataclasses import dataclass, field
from typing import Iterable, Sequence

from ..constants import (
    CONTROL_CPU_AFFINITY,
    GC_COLLECT_PERIOD_S,
    GIL_SWITCH_INTERVAL_S,
    RT_PRIORITY_CONTROL,
)
from ..log import get_logger

_log = get_logger(__name__)

#: mlockall flags from <sys/mman.h>. Same values on every Linux ABI we target.
_MCL_CURRENT = 1
_MCL_FUTURE = 2

_libc: ctypes.CDLL | None = None
_libc_loaded = False


def is_linux() -> bool:
    return sys.platform.startswith("linux")


def _load_libc() -> ctypes.CDLL | None:
    """Resolve libc once. Loading is deferred so importing this module costs
    nothing on a machine that will never call ``mlockall``."""
    global _libc, _libc_loaded
    if _libc_loaded:
        return _libc
    _libc_loaded = True
    if not is_linux():
        return None
    name = ctypes.util.find_library("c") or "libc.so.6"
    try:
        _libc = ctypes.CDLL(name, use_errno=True)
    except OSError as exc:
        _log.warning("libc load failed; mlockall unavailable", library=name, error=str(exc))
        _libc = None
    return _libc


# --------------------------------------------------------------------------
# Scheduling
# --------------------------------------------------------------------------


def set_realtime_priority(
    priority: int = RT_PRIORITY_CONTROL, policy: str = "fifo"
) -> bool:
    """Put the *calling thread* on a real-time scheduling policy.

    Linux applies ``sched_setscheduler(0, ...)`` to the calling thread, not the
    whole process, which is exactly what we want: the control thread goes
    real-time and the asyncio thread stays on CFS where a blocking DNS lookup
    cannot lock up the machine.

    Grant the capability with ``AmbientCapabilities=CAP_SYS_NICE`` in the systemd
    unit rather than running the firmware as root.
    """
    if not is_linux() or not hasattr(os, "sched_setscheduler"):
        _log.info("real-time scheduling unavailable on this platform",
                  platform=sys.platform)
        return False

    policies = {
        "fifo": os.SCHED_FIFO,
        "rr": os.SCHED_RR,
        "other": os.SCHED_OTHER,
    }
    selected = policies.get(policy.lower())
    if selected is None:
        raise ValueError(f"unknown scheduling policy {policy!r}; expected one of {sorted(policies)}")

    lo = os.sched_get_priority_min(selected)
    hi = os.sched_get_priority_max(selected)
    clamped = min(max(priority, lo), hi)
    try:
        os.sched_setscheduler(0, selected, os.sched_param(clamped))
    except PermissionError:
        _log.warning(
            "no permission for real-time scheduling; loop jitter will be higher. "
            "Add AmbientCapabilities=CAP_SYS_NICE to the systemd unit",
            policy=policy,
            priority=clamped,
        )
        return False
    except OSError as exc:
        _log.warning("sched_setscheduler failed", policy=policy, error=str(exc))
        return False
    _log.info("real-time scheduling active", policy=policy, priority=clamped)
    return True


def current_policy_name() -> str:
    """Scheduling policy of the calling thread, for diagnostics."""
    if not is_linux() or not hasattr(os, "sched_getscheduler"):
        return "unknown"
    try:
        value = os.sched_getscheduler(0)
    except OSError:
        return "unknown"
    return {
        os.SCHED_FIFO: "fifo",
        os.SCHED_RR: "rr",
        os.SCHED_OTHER: "other",
    }.get(value, f"policy-{value}")


def set_cpu_affinity(cpus: Sequence[int] = CONTROL_CPU_AFFINITY) -> bool:
    """Pin the calling thread to ``cpus``.

    Pinning helps mostly through cache locality and by keeping the loop away
    from core 0. Requested CPUs that do not exist are dropped rather than
    treated as an error, so the same call works on a 4-core Pi and a 1-core VM.
    """
    if not hasattr(os, "sched_setaffinity"):
        _log.info("CPU affinity unavailable on this platform", platform=sys.platform)
        return False
    online = os.cpu_count() or 1
    wanted = {cpu for cpu in cpus if 0 <= cpu < online}
    if not wanted:
        _log.info("no requested CPU exists; leaving affinity alone",
                  requested=list(cpus), online=online)
        return False
    try:
        os.sched_setaffinity(0, wanted)
    except OSError as exc:
        _log.warning("sched_setaffinity failed", cpus=sorted(wanted), error=str(exc))
        return False
    _log.info("cpu affinity set", cpus=sorted(wanted))
    return True


# --------------------------------------------------------------------------
# Memory
# --------------------------------------------------------------------------


def lock_memory(future: bool = True) -> bool:
    """``mlockall`` the process so no page of it can be swapped out.

    Raspberry Pi OS enables swap by default, and a major fault inside a control
    tick is unbounded -- tens of milliseconds against a 10 ms budget. Requires
    ``CAP_IPC_LOCK`` or a raised ``RLIMIT_MEMLOCK``; the systemd unit sets
    ``LimitMEMLOCK=infinity``.
    """
    libc = _load_libc()
    if libc is None:
        return False
    flags = _MCL_CURRENT | (_MCL_FUTURE if future else 0)
    ctypes.set_errno(0)
    if libc.mlockall(flags) != 0:
        err = ctypes.get_errno()
        _log.warning(
            "mlockall failed; pages may be swapped out mid-tick. "
            "Set LimitMEMLOCK=infinity in the systemd unit",
            errno=err,
            error=os.strerror(err),
        )
        return False
    _log.info("process memory locked", mcl_future=future)
    return True


def unlock_memory() -> bool:
    libc = _load_libc()
    if libc is None:
        return False
    ctypes.set_errno(0)
    if libc.munlockall() != 0:
        err = ctypes.get_errno()
        _log.warning("munlockall failed", errno=err, error=os.strerror(err))
        return False
    return True


def set_oom_score_adj(value: int, pid: int = 0) -> bool:
    """Bias the OOM killer.

    The control process is set strongly negative and the camera process
    positive, so that when 512 MB runs out the kernel takes the video feed and
    leaves the thing that is steering a moving vehicle alone.
    """
    if not is_linux():
        return False
    value = min(max(value, -1000), 1000)
    target = "self" if pid == 0 else str(pid)
    path = f"/proc/{target}/oom_score_adj"
    try:
        with open(path, "w", encoding="ascii") as handle:
            handle.write(str(value))
    except OSError as exc:
        _log.warning("could not set oom_score_adj", path=path, value=value, error=str(exc))
        return False
    _log.info("oom_score_adj set", value=value, pid=pid or os.getpid())
    return True


# --------------------------------------------------------------------------
# Interpreter tuning
# --------------------------------------------------------------------------


def set_switch_interval(seconds: float = GIL_SWITCH_INTERVAL_S) -> bool:
    """Shorten the GIL handoff interval.

    CPython's 5 ms default is half a control period: if the asyncio thread takes
    the GIL just as a tick begins, the loop waits 5 ms for it. 1 ms costs more
    context switches and buys back most of that latency.
    """
    if seconds <= 0.0:
        raise ValueError("switch interval must be positive")
    try:
        sys.setswitchinterval(seconds)
    except (ValueError, AttributeError) as exc:
        _log.warning("could not set GIL switch interval", error=str(exc))
        return False
    _log.info("gil switch interval set", seconds=seconds)
    return True


class GcController:
    """Takes the garbage collector out of the control loop's way.

    ``gc.freeze()`` moves everything allocated during startup -- modules,
    classes, the config -- into a permanent generation the collector never
    examines again. ``gc.disable()`` then stops automatic collection entirely,
    so the only remaining pauses are the ones this class triggers deliberately.

    Reference counting still reclaims almost everything; disabling the cycle
    collector only defers *cycles*. The firmware allocates nothing cyclic on a
    steady-state tick, so :meth:`maybe_collect` at 30 s intervals from the
    asyncio thread is ample -- and it runs where a 20 ms pause is invisible.
    """

    __slots__ = ("_freeze", "_disable", "_installed", "_was_enabled", "_last_collect",
                 "collections", "last_duration")

    def __init__(self, *, freeze: bool = True, disable: bool = True) -> None:
        self._freeze = freeze
        self._disable = disable
        self._installed = False
        self._was_enabled = gc.isenabled()
        self._last_collect = 0.0
        self.collections = 0
        self.last_duration = 0.0

    def install(self) -> bool:
        """Call once, after all imports and construction are finished. Calling it
        earlier freezes less and leaves the collector scanning objects that will
        live forever anyway."""
        if self._installed:
            return True
        self._was_enabled = gc.isenabled()
        if self._freeze:
            gc.collect()
            gc.freeze()
        if self._disable:
            gc.disable()
        self._installed = True
        _log.info(
            "gc configured",
            frozen=gc.get_freeze_count() if self._freeze else 0,
            automatic=gc.isenabled(),
        )
        return True

    def maybe_collect(self, now: float, interval: float = GC_COLLECT_PERIOD_S) -> bool:
        """Manual collection hook. Call from the asyncio thread, never from the
        control thread -- the whole point is to choose when the pause happens."""
        if now - self._last_collect < interval:
            return False
        self._last_collect = now
        import time as _time  # local: keeps the hot import surface small

        started = _time.perf_counter()
        collected = gc.collect()
        self.last_duration = _time.perf_counter() - started
        self.collections += 1
        if self.last_duration > 0.020:
            _log.warning(
                "gc pause exceeded a control period",
                seconds=round(self.last_duration, 4),
                objects=collected,
            )
        return True

    def restore(self) -> None:
        """Undo :meth:`install`. Mostly for tests, which must not leave the
        collector disabled for whatever runs next."""
        if not self._installed:
            return
        gc.unfreeze()
        if self._was_enabled:
            gc.enable()
        self._installed = False

    @property
    def installed(self) -> bool:
        return self._installed


# --------------------------------------------------------------------------
# One-call setup
# --------------------------------------------------------------------------


@dataclass(frozen=True, slots=True)
class RtStatus:
    """What actually took effect. Logged at startup and surfaced in diagnostics
    so 'why is p99 bad today' is answerable without guessing."""

    realtime: bool = False
    affinity: bool = False
    memlock: bool = False
    switch_interval: bool = False
    gc_tuned: bool = False
    policy: str = "unknown"
    notes: tuple[str, ...] = field(default_factory=tuple)

    @property
    def fully_applied(self) -> bool:
        return self.realtime and self.affinity and self.memlock and self.gc_tuned

    def describe(self) -> str:
        parts = [
            f"policy={self.policy}",
            f"affinity={'yes' if self.affinity else 'no'}",
            f"memlock={'yes' if self.memlock else 'no'}",
            f"gc={'tuned' if self.gc_tuned else 'default'}",
        ]
        return " ".join(parts)


def apply_realtime(
    *,
    priority: int = RT_PRIORITY_CONTROL,
    cpus: Iterable[int] | None = CONTROL_CPU_AFFINITY,
    lock_mem: bool = True,
    gc_controller: GcController | None = None,
) -> RtStatus:
    """Apply every real-time tweak that this process is allowed to make.

    Call from inside the control thread, not from ``main`` -- the scheduler
    policy and the affinity mask are per-thread on Linux.
    """
    notes: list[str] = []
    realtime = set_realtime_priority(priority)
    if not realtime:
        notes.append("no SCHED_FIFO")

    affinity = False
    if cpus is not None:
        affinity = set_cpu_affinity(tuple(cpus))
        if not affinity:
            notes.append("no CPU affinity")

    memlock = False
    if lock_mem:
        memlock = lock_memory()
        if not memlock:
            notes.append("no mlockall")

    switch = set_switch_interval()

    gc_tuned = False
    if gc_controller is not None:
        gc_tuned = gc_controller.install()

    status = RtStatus(
        realtime=realtime,
        affinity=affinity,
        memlock=memlock,
        switch_interval=switch,
        gc_tuned=gc_tuned,
        policy=current_policy_name(),
        notes=tuple(notes),
    )
    if not status.fully_applied:
        _log.warning(
            "running with reduced real-time guarantees",
            detail=status.describe(),
            machine=platform.machine(),
        )
    return status
