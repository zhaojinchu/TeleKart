"""Finding the car.

Two mechanisms, in a deliberate order. ``getaddrinfo("telekart.local")`` is
tried first and is the one the app depends on: on macOS it goes straight to
mDNSResponder, which is the same Bonjour resolver Finder uses, is already
warm, is cached, and gets the answer right including the IPv4/IPv6 ordering.
A hostname lookup is also what a human will type, so making it the primary path
means the documented ``nc telekart.local 4212`` and the app agree.

zeroconf is used only to populate a *browse list* -- "which cars are on this
network" -- because a hostname lookup cannot enumerate. It is never load
bearing: if the package is missing, if the browse finds nothing, or if the
service record is stale, connecting by hostname still works.
"""

from __future__ import annotations

import queue
import socket
import threading
import time
from dataclasses import dataclass, field
from typing import Any, Callable, Mapping

from telekart_protocol import MDNS_SERVICE_TYPE, TCP_SESSION_PORT

from ..core.log import get_logger

_log = get_logger(__name__)

#: The car advertises this over mDNS and it is what the docs tell people to
#: type. Configurable only because a second car needs a second name.
DEFAULT_HOSTNAME = "telekart.local"

try:  # zeroconf is a convenience, never a requirement
    from zeroconf import ServiceBrowser, Zeroconf

    _ZEROCONF_ERROR = ""
except Exception as exc:  # pragma: no cover - depends on the environment
    ServiceBrowser = None  # type: ignore[assignment,misc]
    Zeroconf = None  # type: ignore[assignment,misc]
    _ZEROCONF_ERROR = str(exc)


@dataclass(frozen=True, slots=True)
class DiscoveredCar:
    """One reachable car. ``address`` is what the sockets actually use."""

    name: str
    host: str
    address: str
    port: int = TCP_SESSION_PORT
    car_id: str = ""
    fw_version: str = ""
    source: str = "dns"  # "dns" | "mdns" | "manual"
    properties: Mapping[str, str] = field(default_factory=dict)
    last_seen: float = 0.0

    @property
    def label(self) -> str:
        if self.car_id and self.car_id != self.name:
            return f"{self.car_id} ({self.address})"
        return f"{self.name} ({self.address})"


# --------------------------------------------------------------------------
# Hostname resolution -- the primary path
# --------------------------------------------------------------------------


def resolve_addresses(host: str, *, timeout: float = 3.0) -> list[str]:
    """Resolve to IP literals, IPv4 first, within a deadline.

    ``getaddrinfo`` takes no timeout, and an mDNS lookup for a car that is
    switched off blocks for the resolver's own timeout -- five seconds on
    macOS, which is five seconds of a frozen connect dialog. Running it on a
    throwaway thread and abandoning it costs one parked daemon thread and keeps
    the UI responsive.

    IPv4 is preferred because a link-local IPv6 address needs a scope id that
    the UDP paths would have to thread through everywhere; the car is on a LAN
    and always has a v4 address.
    """
    if not host:
        return []
    result: list[str] = []
    done = threading.Event()

    def worker() -> None:
        try:
            infos = socket.getaddrinfo(host, None, proto=socket.IPPROTO_TCP)
        except OSError as exc:
            _log.debug("getaddrinfo(%s) failed: %s", host, exc)
            done.set()
            return
        v4: list[str] = []
        v6: list[str] = []
        for family, _type, _proto, _canon, sockaddr in infos:
            addr = sockaddr[0]
            if family == socket.AF_INET:
                if addr not in v4:
                    v4.append(addr)
            elif family == socket.AF_INET6 and not addr.startswith("fe80:"):
                if addr not in v6:
                    v6.append(addr)
        result.extend(v4)
        result.extend(v6)
        done.set()

    thread = threading.Thread(target=worker, name="resolve", daemon=True)
    thread.start()
    if not done.wait(timeout):
        _log.warning("resolving %s timed out after %.1fs", host, timeout)
        return []
    return result


def split_host_port(target: str, default_port: int = TCP_SESSION_PORT) -> tuple[str, int]:
    """Split ``host``, ``host:port`` or ``[v6]:port``.

    The port suffix exists because a simulator, a second car, or an SSH tunnel
    will not be on 4212, and "type an address" is the escape hatch that has to
    keep working when discovery does not.
    """
    target = target.strip()
    if not target:
        return "", default_port
    if target.startswith("["):
        end = target.find("]")
        if end > 0:
            host = target[1:end]
            rest = target[end + 1 :]
            if rest.startswith(":") and rest[1:].isdigit():
                return host, int(rest[1:])
            return host, default_port
        return target, default_port
    # A bare IPv6 literal has several colons and no port; only a single colon
    # followed by digits is a port suffix.
    if target.count(":") == 1:
        host, _, port_text = target.partition(":")
        if port_text.isdigit():
            value = int(port_text)
            if 0 < value < 65536:
                return host, value
    return target, default_port


def resolve(
    host: str = DEFAULT_HOSTNAME,
    *,
    port: int = TCP_SESSION_PORT,
    timeout: float = 3.0,
) -> DiscoveredCar | None:
    """Resolve one host, optionally with a ``:port`` suffix."""
    if not host:
        return None
    host, port = split_host_port(host, port)
    literal = _as_literal(host)
    if literal is not None:
        return DiscoveredCar(
            name=host,
            host=host,
            address=literal,
            port=port,
            source="manual",
            last_seen=time.time(),
        )
    addresses = resolve_addresses(host, timeout=timeout)
    if not addresses:
        return None
    return DiscoveredCar(
        name=host,
        host=host,
        address=addresses[0],
        port=port,
        source="dns",
        last_seen=time.time(),
    )


def _as_literal(host: str) -> str | None:
    for family in (socket.AF_INET, socket.AF_INET6):
        try:
            socket.inet_pton(family, host)
        except OSError:
            continue
        return host
    return None


# --------------------------------------------------------------------------
# mDNS browse -- the convenience path
# --------------------------------------------------------------------------


class ZeroconfBrowser:
    """Background browse of ``_telekart._tcp.local.``.

    Every failure mode here degrades to an empty list. Discovery being empty
    must never be the reason a driver cannot connect -- the connect dialog
    always offers a hostname field, and that path does not involve this class
    at all.
    """

    def __init__(
        self,
        *,
        service_type: str = MDNS_SERVICE_TYPE,
        on_change: Callable[[list[DiscoveredCar]], None] | None = None,
    ) -> None:
        self._service_type = service_type
        self._on_change = on_change
        self._lock = threading.Lock()
        self._cars: dict[str, DiscoveredCar] = {}
        self._zc: Any = None
        self._browser: Any = None
        self._pending: queue.Queue[tuple[str, str] | None] = queue.Queue()
        self._resolver: threading.Thread | None = None
        self._running = False

    @property
    def available(self) -> bool:
        return Zeroconf is not None

    def start(self) -> bool:
        """Begin browsing. False means "not available", not "error"."""
        if self._running:
            return True
        if Zeroconf is None or ServiceBrowser is None:
            _log.info("mDNS browse unavailable (%s); hostname lookup still works",
                      _ZEROCONF_ERROR or "zeroconf not installed")
            return False
        try:
            self._zc = Zeroconf()
            self._resolver = threading.Thread(
                target=self._resolve_loop, name="mdnsResolve", daemon=True
            )
            self._resolver.start()
            self._browser = ServiceBrowser(self._zc, self._service_type, self)
        except Exception as exc:  # pragma: no cover - platform dependent
            _log.warning("mDNS browse could not start: %s", exc)
            self.stop()
            return False
        self._running = True
        return True

    def stop(self) -> None:
        self._running = False
        self._pending.put(None)
        browser, self._browser = self._browser, None
        zc, self._zc = self._zc, None
        # ServiceBrowser cancels, Zeroconf closes, and the spelling has moved
        # between zeroconf releases -- try both rather than pin a version for
        # a component that is explicitly optional.
        for obj, methods in ((browser, ("cancel", "close")), (zc, ("close",))):
            if obj is None:
                continue
            for method in methods:
                fn = getattr(obj, method, None)
                if fn is None:
                    continue
                try:
                    fn()
                except Exception:  # pragma: no cover - shutdown races are not news
                    pass
                break
        resolver, self._resolver = self._resolver, None
        if resolver is not None and resolver.is_alive():
            resolver.join(timeout=1.0)

    def cars(self) -> list[DiscoveredCar]:
        with self._lock:
            return sorted(self._cars.values(), key=lambda c: c.name)

    # -- zeroconf listener protocol ----------------------------------------
    # These run on zeroconf's own thread and must return immediately: blocking
    # here stalls the whole mDNS engine, including other applications' browses
    # sharing the process. The actual resolve is queued.

    def add_service(self, zc: Any, service_type: str, name: str) -> None:
        self._pending.put((service_type, name))

    def update_service(self, zc: Any, service_type: str, name: str) -> None:
        self._pending.put((service_type, name))

    def remove_service(self, zc: Any, service_type: str, name: str) -> None:
        with self._lock:
            existed = self._cars.pop(name, None) is not None
        if existed:
            self._notify()

    def _resolve_loop(self) -> None:
        while True:
            item = self._pending.get()
            if item is None:
                return
            service_type, name = item
            zc = self._zc
            if zc is None:
                return
            try:
                info = zc.get_service_info(service_type, name, timeout=2000)
            except Exception as exc:  # pragma: no cover - network dependent
                _log.debug("mDNS resolve of %s failed: %s", name, exc)
                continue
            if info is None:
                continue
            car = _car_from_info(name, info)
            if car is None:
                continue
            with self._lock:
                self._cars[name] = car
            self._notify()

    def _notify(self) -> None:
        callback = self._on_change
        if callback is None:
            return
        try:
            callback(self.cars())
        except Exception as exc:
            _log.error("discovery listener raised: %s", exc)


def _car_from_info(name: str, info: Any) -> DiscoveredCar | None:
    try:
        addresses = list(info.parsed_addresses())
    except Exception:  # pragma: no cover - zeroconf version differences
        addresses = []
    v4 = [a for a in addresses if ":" not in a]
    address = v4[0] if v4 else (addresses[0] if addresses else "")
    if not address:
        return None
    props = _decode_properties(getattr(info, "properties", None))
    short = name.split(".", 1)[0]
    return DiscoveredCar(
        name=short,
        host=getattr(info, "server", "") or short,
        address=address,
        port=int(getattr(info, "port", TCP_SESSION_PORT) or TCP_SESSION_PORT),
        car_id=props.get("car_id", short),
        fw_version=props.get("fw", props.get("fw_version", "")),
        source="mdns",
        properties=props,
        last_seen=time.time(),
    )


def _decode_properties(raw: Any) -> dict[str, str]:
    """TXT records arrive as bytes and may be malformed. Never raise on them."""
    out: dict[str, str] = {}
    if not isinstance(raw, dict):
        return out
    for key, value in raw.items():
        try:
            k = key.decode("utf-8") if isinstance(key, bytes) else str(key)
            if value is None:
                out[k] = ""
            elif isinstance(value, bytes):
                out[k] = value.decode("utf-8", "replace")
            else:
                out[k] = str(value)
        except Exception:
            continue
    return out


def browse_once(*, timeout: float = 3.0) -> list[DiscoveredCar]:
    """Blocking one-shot browse, for the CLI and for tests."""
    browser = ZeroconfBrowser()
    if not browser.start():
        return []
    try:
        time.sleep(timeout)
        return browser.cars()
    finally:
        browser.stop()


def find_car(
    host: str = "",
    *,
    port: int = TCP_SESSION_PORT,
    timeout: float = 3.0,
    browse: bool = True,
) -> DiscoveredCar | None:
    """The connect path: pinned host, then ``telekart.local``, then mDNS.

    The order is the point. A pinned host is an explicit instruction. The
    default hostname is one cheap cached lookup. Only if both fail do we spend
    seconds waiting for a browse to converge.

    ``host`` accepts a ``:port`` suffix.
    """
    if host:
        return resolve(host, port=port, timeout=timeout)
    found = resolve(DEFAULT_HOSTNAME, port=port, timeout=timeout)
    if found is not None:
        return found
    if not browse:
        return None
    cars = browse_once(timeout=timeout)
    return cars[0] if cars else None
