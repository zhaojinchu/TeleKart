"""Finding the car: resolve a name to an address.

``getaddrinfo("telekart.local")`` is the whole mechanism. On macOS it goes
straight to mDNSResponder -- the same Bonjour resolver Finder uses, already warm
and already cached -- and gets the answer right, including the IPv4/IPv6
ordering. A hostname lookup is also what a human types, so the documented
``nc telekart.local 4212`` and this app agree by construction.

The previous station also ran a zeroconf ``ServiceBrowser`` to populate a
"cars on this network" table. That table is gone, and with it the only thing a
browse could do that a lookup cannot -- enumerate. Resolution was always the
load-bearing path; browsing was a list on top of it.
"""

from __future__ import annotations

import socket
import threading
from dataclasses import dataclass

from telekart_protocol import TCP_SESSION_PORT

from ..core.log import get_logger

_log = get_logger(__name__)

#: What the car advertises and what the docs tell people to type. Configurable
#: only because a second car needs a second name.
DEFAULT_HOSTNAME = "telekart.local"


@dataclass(frozen=True, slots=True)
class Target:
    """A resolved place to connect to. ``address`` is what the sockets use."""

    host: str
    address: str
    port: int = TCP_SESSION_PORT

    @property
    def label(self) -> str:
        return self.host if self.host == self.address else f"{self.host} ({self.address})"


def resolve_addresses(host: str, *, timeout: float = 3.0) -> list[str]:
    """Resolve to IP literals, IPv4 first, within a deadline.

    ``getaddrinfo`` takes no timeout, and an mDNS lookup for a car that is
    switched off blocks for the resolver's own timeout -- five seconds on
    macOS, which is five seconds of a frozen connect panel. Running it on a
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

    The port suffix exists because a second car, a locally-run firmware process
    or an SSH tunnel will not be on 4212, and "type an address" is the escape
    hatch that has to keep working when name resolution does not.
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
) -> Target | None:
    """Resolve one host, optionally with a ``:port`` suffix. ``None`` if unknown."""
    if not host:
        return None
    host, port = split_host_port(host, port)
    literal = _as_literal(host)
    if literal is not None:
        # Already an IP: skip the resolver entirely. This is the path a locally
        # run firmware process (127.0.0.1) takes, and it must not depend on
        # mDNS being healthy.
        return Target(host=host, address=literal, port=port)
    addresses = resolve_addresses(host, timeout=timeout)
    if not addresses:
        return None
    return Target(host=host, address=addresses[0], port=port)


def _as_literal(host: str) -> str | None:
    for family in (socket.AF_INET, socket.AF_INET6):
        try:
            socket.inet_pton(family, host)
        except OSError:
            continue
        return host
    return None


__all__ = [
    "DEFAULT_HOSTNAME",
    "Target",
    "resolve",
    "resolve_addresses",
    "split_host_port",
]
