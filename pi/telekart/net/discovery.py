"""mDNS advertisement of ``_telekart._tcp.local.``

Convenience, never a dependency. The desktop app can always be pointed at an IP
address typed by hand, and it must be able to: mDNS is the first thing to break
on a guest network, on a corporate WLAN with multicast filtering, or on a laptop
whose firewall blocks UDP 5353. So every failure in this module -- zeroconf not
installed, no usable address, a registration that times out -- degrades to a
no-op with one log line, and the car carries on.

zeroconf's registration does real network I/O and can block for a second or
more, which is why both ``start`` and ``stop`` push it onto a worker thread
rather than stalling the event loop that is also running the telemetry pacer.
"""

from __future__ import annotations

import asyncio
import socket
from typing import Any

from telekart_protocol.constants import (
    MDNS_SERVICE_TYPE,
    PROTO_VERSION,
    TCP_SESSION_PORT,
    TCP_VIDEO_PORT,
    UDP_CONTROL_PORT,
    UDP_TELEMETRY_PORT,
)

from ..constants import FIRMWARE_VERSION
from ..log import get_logger

_log = get_logger(__name__)

#: A well-known address that is never actually contacted. Connecting a UDP
#: socket performs no I/O -- it only asks the kernel which local interface would
#: be used -- so this reports the address the operator can actually reach us on,
#: which `gethostbyname(hostname)` frequently does not.
_ROUTE_PROBE = ("192.0.2.1", 9)

#: Registration is a handful of multicast packets. If it has not completed in
#: this long the network is dropping them and retrying will not help.
_REGISTER_TIMEOUT_S = 5.0


def local_ipv4() -> str | None:
    """Best guess at the address a laptop on this LAN would reach us on."""
    probe = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        probe.connect(_ROUTE_PROBE)
        return str(probe.getsockname()[0])
    except OSError:
        pass
    finally:
        probe.close()
    try:
        address = socket.gethostbyname(socket.gethostname())
    except OSError:
        return None
    # A loopback answer means the hostname is only in /etc/hosts; useless for
    # advertising, and worse than useless because a client would try to use it.
    return None if address.startswith("127.") else address


def _sanitize(name: str) -> str:
    """mDNS instance names may not contain a dot -- it is the label separator."""
    cleaned = "".join(ch if ch.isalnum() or ch in "-_ " else "-" for ch in name).strip()
    return cleaned or "telekart"


class Discovery:
    """Advertises the session port, or quietly does nothing."""

    def __init__(
        self,
        *,
        car_id: str,
        port: int = TCP_SESSION_PORT,
        control_port: int = UDP_CONTROL_PORT,
        telemetry_port: int = UDP_TELEMETRY_PORT,
        video_port: int = TCP_VIDEO_PORT,
        fw_version: str = FIRMWARE_VERSION,
        address: str | None = None,
        extra: dict[str, str] | None = None,
    ) -> None:
        self._car_id = car_id
        self._port = port
        self._address = address
        self._properties: dict[str, str] = {
            # TXT records are what let the app show a useful picker before it
            # has connected to anything, and what let it refuse a car speaking a
            # protocol it does not know without opening a socket first.
            "proto": str(PROTO_VERSION),
            "car_id": car_id,
            "fw": fw_version,
            "control_port": str(control_port),
            "telemetry_port": str(telemetry_port),
            "video_port": str(video_port),
        }
        if extra:
            self._properties.update({str(k): str(v) for k, v in extra.items()})

        self._zeroconf: Any = None
        self._service: Any = None
        self._active = False
        self._reason = "not started"

    # -- lifecycle ----------------------------------------------------------

    async def start(self) -> bool:
        """Register the service. Returns whether it actually got advertised."""
        if self._active:
            return True
        try:
            from zeroconf import ServiceInfo, Zeroconf
        except ImportError:
            self._reason = "zeroconf is not installed"
            _log.info(
                "mDNS advertisement disabled; connect by IP address instead",
                reason=self._reason,
            )
            return False

        address = self._address or local_ipv4()
        if address is None:
            self._reason = "no routable IPv4 address"
            _log.warning("mDNS advertisement skipped", reason=self._reason)
            return False

        hostname = socket.gethostname().split(".")[0] or "telekart"
        info = ServiceInfo(
            MDNS_SERVICE_TYPE,
            f"{_sanitize(self._car_id)}.{MDNS_SERVICE_TYPE}",
            addresses=[socket.inet_aton(address)],
            port=self._port,
            properties=dict(self._properties),
            server=f"{hostname}.local.",
        )

        def register() -> None:
            zeroconf = Zeroconf()
            try:
                zeroconf.register_service(info)
            except BaseException:
                zeroconf.close()
                raise
            self._zeroconf = zeroconf
            self._service = info

        try:
            await asyncio.wait_for(asyncio.to_thread(register), _REGISTER_TIMEOUT_S)
        except (asyncio.TimeoutError, OSError, RuntimeError) as exc:
            self._reason = f"registration failed: {exc}"
            _log.warning("mDNS advertisement failed", reason=self._reason)
            return False
        except Exception as exc:  # noqa: BLE001 - zeroconf raises its own types
            self._reason = f"registration failed: {exc!r}"
            _log.warning("mDNS advertisement failed", reason=self._reason)
            return False

        self._active = True
        self._reason = ""
        _log.info(
            "advertising over mDNS",
            service=MDNS_SERVICE_TYPE,
            name=_sanitize(self._car_id),
            address=address,
            port=self._port,
        )
        return True

    async def stop(self) -> None:
        """Withdraw the advertisement. Never raises; shutdown paths run it."""
        zeroconf, self._zeroconf = self._zeroconf, None
        service, self._service = self._service, None
        self._active = False
        if zeroconf is None:
            return

        def unregister() -> None:
            try:
                if service is not None:
                    zeroconf.unregister_service(service)
            finally:
                zeroconf.close()

        try:
            await asyncio.wait_for(asyncio.to_thread(unregister), _REGISTER_TIMEOUT_S)
        except Exception as exc:  # noqa: BLE001 - shutdown must not be blocked
            _log.debug("mDNS withdrawal failed", error=str(exc))
        else:
            _log.info("mDNS advertisement withdrawn")

    # -- introspection ------------------------------------------------------

    @property
    def active(self) -> bool:
        return self._active

    @property
    def reason(self) -> str:
        """Why the advertisement is not up. Empty when it is."""
        return self._reason

    @property
    def properties(self) -> dict[str, str]:
        return dict(self._properties)

    def __repr__(self) -> str:
        status = "active" if self._active else f"inactive ({self._reason})"
        return f"Discovery({self._car_id}, port={self._port}, {status})"


__all__ = ["Discovery", "local_ipv4"]
