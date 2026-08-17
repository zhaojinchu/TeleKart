"""Read an XInput (Xbox 360 protocol) wheel over raw USB.

macOS has no XInput driver, so this device is invisible to HID, to SDL, and to pygame --
it declares itself vendor-specific (class 0xFF, subclass 0x5D, protocol 0x01). That is
also what makes this possible: no kernel driver claims a vendor-specific interface, so
libusb can take it directly.

Worth the detour because the wheel's *other* personality (Switch Pro Controller
emulation) exposes the pedals as plain buttons, while XInput carries them as the two
trigger bytes -- 0-255 each. Analog pedal travel exists only on this side.

Wired XInput input report, 20 bytes on the interrupt IN endpoint::

    [0]     message type (0x00 = input)
    [1]     packet length (0x14)
    [2]     buttons: dpad U/D/L/R, start, back, L3, R3
    [3]     buttons: LB, RB, guide, -, A, B, X, Y
    [4]     LEFT trigger   0-255   <- brake pedal
    [5]     RIGHT trigger  0-255   <- throttle pedal
    [6:8]   left stick X   int16   <- steering
    [8:10]  left stick Y   int16
    [10:12] right stick X  int16
    [12:14] right stick Y  int16
"""

from __future__ import annotations

import struct
import threading
from dataclasses import dataclass

import usb.core
import usb.util

XINPUT_SUBCLASS = 0x5D
XINPUT_PROTOCOL = 0x01
REPORT_LEN = 20


@dataclass
class XInputState:
    """One decoded report. Sticks are -1..1, triggers 0..1."""

    lx: float = 0.0
    ly: float = 0.0
    rx: float = 0.0
    ry: float = 0.0
    lt: float = 0.0
    rt: float = 0.0
    buttons: int = 0

    def button(self, bit: int) -> bool:
        return bool(self.buttons & (1 << bit))


class XInputDevice:
    """A wheel speaking the wired Xbox 360 protocol, read straight off the endpoint."""

    def __init__(self, dev, intf, ep_in) -> None:
        self.dev = dev
        self.intf = intf
        self.ep_in = ep_in
        self.state = XInputState()
        self.error: str = ""
        self._running = False
        self._thread: threading.Thread | None = None

    @classmethod
    def find(cls) -> "XInputDevice | None":
        """Locate the first XInput interface on any attached device, or None."""
        for dev in usb.core.find(find_all=True):
            for cfg in dev:
                for intf in cfg:
                    if (
                        intf.bInterfaceSubClass != XINPUT_SUBCLASS
                        or intf.bInterfaceProtocol != XINPUT_PROTOCOL
                    ):
                        continue
                    ep_in = next(
                        (
                            e
                            for e in intf
                            if usb.util.endpoint_direction(e.bEndpointAddress)
                            == usb.util.ENDPOINT_IN
                        ),
                        None,
                    )
                    if ep_in is None:
                        continue
                    try:
                        # Only set the configuration if the device has none active.
                        # Re-setting it on a live device resets endpoint state and
                        # makes the first reads fail.
                        if dev.get_active_configuration() is None:
                            dev.set_configuration()
                    except usb.core.USBError:
                        try:
                            dev.set_configuration()
                        except usb.core.USBError:
                            pass
                    try:
                        usb.util.claim_interface(dev, intf.bInterfaceNumber)
                    except usb.core.USBError:
                        pass  # already claimed by us is fine; anything else shows on read
                    return cls(dev, intf, ep_in)
        return None

    @property
    def name(self) -> str:
        try:
            return usb.util.get_string(self.dev, self.dev.iProduct) or "XInput device"
        except usb.core.USBError:
            return "XInput device"

    def poll(self, timeout_ms: int = 4) -> XInputState | None:
        """Read one report. Returns None on timeout, which is normal and not an error.

        The endpoint only produces data when something changes, so a timeout simply
        means the wheel is being held still -- callers keep using the last state.
        """
        try:
            data = self.dev.read(self.ep_in.bEndpointAddress, REPORT_LEN, timeout_ms)
        except usb.core.USBError as exc:
            if exc.errno in (60, 110) or "timeout" in str(exc).lower():
                return None
            raise
        if len(data) < 14 or data[0] != 0x00:
            return None  # not an input report (the protocol also carries LED/rumble)

        buttons, lt, rt = struct.unpack_from("<HBB", data, 2)
        lx, ly, rx, ry = struct.unpack_from("<hhhh", data, 6)
        self.state = XInputState(
            lx=lx / 32767.0,
            ly=ly / 32767.0,
            rx=rx / 32767.0,
            ry=ry / 32767.0,
            lt=lt / 255.0,
            rt=rt / 255.0,
            buttons=buttons,
        )
        return self.state

    # -- background reader ------------------------------------------------------

    def start(self) -> "XInputDevice":
        """Read the endpoint on its own thread, keeping only the newest state.

        This has to be a thread. XInput is a *streaming* endpoint: the wheel sends a
        report every ~4 ms whether anything moved or not, so there is no such thing as
        draining it until empty -- there is always another packet coming, and each read
        blocks until it arrives. Polling it inline would hand the whole frame budget to
        the USB stack. Same reasoning as the MJPEG reader: newest value wins, and the
        control loop never blocks on a peripheral.
        """
        self._running = True
        self._thread = threading.Thread(target=self._run, name="xinput", daemon=True)
        self._thread.start()
        return self

    def _run(self) -> None:
        while self._running:
            try:
                self.poll(timeout_ms=20)
            except Exception as exc:  # noqa: BLE001 -- surfaced via .alive, never raised
                self.error = f"{type(exc).__name__}: {exc}"
                self._running = False
                return

    @property
    def alive(self) -> bool:
        """False once the reader thread has stopped, e.g. the cable was pulled."""
        return self._running

    def close(self) -> None:
        self._running = False
        if self._thread is not None:
            self._thread.join(timeout=0.5)
        try:
            usb.util.release_interface(self.dev, self.intf.bInterfaceNumber)
        except usb.core.USBError:
            pass
