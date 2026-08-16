#!/usr/bin/env python3
"""Bring-up gate 9: prove the camera encodes in hardware, not in software.

It matters because a software fallback would eat a core, and the first symptom
would be the control loop's p99 falling apart once both processes run together
-- a long way from the camera, and easy to misdiagnose.

The gate checks whether the process holds the bcm2835 encoder device open,
which is direct evidence of the code path taken. It deliberately does *not*
gate on CPU cost: on a Zero 2 W a confirmed hardware encode at 640x480@30 still
costs ~35 % of one core in libcamera's ISP work and a Python callback thirty
times a second, and an earlier version of this script called that healthy
encode a software fallback. CPU is reported, and warned about well above the
measured normal, but it does not decide the gate.

    python pi/scripts/camera_test.py --local              # the gate
    python pi/scripts/camera_test.py --serve              # framed TCP, for the desktop
"""

from __future__ import annotations

import argparse
import contextlib
import io
import os
import resource
import socket
import sys
import time
from pathlib import Path

REPO = Path(__file__).resolve().parents[2]
if str(REPO / "packages" / "telekart_protocol") not in sys.path:
    sys.path.insert(0, str(REPO / "packages" / "telekart_protocol"))

from telekart_protocol import VideoCodec, pack_frame  # noqa: E402
from telekart_protocol.constants import TCP_VIDEO_PORT  # noqa: E402

#: Measured on a Pi Zero 2 W, 640x480@30, 2 Mbps, hardware encoder confirmed
#: open on /dev/video11: 35 % of one core. That is NOT the codec -- it is
#: libcamera's ISP work plus a Python callback thirty times a second on an A53.
#: The gate below is set to catch a software encoder (which cannot even hold
#: 30 fps here, and costs 200 %+), not to police this overhead.
CPU_WARN = 0.60

#: Discarded before measuring: libcamera/ISP bring-up is a one-off cost.
SETTLE_S = 2.0


def _hardware_encoder_open() -> bool:
    """True when this process holds the bcm2835 hardware encoder open.

    This is the real test, and it is worth preferring over any CPU threshold:
    it is direct evidence of which code path is running, whereas CPU cost varies
    with scene complexity, resolution and how busy the rest of the board is.
    An earlier version of this script gated on CPU alone and reported a
    perfectly healthy hardware encode as a software fallback.
    """
    encode_devices = set()
    try:
        for entry in Path("/sys/class/video4linux").iterdir():
            name = (entry / "name").read_text().strip()
            if "codec-encode" in name:
                encode_devices.add("/dev/" + entry.name)
    except OSError:
        return False
    if not encode_devices:
        return False
    try:
        for fd in Path("/proc/self/fd").iterdir():
            with contextlib.suppress(OSError):
                if os.readlink(fd) in encode_devices:
                    return True
    except OSError:
        return False
    return False


def _cpu_seconds() -> float:
    me = resource.getrusage(resource.RUSAGE_SELF)
    kids = resource.getrusage(resource.RUSAGE_CHILDREN)
    return me.ru_utime + me.ru_stime + kids.ru_utime + kids.ru_stime


def _output_base():
    """picamera2's Output, imported lazily so this file loads without a camera."""
    from picamera2.outputs import Output

    return Output


class _CountingOutput:
    """Counts bytes and keyframes without ever holding a frame.

    picamera2 hands buffers in on its own thread; anything slow or allocating
    here shows up as encoder backpressure rather than as an obvious bug in this
    file, so it does the minimum and nothing else.
    """

    def __init__(self) -> None:
        self.frames = 0
        self.keyframes = 0
        self.total = 0
        self.first_t = 0.0
        self.last_t = 0.0

    def outputframe(  # noqa: D102 - picamera2's callback name
        self,
        frame: bytes,
        keyframe: bool = True,
        timestamp: int | None = None,
        packet: object = None,
        audio: bool = False,
    ) -> None:
        now = time.monotonic()
        if not self.frames:
            self.first_t = now
        self.last_t = now
        self.frames += 1
        self.keyframes += int(bool(keyframe))
        self.total += len(frame)


def _build_camera(args: argparse.Namespace):
    from picamera2 import Picamera2
    from picamera2.encoders import H264Encoder, MJPEGEncoder

    picam = Picamera2()
    frame_us = int(round(1_000_000 / args.fps))
    controls = {
        # Pin the frame duration. Left to itself, auto-exposure lengthens the
        # frame in a dim room, which silently adds ~100 ms of latency while the
        # frame rate quietly collapses -- and nothing in the picture looks wrong.
        "FrameDurationLimits": (frame_us, frame_us),
    }
    if args.max_exposure_us > 0:
        # Motion blur is an exposure problem, not a resolution problem. Capping
        # exposure and letting gain rise trades noise for sharpness, which is
        # the right trade for a moving vehicle.
        controls["ExposureTime"] = args.max_exposure_us
        controls["AnalogueGain"] = args.gain

    config = picam.create_video_configuration(
        main={"size": (args.width, args.height)},
        buffer_count=4,
        queue=False,
        controls=controls,
    )
    picam.configure(config)

    if args.codec == "mjpeg":
        encoder = MJPEGEncoder(bitrate=args.bitrate)
    else:
        encoder = H264Encoder(
            bitrate=args.bitrate,
            iperiod=args.iperiod,
            repeat=True,
        )
    return picam, encoder


def run_local(args: argparse.Namespace) -> int:
    picam, encoder = _build_camera(args)
    sink_cls = type("_Sink", (_CountingOutput, _output_base()), {})
    sink = sink_cls()

    print(
        f"capturing {args.seconds:.0f}s at {args.width}x{args.height}@{args.fps} "
        f"{args.codec} target {args.bitrate / 1e6:.1f} Mbps"
    )
    picam.start_recording(encoder, sink)
    try:
        # Let libcamera finish standing up the ISP and the encoder settle before
        # the clock starts. Counting one-off initialisation against an 8-second
        # window reads as ~25 % of a core and looks exactly like a software
        # encoder, which is the wrong conclusion from the right number.
        time.sleep(SETTLE_S)
        hardware = _hardware_encoder_open()
        frames0, bytes0 = sink.frames, sink.total
        cpu0 = _cpu_seconds()
        wall0 = time.monotonic()
        time.sleep(args.seconds)
        wall = time.monotonic() - wall0
        cpu = _cpu_seconds() - cpu0
        frames = sink.frames - frames0
        encoded = sink.total - bytes0
        keyframes = sink.keyframes
    finally:
        picam.stop_recording()
        with contextlib.suppress(Exception):
            picam.close()

    if not frames:
        print("\nFAIL: no frames were produced at all")
        return 1

    fps = frames / wall
    mbps = encoded * 8 / wall / 1e6
    cores = cpu / wall
    gop = sink.frames / keyframes if keyframes else float("inf")

    print("\n--- results (steady state, after %.1fs settle) ---" % SETTLE_S)
    print(f"  frames        {frames}  (GOP ~{gop:.0f})")
    print(f"  rate          {fps:.1f} fps")
    print(f"  bitrate       {mbps:.2f} Mbps")
    print(f"  encoded       {sink.total / 1024:.0f} KiB")
    print(f"  CPU           {cores * 100:.1f}% of one core")

    print(f"  encoder       {'HARDWARE (/dev/video1x)' if hardware else 'software / unknown'}")

    ok = True
    if hardware:
        print("\nPASS: the bcm2835 hardware encoder was open during the capture")
    else:
        print(
            "\nFAIL: the hardware encoder was never opened."
            "\n      picamera2 must come from apt (python3-picamera2) and the venv"
            "\n      must have been created with --system-site-packages, or the"
            "\n      V4L2 codec device is not reachable. See docs/pi-setup.md."
        )
        ok = False

    if cores > CPU_WARN:
        print(
            f"WARN: {cores * 100:.0f}% of a core is more than expected."
            "\n      Roughly 35% is normal here and is Python callback plus ISP"
            "\n      overhead, not the codec. Well above that, check resolution"
            "\n      and frame rate before suspecting the encoder."
        )

    if fps < args.fps * 0.9:
        print(
            f"WARN: {fps:.1f} fps is short of the requested {args.fps}."
            "\n      Usually the light level -- check ExposureTime is not being"
            "\n      stretched past the frame duration."
        )
    if mbps < 0.3:
        print(
            f"NOTE: {mbps:.2f} Mbps is very low for {args.width}x{args.height}."
            "\n      A near-black picture compresses to almost nothing -- check the"
            "\n      lens cap, and point the camera at a lit scene before trusting"
            "\n      the CPU figure, which scales with how hard the picture is."
        )
    if args.codec == "h264" and keyframes < 2:
        print("WARN: almost no keyframes; a late joiner would wait to sync")

    return 0 if ok else 1


def run_serve(args: argparse.Namespace) -> int:
    """Serve framed video so the desktop app or ffplay can attach."""
    codec = VideoCodec.MJPEG if args.codec == "mjpeg" else VideoCodec.H264

    class _Sender(_output_base()):
        def __init__(self, conn: socket.socket) -> None:
            super().__init__()
            self.conn = conn
            self.seq = 0
            self.dropped = False
            self.failed = False

        def outputframe(
            self,
            frame: bytes,
            keyframe: bool = True,
            timestamp: int | None = None,
            packet: object = None,
            audio: bool = False,
        ) -> None:
            if self.failed:
                return
            self.seq += 1
            blob = pack_frame(
                self.seq,
                timestamp if timestamp is not None else time.monotonic_ns() // 1000,
                bytes(frame),
                codec=codec,
                keyframe=bool(keyframe),
                dropped_before=self.dropped,
            )
            self.dropped = False
            try:
                self.conn.sendall(blob)
            except OSError:
                # A viewer that went away is not an error worth stopping for.
                self.failed = True

    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server.bind(("0.0.0.0", args.port))
    server.listen(1)
    print(f"listening on tcp/{args.port}  (ffplay tcp://<pi>:{args.port} -fflags nobuffer)")
    print("ctrl-c to stop")

    picam, encoder = _build_camera(args)
    try:
        while True:
            conn, addr = server.accept()
            conn.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            print(f"client {addr[0]}:{addr[1]} connected")
            sender = _Sender(conn)
            picam.start_recording(encoder, sender)
            try:
                while not sender.failed:
                    time.sleep(0.2)
            finally:
                picam.stop_recording()
                with contextlib.suppress(OSError):
                    conn.close()
            print(f"client gone after {sender.seq} frames")
    except KeyboardInterrupt:
        print("\nstopping")
    finally:
        with contextlib.suppress(Exception):
            picam.close()
        server.close()
    return 0


def main(argv: list[str] | None = None) -> int:
    p = argparse.ArgumentParser(
        description="Camera bring-up: prove hardware encoding, or serve a stream.",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    mode = p.add_mutually_exclusive_group()
    mode.add_argument("--local", action="store_true", help="measure locally (the gate)")
    mode.add_argument("--serve", action="store_true", help="serve framed video over TCP")

    p.add_argument("--width", type=int, default=640)
    p.add_argument("--height", type=int, default=480)
    p.add_argument("--fps", type=int, default=30)
    p.add_argument("--bitrate", type=int, default=2_000_000, metavar="BPS")
    p.add_argument("--codec", choices=("h264", "mjpeg"), default="h264")
    p.add_argument("--iperiod", type=int, default=15, metavar="FRAMES",
                   help="keyframe interval; the hardware encoder cannot honour "
                        "on-demand keyframe requests, so this is the only "
                        "recovery mechanism after packet loss")
    p.add_argument("--seconds", type=float, default=10.0, help="--local duration")
    p.add_argument("--port", type=int, default=TCP_VIDEO_PORT, help="--serve port")
    p.add_argument("--max-exposure-us", type=int, default=8000, metavar="US",
                   help="exposure cap; 0 leaves auto-exposure alone")
    p.add_argument("--gain", type=float, default=4.0,
                   help="analogue gain used alongside the exposure cap")

    args = p.parse_args(argv)
    if not args.local and not args.serve:
        args.local = True

    try:
        import picamera2  # noqa: F401
    except Exception as exc:  # noqa: BLE001
        print(f"picamera2 is not importable: {exc}", file=sys.stderr)
        print(
            "  On the Pi it comes from apt (python3-picamera2) and the venv must\n"
            "  have been created with --system-site-packages. See docs/pi-setup.md.",
            file=sys.stderr,
        )
        return 2

    try:
        return run_serve(args) if args.serve else run_local(args)
    except KeyboardInterrupt:
        return 130


if __name__ == "__main__":
    os.environ.setdefault("LIBCAMERA_LOG_LEVELS", "*:ERROR")
    sys.exit(main())
