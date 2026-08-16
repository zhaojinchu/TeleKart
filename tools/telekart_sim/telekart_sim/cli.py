"""Command line for the simulator.

Every flag either injects a specific failure or pins a specific number. There is
no "make it interesting" knob, because a simulator whose behaviour cannot be
stated exactly is not a test fixture -- it is a demo.
"""

from __future__ import annotations

import argparse
import json
import logging
import math
import signal
import sys
from collections.abc import Sequence
from types import FrameType
from typing import Any

from telekart_protocol import (
    PROTO_VERSION,
    TCP_SESSION_PORT,
    TCP_VIDEO_PORT,
    UDP_CONTROL_PORT,
)
from telekart_protocol.params import ParamError, coerce_all

from .autodrive import TRACKS_DIR, PurePursuitDriver, Track, TrackError
from .physics import PlantParams, SimFaultOptions, VehicleSim
from .transport import NetworkOptions, ServerOptions, SimServer, detect_local_ip
from .video_gen import EncoderUnavailable, VideoConfig

LOG = logging.getLogger("telekart.sim")


# --------------------------------------------------------------------------
# argparse types
# --------------------------------------------------------------------------


def fraction(text: str) -> float:
    """Accept `3%` or `0.03`. Both mean the same thing; people type both."""
    cleaned = text.strip()
    try:
        if cleaned.endswith("%"):
            return float(cleaned[:-1]) / 100.0
        return float(cleaned)
    except ValueError as exc:
        raise argparse.ArgumentTypeError(
            f"{text!r} is not a number or a percentage (try 0.03 or 3%%)"
        ) from exc


def signed_fraction(text: str) -> float:
    value = fraction(text)
    if abs(value) > 0.5:
        raise argparse.ArgumentTypeError(
            f"{text!r} is more than 50 %; that is not a calibration error, it is a different car"
        )
    return value


def unit_fraction(text: str) -> float:
    value = fraction(text)
    if not 0.0 <= value <= 1.0:
        raise argparse.ArgumentTypeError(
            f"{text!r} must be between 0 and 1 (or 0% and 100%)"
        )
    return value


def stall_spec(text: str) -> tuple[float, float]:
    """`2` means a 2 s stall every 30 s; `2@15` means every 15 s."""
    duration, _, period = text.partition("@")
    try:
        seconds = float(duration)
        interval = float(period) if period else 30.0
    except ValueError as exc:
        raise argparse.ArgumentTypeError(
            f"{text!r} is not DURATION or DURATION@PERIOD in seconds"
        ) from exc
    if seconds < 0.0 or interval <= 0.0:
        raise argparse.ArgumentTypeError("stall duration and period must be positive")
    if seconds >= interval:
        raise argparse.ArgumentTypeError(
            f"a {seconds:g} s stall every {interval:g} s never resumes"
        )
    return (seconds, interval)


def non_negative(text: str) -> float:
    try:
        value = float(text)
    except ValueError as exc:
        raise argparse.ArgumentTypeError(f"{text!r} is not a number") from exc
    if value < 0.0:
        raise argparse.ArgumentTypeError(f"{text!r} must not be negative")
    return value


def video_size(text: str) -> tuple[int, int]:
    width, _, height = text.lower().partition("x")
    try:
        parsed = (int(width), int(height))
    except ValueError as exc:
        raise argparse.ArgumentTypeError(
            f"{text!r} is not WIDTHxHEIGHT, e.g. 640x480"
        ) from exc
    if parsed[0] % 2 or parsed[1] % 2:
        raise argparse.ArgumentTypeError("H.264 needs even dimensions")
    return parsed


def param_assignment(text: str) -> tuple[str, Any]:
    name, sep, raw = text.partition("=")
    if not sep:
        raise argparse.ArgumentTypeError(f"{text!r} must be NAME=VALUE")
    try:
        value = json.loads(raw)
    except json.JSONDecodeError:
        value = raw  # a bare string, for enum parameters like video_codec=h264
    return (name.strip(), value)


# --------------------------------------------------------------------------
# Parser
# --------------------------------------------------------------------------


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        prog="telekart-sim",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        description=(
            "A protocol-identical TeleKart car: the same UDP ports, the same TCP/JSON\n"
            "session, the same mDNS service and the same framed H.264 as the real one."
        ),
        epilog=(
            "examples:\n"
            "  telekart-sim                                  drive it from the desktop app\n"
            "  telekart-sim --autodrive --track figure8       reference laps, no human\n"
            "  telekart-sim --seed 7 --realtime 0 --duration 60\n"
            "                                                deterministic CI run, fast\n"
            "  telekart-sim --packet-loss 2% --latency 60 --jitter 25\n"
            "                                                a bad afternoon on 2.4 GHz\n"
            "  telekart-sim --wheelspin 0.8 --autodrive       develop traction control\n"
        ),
    )

    world = parser.add_argument_group("world")
    world.add_argument(
        "--track",
        default="oval",
        metavar="NAME|PATH",
        help="track to load: a name from tools/telekart_sim/tracks/ or a path (default: oval)",
    )
    world.add_argument(
        "--list-tracks", action="store_true", help="list the built-in tracks and exit"
    )
    world.add_argument(
        "--autodrive",
        action="store_true",
        help="a pure-pursuit virtual operator drives the track; incoming control packets "
        "are then ignored",
    )
    world.add_argument(
        "--seed",
        type=int,
        default=1,
        help="seed for every random stream: encoder noise, tyre asymmetry, packet loss "
        "(default: 1)",
    )
    world.add_argument(
        "--realtime",
        type=non_negative,
        default=1.0,
        metavar="RATE",
        help="time scale: 1 = wall clock, 2 = twice as fast, 0 = unpaced. 0 is for CI; "
        "a live driver will trip the failsafe because simulated time runs far ahead "
        "of the real link (default: 1)",
    )
    world.add_argument(
        "--duration",
        type=non_negative,
        default=0.0,
        metavar="SECONDS",
        help="stop after this much SIMULATED time; 0 runs until interrupted (default: 0)",
    )

    plant = parser.add_argument_group("vehicle behaviour")
    plant.add_argument(
        "--wheelspin",
        type=unit_fraction,
        default=0.15,
        metavar="0..1",
        help="grip: 0 is dry tarmac, 1 is ice. Wheelspin is what traction control and "
        "the slip indicator are developed against (default: 0.15)",
    )
    plant.add_argument(
        "--track-width-error",
        type=signed_fraction,
        default=0.0,
        metavar="FRACTION|PCT",
        help="systematic odometry error: the car believes its track width is off by this "
        "much, e.g. 3%% (default: 0)",
    )
    plant.add_argument(
        "--encoder-fault",
        choices=("left", "right"),
        default="",
        help="sever one encoder's channel A: counts freeze and ENCODER_FAIL_* is raised",
    )
    plant.add_argument(
        "--battery-drain",
        type=non_negative,
        default=0.0,
        metavar="V_PER_MIN",
        help="open-circuit pack decay, volts per minute, for exercising LOW_BATTERY and "
        "CRITICAL_BATTERY (default: 0)",
    )
    plant.add_argument(
        "--reject-arm",
        action="store_true",
        help="refuse every ARM with not_allowed_in_state, to exercise the app's error path",
    )
    plant.add_argument(
        "--param",
        type=param_assignment,
        action="append",
        default=[],
        metavar="NAME=VALUE",
        help="override a vehicle parameter at startup; repeatable (e.g. --param max_duty=0.5)",
    )

    net = parser.add_argument_group("link impairment")
    net.add_argument(
        "--packet-loss",
        type=unit_fraction,
        default=0.0,
        metavar="FRACTION|PCT",
        help="per-datagram loss, applied in both directions (default: 0)",
    )
    net.add_argument(
        "--latency",
        type=non_negative,
        default=0.0,
        metavar="MS",
        help="one-way delay in milliseconds; round-trip is roughly twice this (default: 0)",
    )
    net.add_argument(
        "--jitter",
        type=non_negative,
        default=0.0,
        metavar="MS",
        help="uniform +/- jitter added per datagram. Enough of it reorders packets, which "
        "the strictly-increasing sequence rule then drops (default: 0)",
    )
    net.add_argument(
        "--tcp-drop",
        type=non_negative,
        default=0.0,
        metavar="SECONDS",
        help="sever the session connection this often, with an RST; 0 never does "
        "(default: 0)",
    )
    net.add_argument(
        "--video-stall",
        type=stall_spec,
        default=(0.0, 30.0),
        metavar="SECONDS[@PERIOD]",
        help="freeze the video stream for SECONDS every PERIOD seconds (default: off)",
    )

    video = parser.add_argument_group("video")
    video.add_argument(
        "--no-video", action="store_true", help="do not serve video at all"
    )
    video.add_argument(
        "--video-size",
        type=video_size,
        default=None,
        metavar="WxH",
        help="default: 640x480",
    )
    video.add_argument("--video-fps", type=int, default=None, help="default: 30")
    video.add_argument(
        "--video-bitrate", type=int, default=None, help="bits/s, default: 2000000"
    )
    video.add_argument("--video-codec", choices=("h264", "mjpeg"), default=None)

    link = parser.add_argument_group("identity and ports")
    link.add_argument(
        "--car-id", default="telekart-sim", help="mDNS instance name and car_id"
    )
    link.add_argument(
        "--key", default="telekart", help="shared passphrase; must match the app's"
    )
    link.add_argument(
        "--bind", default="0.0.0.0", help="local address to bind (default: all)"
    )
    link.add_argument(
        "--advertise-ip",
        default="",
        help="address to publish over mDNS (default: autodetect)",
    )
    link.add_argument("--control-port", type=int, default=UDP_CONTROL_PORT)
    link.add_argument("--session-port", type=int, default=TCP_SESSION_PORT)
    link.add_argument("--video-port", type=int, default=TCP_VIDEO_PORT)
    link.add_argument(
        "--no-mdns", action="store_true", help="do not advertise over mDNS"
    )
    link.add_argument(
        "--proto-version",
        type=int,
        default=PROTO_VERSION,
        help=f"protocol version to claim and require; anything but {PROTO_VERSION} makes the "
        "app's version-mismatch path fire",
    )

    output = parser.add_argument_group("output")
    output.add_argument(
        "--log-level",
        default="info",
        choices=("debug", "info", "warning", "error"),
        help="default: info",
    )
    output.add_argument(
        "--status-interval",
        type=non_negative,
        default=5.0,
        metavar="SECONDS",
        help="how often to print a status line; 0 is silent (default: 5)",
    )
    return parser


# --------------------------------------------------------------------------
# Wiring
# --------------------------------------------------------------------------


def build_server(args: argparse.Namespace) -> SimServer:
    """Turn parsed arguments into a running-ready server. Raises on bad config."""
    track = Track.load(args.track)

    overrides: dict[str, Any] = dict(args.param)
    if args.video_size is not None:
        overrides["video_width"], overrides["video_height"] = args.video_size
    if args.video_fps is not None:
        overrides["video_fps"] = args.video_fps
    if args.video_bitrate is not None:
        overrides["video_bitrate"] = args.video_bitrate
    if args.video_codec is not None:
        overrides["video_codec"] = args.video_codec
    if overrides:
        coerce_all(overrides)  # reject early, with every problem reported at once

    vehicle = VehicleSim(
        params=overrides,
        plant=PlantParams(),
        faults=SimFaultOptions(
            encoder_fault=args.encoder_fault,
            battery_drain_v_per_min=args.battery_drain,
            track_width_error=args.track_width_error,
            wheelspin=args.wheelspin,
            reject_arm=args.reject_arm,
        ),
        seed=args.seed,
    )
    # Start on the grid with both frames agreeing, so everything that separates
    # the reported pose from the true one afterwards is accumulated drift.
    vehicle.place(track.start_x, track.start_y, track.start_heading)

    driver: PurePursuitDriver | None = None
    if args.autodrive:
        driver = PurePursuitDriver(
            track,
            wheelbase_m=float(vehicle.params["wheelbase_m"]),
            steer_max_rad=math.radians(float(vehicle.params["steer_max_deg"])),
            v_max=vehicle.v_max,
        )

    video: VideoConfig | None = None
    if not args.no_video:
        video = VideoConfig(
            width=int(vehicle.params["video_width"]),
            height=int(vehicle.params["video_height"]),
            fps=int(vehicle.params["video_fps"]),
            bitrate=int(vehicle.params["video_bitrate"]),
            gop=int(vehicle.params["video_iperiod"]),
            codec=str(vehicle.params["video_codec"]),
        )

    stall_duration, stall_period = args.video_stall
    return SimServer(
        vehicle=vehicle,
        track=track,
        driver=driver,
        options=ServerOptions(
            car_id=args.car_id,
            shared_key=args.key,
            bind_host=args.bind,
            advertise_ip=args.advertise_ip,
            control_port=args.control_port,
            session_port=args.session_port,
            video_port=args.video_port,
            proto_version=args.proto_version,
            mdns=not args.no_mdns,
            realtime=args.realtime,
            duration_s=args.duration,
            status_interval_s=args.status_interval,
            seed=args.seed,
        ),
        network=NetworkOptions(
            packet_loss=args.packet_loss,
            latency_s=args.latency / 1000.0,
            jitter_s=args.jitter / 1000.0,
            tcp_drop_s=args.tcp_drop,
            video_stall_s=stall_duration,
            video_stall_period_s=stall_period,
        ),
        video=video,
    )


def _configure_logging(level: str) -> None:
    logging.basicConfig(
        level=getattr(logging, level.upper()),
        format="%(asctime)s.%(msecs)03d %(levelname)-7s %(message)s",
        datefmt="%H:%M:%S",
    )


def _list_tracks() -> int:
    found = sorted(TRACKS_DIR.glob("*.json"))
    if not found:
        print(f"no tracks in {TRACKS_DIR}")
        return 1
    for path in found:
        try:
            track = Track.load(str(path))
        except TrackError as exc:
            print(f"{path.stem:<12} UNREADABLE: {exc}")
            continue
        print(
            f"{track.name:<12} {track.length:6.2f} m  width {track.width_m:.2f} m  "
            f"{len(track.points):4d} points"
        )
    return 0


def main(argv: Sequence[str] | None = None) -> int:
    parser = build_parser()
    args = parser.parse_args(argv)
    _configure_logging(args.log_level)

    if args.list_tracks:
        return _list_tracks()

    try:
        server = build_server(args)
    except TrackError as exc:
        parser.error(str(exc))
    except ParamError as exc:
        parser.error(f"parameter rejected: {exc}")
    except EncoderUnavailable as exc:
        # Fail at configuration time with an actionable message rather than
        # starting a car whose camera will never come up.
        print(f"telekart-sim: {exc}", file=sys.stderr)
        return 2
    except ValueError as exc:
        parser.error(str(exc))

    if args.proto_version != PROTO_VERSION:
        LOG.warning(
            "claiming protocol version %d while this build speaks %d: every app that "
            "follows the spec will refuse the handshake, which is the point",
            args.proto_version,
            PROTO_VERSION,
        )
    if args.realtime == 0.0:
        LOG.warning(
            "--realtime 0: simulated time is unpaced, so a live driver's link will look "
            "stale and the failsafe will fire. Pair it with --autodrive and --duration."
        )
    if not args.no_mdns:
        LOG.info(
            "connect the app to %s or %s:%d",
            f"{args.car_id}.local",
            args.advertise_ip or detect_local_ip(),
            args.session_port,
        )

    def _signal(signum: int, _frame: FrameType | None) -> None:
        LOG.info("signal %d: shutting down", signum)
        server.stop()

    signal.signal(signal.SIGINT, _signal)
    signal.signal(signal.SIGTERM, _signal)
    return server.run()
