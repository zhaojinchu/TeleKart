#!/usr/bin/env python3

import argparse
import hmac
import hashlib
import json
import random
import socket
import struct
import threading
import time
import urllib.error
import urllib.parse
import urllib.request
from dataclasses import dataclass
from typing import Optional

import pygame

try:
    import cv2  # type: ignore
except ImportError:  # pragma: no cover - optional dependency
    cv2 = None


CONTROL_PACKET = struct.Struct("<HIIIhhhBBI")
TELEMETRY_PACKET = struct.Struct("<HIIIhhhHHHHB")
CONTROL_VERSION = 1

FLAG_ESTOP = 1 << 0
FLAG_REVERSE = 1 << 1


@dataclass
class ControllerConfig:
    vehicle_url: str = "http://telekart-01.local"
    vehicle_name: str = "telekart-01"
    auth_key: str = "changeme-telekart"
    controller_port: int = 4211
    control_port: int = 4210
    camera_url: str = ""
    steer_axis: int = 0
    throttle_axis: int = 2
    brake_axis: int = 5
    steer_deadzone: float = 0.05
    pedal_deadzone: float = 0.03
    steer_filter_alpha: float = 0.25
    invert_throttle: bool = False
    invert_brake: bool = False
    reverse_button: int = -1
    estop_button: int = -1


class VideoWorker(threading.Thread):
    def __init__(self, url: str) -> None:
        super().__init__(daemon=True)
        self.url = url
        self._running = True

    def stop(self) -> None:
        self._running = False

    def run(self) -> None:  # pragma: no cover - UI side effect
        if not self.url:
            return
        if cv2 is None:
            print("Video requested but OpenCV is not installed; skipping video preview.")
            return

        cap = cv2.VideoCapture(self.url)
        if not cap.isOpened():
            print(f"Failed to open camera stream: {self.url}")
            return

        while self._running:
            ok, frame = cap.read()
            if not ok:
                time.sleep(0.1)
                continue
            cv2.imshow("TeleKart Camera", frame)
            if cv2.waitKey(1) & 0xFF == ord("q"):
                self._running = False
                break

        cap.release()
        cv2.destroyAllWindows()


def load_config(path: Optional[str]) -> ControllerConfig:
    config = ControllerConfig()
    if not path:
        return config

    with open(path, "r", encoding="utf-8") as handle:
        data = json.load(handle)

    for key, value in data.items():
        if hasattr(config, key):
            setattr(config, key, value)

    return config


def apply_cli_overrides(config: ControllerConfig, args: argparse.Namespace) -> ControllerConfig:
    for key in vars(config).keys():
        value = getattr(args, key, None)
        if value is not None:
            setattr(config, key, value)
    return config


def truncated_hmac(key: str, payload: bytes) -> int:
    digest = hmac.new(key.encode("utf-8"), payload, hashlib.sha256).digest()
    return int.from_bytes(digest[:4], byteorder="big", signed=False)


def pair_with_vehicle(config: ControllerConfig, sock: socket.socket) -> tuple[str, int]:
    parsed = urllib.parse.urlparse(config.vehicle_url)
    if not parsed.scheme or not parsed.netloc:
        raise RuntimeError("vehicle_url must include scheme and host, e.g. http://10.0.0.42")

    host = parsed.hostname
    if not host:
        raise RuntimeError("Unable to determine vehicle host")

    sock.bind(("", config.controller_port))
    sock.setblocking(False)

    timestamp_ms = int(time.time() * 1000) & 0xFFFFFFFF
    nonce = random.getrandbits(32)
    pair_payload = f"{config.vehicle_name}|{timestamp_ms}|{nonce}|{config.controller_port}".encode("utf-8")
    mac_tag = truncated_hmac(config.auth_key, pair_payload)

    body = json.dumps(
        {
            "vehicle_name": config.vehicle_name,
            "timestamp": timestamp_ms,
            "controller_nonce": nonce,
            "controller_port": config.controller_port,
            "mac_tag": mac_tag,
        }
    ).encode("utf-8")

    request = urllib.request.Request(
        urllib.parse.urljoin(config.vehicle_url, "/api/pair"),
        data=body,
        headers={"Content-Type": "application/json"},
        method="POST",
    )

    try:
        with urllib.request.urlopen(request, timeout=3) as response:
            reply = json.loads(response.read().decode("utf-8"))
    except urllib.error.HTTPError as error:
        details = error.read().decode("utf-8", errors="replace")
        raise RuntimeError(f"Pairing failed: {error.code} {details}") from error
    except urllib.error.URLError as error:
        raise RuntimeError(f"Pairing failed: {error.reason}") from error

    if not reply.get("ok"):
        raise RuntimeError(f"Pairing rejected: {reply}")

    session_id = int(reply["session_id"])
    print(f"Paired with {host} as session {session_id}")
    return host, session_id


def axis_to_unit(value: float, deadzone: float, invert: bool = False) -> float:
    if value != value:
        return 0.0

    if 0.0 <= value <= 1.0:
        unit = value
    else:
        unit = (value + 1.0) * 0.5

    unit = max(0.0, min(1.0, unit))
    if invert:
        unit = 1.0 - unit

    if unit <= deadzone:
        return 0.0

    scaled = (unit - deadzone) / (1.0 - deadzone)
    return max(0.0, min(1.0, scaled))


def signed_axis(value: float, deadzone: float) -> float:
    if value != value:
        return 0.0
    if abs(value) < deadzone:
        return 0.0
    return max(-1.0, min(1.0, value))


def print_telemetry(packet: tuple[int, ...]) -> None:
    (
        _version,
        session_id,
        seq,
        _device_time,
        current_steer,
        current_throttle,
        speed_pct,
        packet_age_ms,
        wifi_rssi,
        failsafe_count,
        dropped_packet_count,
        flags,
    ) = packet

    print(
        "telemetry",
        {
            "session": session_id,
            "seq": seq,
            "steer_pct": current_steer / 10.0,
            "throttle_pct": current_throttle / 10.0,
            "speed_pct": speed_pct / 10.0,
            "packet_age_ms": packet_age_ms,
            "wifi_rssi_dbm": -wifi_rssi,
            "failsafe_count": failsafe_count,
            "dropped_packets": dropped_packet_count,
            "flags": flags,
        },
    )


def main() -> int:
    parser = argparse.ArgumentParser(description="Native TeleKart operator app")
    parser.add_argument("--config", help="Optional JSON config file")

    defaults = ControllerConfig()
    for field_name, default_value in vars(defaults).items():
        argument = "--" + field_name.replace("_", "-")
        if isinstance(default_value, bool):
            parser.add_argument(argument, dest=field_name, action="store_true", default=None)
        elif isinstance(default_value, int):
            parser.add_argument(argument, dest=field_name, type=int)
        elif isinstance(default_value, float):
            parser.add_argument(argument, dest=field_name, type=float)
        else:
            parser.add_argument(argument, dest=field_name)

    args = parser.parse_args()

    config = load_config(args.config)
    config = apply_cli_overrides(config, args)

    pygame.init()
    pygame.joystick.init()
    clock = pygame.time.Clock()

    joystick = None
    if pygame.joystick.get_count() > 0:
        joystick = pygame.joystick.Joystick(0)
        joystick.init()
        print(f"Using joystick: {joystick.get_name()}")
    else:
        print("No joystick detected; keyboard fallback active (W/S throttle, A/D steer, R reverse, Space e-stop).")

    key_state = {
        "w": False,
        "a": False,
        "s": False,
        "d": False,
        "r": False,
        "space": False,
    }

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)

    host, session_id = pair_with_vehicle(config, sock)

    video_worker = VideoWorker(config.camera_url)
    if config.camera_url:
        video_worker.start()

    last_send = 0.0
    last_print = 0.0
    sequence = 0
    filtered_steer = 0.0

    try:
        while True:
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    return 0
                if event.type == pygame.KEYDOWN:
                    if event.key == pygame.K_ESCAPE:
                        return 0
                    if event.key == pygame.K_w:
                        key_state["w"] = True
                    elif event.key == pygame.K_a:
                        key_state["a"] = True
                    elif event.key == pygame.K_s:
                        key_state["s"] = True
                    elif event.key == pygame.K_d:
                        key_state["d"] = True
                    elif event.key == pygame.K_r:
                        key_state["r"] = True
                    elif event.key == pygame.K_SPACE:
                        key_state["space"] = True
                if event.type == pygame.KEYUP:
                    if event.key == pygame.K_w:
                        key_state["w"] = False
                    elif event.key == pygame.K_a:
                        key_state["a"] = False
                    elif event.key == pygame.K_s:
                        key_state["s"] = False
                    elif event.key == pygame.K_d:
                        key_state["d"] = False
                    elif event.key == pygame.K_r:
                        key_state["r"] = False
                    elif event.key == pygame.K_SPACE:
                        key_state["space"] = False

            raw_steer = 0.0
            throttle = 0.0
            brake = 0.0
            flags = 0

            if joystick is not None:
                raw_steer = signed_axis(joystick.get_axis(config.steer_axis), config.steer_deadzone)
                throttle = axis_to_unit(
                    joystick.get_axis(config.throttle_axis),
                    config.pedal_deadzone,
                    invert=config.invert_throttle,
                )
                brake = axis_to_unit(
                    joystick.get_axis(config.brake_axis),
                    config.pedal_deadzone,
                    invert=config.invert_brake,
                )

                if config.reverse_button >= 0 and joystick.get_button(config.reverse_button):
                    flags |= FLAG_REVERSE
                if config.estop_button >= 0 and joystick.get_button(config.estop_button):
                    flags |= FLAG_ESTOP

            if key_state["a"] and not key_state["d"]:
                raw_steer = -1.0
            elif key_state["d"] and not key_state["a"]:
                raw_steer = 1.0

            if key_state["w"]:
                throttle = 1.0
            if key_state["s"]:
                brake = 1.0
            if key_state["r"]:
                flags |= FLAG_REVERSE
            if key_state["space"]:
                flags |= FLAG_ESTOP

            filtered_steer += (raw_steer - filtered_steer) * config.steer_filter_alpha
            if abs(filtered_steer) < 0.005:
                filtered_steer = 0.0

            now = time.monotonic()
            if (now - last_send) >= 0.01:
                sequence = (sequence + 1) & 0xFFFFFFFF
                client_time_ms = int(time.time() * 1000) & 0xFFFFFFFF

                steering_cmd = max(-1000, min(1000, int(round(filtered_steer * 1000.0))))
                throttle_cmd = max(0, min(1000, int(round(throttle * 1000.0))))
                brake_cmd = max(0, min(1000, int(round(brake * 1000.0))))

                packet_without_tag = CONTROL_PACKET.pack(
                    CONTROL_VERSION,
                    session_id,
                    sequence,
                    client_time_ms,
                    steering_cmd,
                    throttle_cmd,
                    brake_cmd,
                    flags,
                    0,
                    0,
                )
                mac_tag = truncated_hmac(config.auth_key, packet_without_tag)
                packet = CONTROL_PACKET.pack(
                    CONTROL_VERSION,
                    session_id,
                    sequence,
                    client_time_ms,
                    steering_cmd,
                    throttle_cmd,
                    brake_cmd,
                    flags,
                    0,
                    mac_tag,
                )
                sock.sendto(packet, (host, config.control_port))
                last_send = now

            try:
                while True:
                    data, _addr = sock.recvfrom(256)
                    if len(data) != TELEMETRY_PACKET.size:
                        continue
                    telemetry = TELEMETRY_PACKET.unpack(data)
                    if (now - last_print) >= 0.5:
                        print_telemetry(telemetry)
                        last_print = now
            except BlockingIOError:
                pass

            clock.tick(200)
    finally:
        if config.camera_url:
            video_worker.stop()
            video_worker.join(timeout=1.0)
        sock.close()
        pygame.quit()


if __name__ == "__main__":
    raise SystemExit(main())
