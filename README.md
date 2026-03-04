# TeleKart

TeleKart is an ESP32-based RC car controller for a steering servo and brushless ESC, refactored for infrastructure Wi-Fi (`CMU-DEVICE`) instead of a short-range self-hosted access point.

## Architecture
- `TeleKart.ino`: main ESP32 firmware entrypoint
- `network_manager.*`: station-mode Wi-Fi, fallback setup AP, persisted config
- `control_link.*`: authenticated pairing, UDP control packets, telemetry
- `drive_control.*`: deterministic control loop and vehicle-feel model
- `web_handlers.*` + `web_page.h`: setup, calibration, status, and emergency UI
- `controller_app/`: native laptop operator app for wheel/pedals over UDP
- `camera_node/`: notes for the separate video processor

## Runtime Model
- The ESP32 joins `CMU-DEVICE` as a Wi-Fi client.
- If it cannot connect within 15 seconds, it can fall back to a temporary setup AP (`TeleKart-Setup`).
- A native laptop controller app pairs with the ESP32 over HTTP, then sends authenticated UDP control packets on port `4210`.
- The ESP32 returns telemetry over UDP to the same controller socket.
- The browser UI is no longer the primary driving surface; it is for setup, calibration, diagnostics, and bench debug.

## Hardware
- Steering servo signal: GPIO 18
- ESC signal: GPIO 19
- Recommended: power the servo from a dedicated 5V regulator/BEC, not from the ESP32 5V pin

## Build Requirements
- ESP32 Arduino core
- `ESP32Servo`
- The ESP32 core ships the other libraries used here (`WiFi`, `WebServer`, `Preferences`, `WiFiUDP`, `ESPmDNS`, `mbedtls`)

## Laptop Controller App
The operator app lives in `controller_app/`.

Typical usage:
```bash
cd controller_app
python3 -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
python3 controller.py --vehicle-url http://TELEKART_IP --vehicle-name telekart-01 --auth-key YOUR_SHARED_KEY
```

## Camera
Production video should run on a separate SBC and stream RTSP to the laptop. The control ESP32 intentionally does not host the camera pipeline.

## Arduino Compile Code
arduino-cli compile --fqbn esp32:esp32:esp32 /Users/zhaojin/Projects/TeleKart

arduino-cli board list

arduino-cli upload \
  -p /dev/cu.usbserial-XXXX \
  --fqbn esp32:esp32:esp32 \
  /Users/zhaojin/Projects/TeleKart

## Command
python controller.py --vehicle-url http://172.26.206.14 --vehicle-name telekart-01 --auth-key changeme-telekart --controller-port 4211 --pedal-source axes --throttle-axis 5 --brake-axis 4 --input-debug
