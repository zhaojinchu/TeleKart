# TeleKart

TeleKart is an ESP32-based RC car controller for a steering servo and brushless ESC, using infrastructure Wi-Fi (`CMU-DEVICE`) instead of a short-range self-hosted access point.

## Architecture
- `TeleKart.ino`: main ESP32 firmware entrypoint
- `network_manager.*`: station-mode Wi-Fi, fallback setup AP, persisted config
- `control_link.*`: authenticated pairing, UDP control packets, telemetry
- `drive_control.*`: deterministic control loop and vehicle-feel model
- `rc_output.*`: servo/ESC PWM output with calibration and rate limiting
- `app_state.*`: shared runtime state, config structs, globals
- `web_handlers.*` + `web_page.h`: setup, calibration, status, and emergency UI
- `controller_app/`: native laptop operator app for wheel/pedals/keyboard over UDP
- `camera_node/`: notes for the separate video processor
- `brushless_esc_code/`: legacy standalone AP-mode firmware (not used in main build)

## Hardware
- Steering servo signal: GPIO 18
- ESC signal: GPIO 19
- Recommended: power the servo from a dedicated 5V regulator/BEC, not from the ESP32 5V pin

## Build & Flash

Requires ESP32 Arduino core and `ESP32Servo` library.

```bash
# Compile
arduino-cli compile --fqbn esp32:esp32:esp32 /Users/zhaojin/Projects/TeleKart

# List connected boards to find your port
arduino-cli board list

# Upload (replace port with your actual device)
arduino-cli upload \
  -p /dev/cu.usbserial-XXXX \
  --fqbn esp32:esp32:esp32 \
  /Users/zhaojin/Projects/TeleKart
```

## Finding the ESP32 IP Address

The ESP32 joins `CMU-DEVICE` as a Wi-Fi client. Its IP is assigned by DHCP. Three ways to find it:

**1. Serial monitor (most reliable)**
```bash
arduino-cli monitor -p /dev/cu.usbserial-XXXX -c baudrate=115200
```
On boot you'll see:
```
Arming ESC...
Vehicle name: telekart-01
Station MAC: AA:BB:CC:DD:EE:FF
Wi-Fi connected! IP: 10.0.0.42
```

**2. mDNS (macOS/Linux)**
```bash
ping telekart-01.local
```

**3. Fallback AP mode**
If the ESP32 cannot connect to Wi-Fi within 15 seconds, it creates its own network:
- SSID: `TeleKart-Setup`
- Password: `telekart-setup`
- IP: `192.168.4.1`

Connect to that network and open `http://192.168.4.1` to configure Wi-Fi credentials via the web UI.

## Running the Controller App (Laptop)

The controller app lives in `controller_app/`. It pairs with the ESP32 over HTTP, then sends authenticated UDP control packets at 100 Hz.

```bash
cd controller_app
python3 -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
```

### With keyboard only (no steering wheel needed)
```bash
python3 controller.py \
  --vehicle-url http://TELEKART_IP \
  --vehicle-name telekart-01 \
  --auth-key changeme-telekart
```
Replace `TELEKART_IP` with the actual IP from the serial monitor or mDNS (e.g. `http://10.0.0.42` or `http://telekart-01.local`).

A HUD window opens. **Click it for keyboard focus**, then use:

| Key     | Action        |
|---------|---------------|
| W       | Throttle      |
| S       | Brake         |
| A / D   | Steer left/right |
| R       | Reverse request (hold with S when stopped) |
| Space   | E-stop (latches, clear via web UI) |
| Esc     | Quit          |

### With steering wheel / pedals
```bash
python3 controller.py \
  --vehicle-url http://TELEKART_IP \
  --vehicle-name telekart-01 \
  --auth-key changeme-telekart \
  --pedal-source buttons \
  --throttle-button 7 \
  --brake-button 6
```

For axis-based pedals (e.g. Logitech G29):
```bash
python3 controller.py \
  --vehicle-url http://TELEKART_IP \
  --vehicle-name telekart-01 \
  --auth-key changeme-telekart \
  --pedal-source axes \
  --steer-axis 0 \
  --throttle-axis 2 \
  --brake-axis 5
```

### With camera preview (optional)
Add `--camera-url rtsp://CAMERA_IP:8554/telekart`. Requires `opencv-python` (`pip install opencv-python`).

## Web UI

Visit `http://TELEKART_IP` in a browser for:
- **Status**: live telemetry, session info, packet stats
- **Drive Monitor**: throttle/brake/steer bar meters and state badges
- **Network**: change Wi-Fi SSID, password, vehicle name, auth key
- **Calibration**: steering trim, center point, per-side range, no-brake mode toggle
- **Debug Drive**: HTTP-based sliders for bench testing (blocked while a controller session is active)

## Driving Behavior Notes

- **No-brake mode** is ON by default. Pressing brake (S key) does NOT actively decelerate -- the car coasts to a stop via drag. Brake input only enables reverse engagement. Toggle this off in the web UI calibration section if you want active braking.
- Throttle is shaped with an expo curve (1.8) and ramp-limited at 220%/s, so full keyboard throttle ramps up gradually.
- Steering is filtered (alpha 0.25), so A/D don't snap to full lock instantly.
- Reverse requires: come to near-stop, then hold R + S for 400ms.

## Camera

Production video runs on a separate SBC streaming RTSP to the laptop. The control ESP32 intentionally does not host the camera pipeline.

## Default Auth Key

The default shared key is `changeme-telekart`. Both the ESP32 and the controller app must use the same key. Change it via the web UI network settings.
