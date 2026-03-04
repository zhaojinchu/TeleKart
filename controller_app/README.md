# TeleKart Controller App

This is the native operator app for driving the car over `CMU-DEVICE`.

## What It Does
- Pairs with the ESP32 over `POST /api/pair`
- Reads steering wheel + pedals through `pygame`
- Sends authenticated UDP control packets at `100 Hz`
- Receives UDP telemetry from the car
- Optionally opens an RTSP camera preview if OpenCV is installed

## Install
```bash
python3 -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
```

## Run
```bash
python3 controller.py \
  --vehicle-url http://TELEKART_IP \
  --vehicle-name telekart-01 \
  --auth-key YOUR_SHARED_KEY \
  --controller-port 4211
```

Optional camera:
```bash
python3 controller.py \
  --vehicle-url http://TELEKART_IP \
  --vehicle-name telekart-01 \
  --auth-key YOUR_SHARED_KEY \
  --controller-port 4211 \
  --camera-url rtsp://CAMERA_IP:8554/telekart
```

## Keyboard Fallback
- `W`: throttle
- `S`: brake
- `A` / `D`: steer
- `R`: reverse request
- `Space`: e-stop
- `Esc`: quit

## Reverse With Pedals
- By default, braking pedal input can assert reverse request automatically when throttle is near zero.
- `pedal_source` supports `buttons`, `axes`, and `auto` (default `buttons`).
- For button pedals, set `throttle_button` and `brake_button` (for your wheel: B7 throttle, B6 brake).
- Tune with config keys:
  - `auto_reverse_from_brake` (default `true`)
  - `reverse_from_brake_threshold` (default `0.12`)
  - `reverse_from_throttle_max` (default `0.05`)
- If your wheel has a dedicated reverse button, set `reverse_button` to that index; it takes priority.

## Input Debugging
- Enable `input_debug` to print live axis/button snapshots and detected pedal source.
- Adjust `input_debug_interval_s` if logs are too fast.
- Quick test:
```bash
python controller.py --vehicle-url http://TELEKART_IP --vehicle-name telekart-01 --auth-key YOUR_SHARED_KEY --pedal-source buttons --throttle-button 7 --brake-button 6 --input-debug
```

## Notes
- The app binds to `controller_port` locally and the ESP32 sends telemetry back to that same port.
- If you want in-window video preview, install `opencv-python` separately.
