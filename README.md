# TeleKart

ESP32 Wi-Fi RC car controller for a brushless ESC and steering servo.

The main modular build is the root sketch:
- `TeleKart.ino`
- `app_state.*`
- `config.h`
- `rc_output.*`
- `web_handlers.*`
- `web_page.h`

This build starts an access point named `ESP32-CAR`, serves a browser-based control page, and drives:
- Steering servo on GPIO 18
- ESC signal on GPIO 19

Other sketches in this repo:
- `with_steering_wheel/with_steering_wheel.ino`: standalone "all-in-one" version with steering wheel / pedal support.
- `brushless_esc_code/brushless_esc_code.ino`: older simpler standalone version.

How it works:
- The ESP32 creates a local Wi-Fi network.
- A browser connects to `http://192.168.4.1/`.
- The web UI sends throttle / steering commands to `/cmd`.
- A failsafe stops the car if commands stop arriving for 700 ms.

Notes:
- This repo expects the ESP32 Arduino core plus `ESP32Servo`.
- ESC arming includes a 3 second delay during boot.
- Calibrate steering center and ESC limits before driving.
