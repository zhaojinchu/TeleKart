# TeleKart Camera Node

The production camera feed should run on a separate SBC, not on the motor-control ESP32.

## Recommended Baseline
- Raspberry Pi Zero 2 W or better
- CSI or USB camera
- RTSP stream over the same `CMU-DEVICE` network

## Suggested Target
- `720p`
- `30 fps`
- `H.264`
- RTSP URL example: `rtsp://CAMERA_IP:8554/telekart`

## Integration Contract
- The laptop controller app opens the RTSP stream directly.
- Camera failure must not affect the ESP32 control loop.
- The ESP32 firmware does not depend on the camera node being present.
