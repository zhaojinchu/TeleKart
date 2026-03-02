#pragma once

#include <Arduino.h>

static const uint16_t CONTROL_PROTOCOL_VERSION = 1;

enum ControlFlags : uint8_t {
  CONTROL_FLAG_ESTOP          = 1 << 0,
  CONTROL_FLAG_REVERSE_REQ    = 1 << 1,
  CONTROL_FLAG_LIGHTS_REQ     = 1 << 2,
  CONTROL_FLAG_HORN_REQ       = 1 << 3,
};

enum TelemetryFlags : uint8_t {
  TELEMETRY_FLAG_WIFI_CONNECTED = 1 << 0,
  TELEMETRY_FLAG_FAILSAFE       = 1 << 1,
  TELEMETRY_FLAG_ESTOP          = 1 << 2,
  TELEMETRY_FLAG_REVERSE_DRIVE  = 1 << 3,
  TELEMETRY_FLAG_FALLBACK_AP    = 1 << 4,
  TELEMETRY_FLAG_SESSION_VALID  = 1 << 5,
};

struct __attribute__((packed)) ControlPacket {
  uint16_t version;
  uint32_t sessionId;
  uint32_t sequence;
  uint32_t clientTimeMs;
  int16_t steeringCmd;  // -1000..1000
  int16_t throttleCmd;  // 0..1000
  int16_t brakeCmd;     // 0..1000
  uint8_t flags;
  uint8_t reserved;
  uint32_t macTag;
};

struct __attribute__((packed)) TelemetryPacket {
  uint16_t version;
  uint32_t sessionId;
  uint32_t sequence;
  uint32_t deviceTimeMs;
  int16_t currentSteeringOut;  // tenths of percent
  int16_t currentThrottleOut;  // tenths of percent
  int16_t virtualSpeedPct;     // tenths of percent
  uint16_t packetAgeMs;
  uint16_t wifiRssi;           // absolute RSSI magnitude (52 = -52 dBm)
  uint16_t failsafeCount;
  uint16_t droppedPacketCount;
  uint8_t stateFlags;
};

static_assert(sizeof(ControlPacket) == 26, "Unexpected ControlPacket size");
static_assert(sizeof(TelemetryPacket) == 29, "Unexpected TelemetryPacket size");
