#include "control_link.h"

#include <math.h>
#include <WiFi.h>
#include <mbedtls/md.h>

#include "app_state.h"
#include "config.h"
#include "control_protocol.h"
#include "network_manager.h"
#include "rc_output.h"

namespace {

uint32_t hmacTag(const uint8_t* data, size_t len) {
  const String key = networkConfig.sharedKey.length() ? networkConfig.sharedKey : String(DEFAULT_SHARED_KEY);

  const mbedtls_md_info_t* info = mbedtls_md_info_from_type(MBEDTLS_MD_SHA256);
  if (info == nullptr) {
    return 0;
  }

  unsigned char digest[32];
  mbedtls_md_context_t ctx;
  mbedtls_md_init(&ctx);

  if (mbedtls_md_setup(&ctx, info, 1) != 0) {
    mbedtls_md_free(&ctx);
    return 0;
  }

  int rc = 0;
  rc |= mbedtls_md_hmac_starts(&ctx, reinterpret_cast<const unsigned char*>(key.c_str()), key.length());
  rc |= mbedtls_md_hmac_update(&ctx, data, len);
  rc |= mbedtls_md_hmac_finish(&ctx, digest);
  mbedtls_md_free(&ctx);

  if (rc != 0) {
    return 0;
  }

  return (static_cast<uint32_t>(digest[0]) << 24) |
         (static_cast<uint32_t>(digest[1]) << 16) |
         (static_cast<uint32_t>(digest[2]) << 8) |
         static_cast<uint32_t>(digest[3]);
}

uint32_t computeControlTag(ControlPacket packet) {
  packet.macTag = 0;
  return hmacTag(reinterpret_cast<const uint8_t*>(&packet), sizeof(packet));
}

void ensureUdpState() {
  if (diagnosticsState.fallbackApActive || WiFi.status() != WL_CONNECTED) {
    if (udpListening) {
      controlUdp.stop();
      udpListening = false;
    }
    if (controlSession.valid) {
      control_link_reset_session();
    }
    return;
  }

  if (!udpListening) {
    controlUdp.begin(CONTROL_PORT);
    udpListening = true;
  }
}

void sendTelemetry(uint32_t now) {
  if (!controlSession.valid || !udpListening || controlSession.controllerPort == 0) {
    return;
  }

  if ((now - diagnosticsState.lastTelemetryMs) < TELEMETRY_INTERVAL_MS) {
    return;
  }

  diagnosticsState.lastPacketAgeMs = now - lastCommandMs;

  TelemetryPacket packet = {};
  packet.version = CONTROL_PROTOCOL_VERSION;
  packet.sessionId = controlSession.sessionId;
  packet.sequence = ++diagnosticsState.telemetrySequence;
  packet.deviceTimeMs = now;
  packet.currentSteeringOut = clampInt(static_cast<int>(lroundf(vehicleState.currentSteerPct * 10.0f)), -1000, 1000);
  packet.currentThrottleOut = clampInt(vehicleState.currentThrottlePct * 10, -1000, 1000);
  packet.virtualSpeedPct = clampInt(static_cast<int>(lroundf(vehicleState.virtualSpeedPct * 10.0f)), 0, 1000);
  packet.packetAgeMs = static_cast<uint16_t>(
      diagnosticsState.lastPacketAgeMs > 65535 ? 65535 : diagnosticsState.lastPacketAgeMs);
  packet.wifiRssi = WiFi.status() == WL_CONNECTED ? static_cast<uint16_t>(max(-WiFi.RSSI(), 0)) : 0;
  packet.failsafeCount = diagnosticsState.failsafeCount;
  packet.droppedPacketCount = diagnosticsState.droppedPacketCount;

  uint8_t flags = 0;
  if (diagnosticsState.wifiConnected) flags |= TELEMETRY_FLAG_WIFI_CONNECTED;
  if (diagnosticsState.inFailsafe) flags |= TELEMETRY_FLAG_FAILSAFE;
  if (diagnosticsState.estopLatched) flags |= TELEMETRY_FLAG_ESTOP;
  if (vehicleState.reverseDrive) flags |= TELEMETRY_FLAG_REVERSE_DRIVE;
  if (diagnosticsState.fallbackApActive) flags |= TELEMETRY_FLAG_FALLBACK_AP;
  if (controlSession.valid) flags |= TELEMETRY_FLAG_SESSION_VALID;
  packet.stateFlags = flags;

  controlUdp.beginPacket(controlSession.controllerIp, controlSession.controllerPort);
  controlUdp.write(reinterpret_cast<const uint8_t*>(&packet), sizeof(packet));
  controlUdp.endPacket();

  diagnosticsState.lastTelemetryMs = now;
}

}  // namespace

void control_link_init() {
  control_link_reset_session();
}

void control_link_poll(uint32_t now) {
  ensureUdpState();

  if (!udpListening) {
    return;
  }

  if (controlSession.valid && now > controlSession.expiresAtMs) {
    control_link_reset_session();
  }

  int packetSize = controlUdp.parsePacket();
  while (packetSize > 0) {
    if (packetSize != static_cast<int>(sizeof(ControlPacket))) {
      uint8_t sink[64];
      while (controlUdp.available() > 0) {
        controlUdp.read(sink, min<int>(controlUdp.available(), sizeof(sink)));
      }
      diagnosticsState.droppedPacketCount++;
      packetSize = controlUdp.parsePacket();
      continue;
    }

    ControlPacket packet = {};
    controlUdp.read(reinterpret_cast<uint8_t*>(&packet), sizeof(packet));

    bool accepted = true;
    if (!controlSession.valid) {
      accepted = false;
    } else if (packet.version != CONTROL_PROTOCOL_VERSION) {
      accepted = false;
    } else if (packet.sessionId != controlSession.sessionId) {
      accepted = false;
    } else if (controlUdp.remoteIP() != controlSession.controllerIp) {
      accepted = false;
    } else if (controlSession.controllerPort != 0 && controlUdp.remotePort() != controlSession.controllerPort) {
      accepted = false;
    } else if (packet.sequence <= controlSession.lastSequence) {
      accepted = false;
      diagnosticsState.droppedPacketCount++;
    } else if (packet.macTag != computeControlTag(packet)) {
      accepted = false;
    }

    if (!accepted) {
      diagnosticsState.authFailureCount++;
      packetSize = controlUdp.parsePacket();
      continue;
    }

    controlSession.lastSequence = packet.sequence;
    controlSession.expiresAtMs = now + SESSION_TTL_MS;

    driveInput.steeringCmd = clampInt(packet.steeringCmd, -1000, 1000);
    driveInput.throttleCmd = clampInt(packet.throttleCmd, 0, 1000);
    driveInput.brakeCmd = clampInt(packet.brakeCmd, 0, 1000);
    driveInput.flags = packet.flags;
    driveInput.clientTimeMs = packet.clientTimeMs;
    driveInput.receivedAtMs = now;

    manualOverrideActive = false;
    lastCommandMs = now;
    diagnosticsState.inFailsafe = false;

    packetSize = controlUdp.parsePacket();
  }

  sendTelemetry(now);
}

void control_link_reset_session() {
  controlSession.sessionId = 0;
  controlSession.createdAtMs = 0;
  controlSession.expiresAtMs = 0;
  controlSession.controllerNonce = 0;
  controlSession.lastSequence = 0;
  controlSession.controllerPort = 0;
  controlSession.valid = false;
  controlSession.controllerIp = IPAddress(0, 0, 0, 0);
}

bool control_link_pair(
    const String& vehicleName,
    uint32_t timestampMs,
    uint32_t controllerNonce,
    uint16_t controllerPort,
    uint32_t providedTag,
    const IPAddress& remoteIp,
    String& responseJson) {
  if (!network_manager_is_station_connected()) {
    responseJson = "{\"ok\":false,\"error\":\"wifi_not_connected\"}";
    return false;
  }

  if (controllerPort == 0) {
    responseJson = "{\"ok\":false,\"error\":\"invalid_port\"}";
    return false;
  }

  if (vehicleName != networkConfig.vehicleName) {
    responseJson = "{\"ok\":false,\"error\":\"vehicle_name_mismatch\"}";
    return false;
  }

  uint32_t expectedTag = control_link_compute_pair_tag(vehicleName, timestampMs, controllerNonce, controllerPort);
  if (expectedTag != providedTag) {
    diagnosticsState.authFailureCount++;
    responseJson = "{\"ok\":false,\"error\":\"bad_auth\"}";
    return false;
  }

  if (controlSession.valid &&
      remoteIp == controlSession.controllerIp &&
      controllerPort == controlSession.controllerPort &&
      controllerNonce == controlSession.controllerNonce) {
    responseJson = "{\"ok\":false,\"error\":\"duplicate_pair\"}";
    return false;
  }

  uint32_t now = millis();
  controlSession.sessionId = esp_random();
  if (controlSession.sessionId == 0) {
    controlSession.sessionId = now ^ controllerNonce;
  }
  controlSession.createdAtMs = now;
  controlSession.expiresAtMs = now + SESSION_TTL_MS;
  controlSession.controllerNonce = controllerNonce;
  controlSession.lastSequence = 0;
  controlSession.controllerPort = controllerPort;
  controlSession.valid = true;
  controlSession.controllerIp = remoteIp;

  responseJson =
      "{\"ok\":true,\"session_id\":" + String(controlSession.sessionId) +
      ",\"session_nonce\":" + String(static_cast<uint32_t>(esp_random())) +
      ",\"expires_ms\":" + String(controlSession.expiresAtMs) +
      ",\"ip\":\"" + network_manager_primary_ip().toString() + "\"}";

  return true;
}

uint32_t control_link_compute_pair_tag(
    const String& vehicleName,
    uint32_t timestampMs,
    uint32_t controllerNonce,
    uint16_t controllerPort) {
  String payload = vehicleName + "|" +
                   String(timestampMs) + "|" +
                   String(controllerNonce) + "|" +
                   String(controllerPort);
  return hmacTag(reinterpret_cast<const uint8_t*>(payload.c_str()), payload.length());
}
