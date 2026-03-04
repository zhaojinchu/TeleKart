#include "web_handlers.h"

#include "app_state.h"
#include "config.h"
#include "control_link.h"
#include "drive_control.h"
#include "network_manager.h"
#include "rc_output.h"
#include "web_page.h"

namespace {

String escapeJson(const String& input) {
  String out;
  out.reserve(input.length() + 8);

  for (size_t i = 0; i < input.length(); ++i) {
    char c = input.charAt(i);
    if (c == '"' || c == '\\') {
      out += '\\';
      out += c;
    } else if (c == '\n') {
      out += "\\n";
    } else {
      out += c;
    }
  }

  return out;
}

void sendJson(int code, const String& body) {
  server.sendHeader("Cache-Control", "no-store");
  server.send(code, "application/json", body);
}

String requestBody() {
  if (server.hasArg("plain")) {
    return server.arg("plain");
  }
  return String();
}

bool findJsonValueStart(const String& body, const char* key, int& index) {
  String needle = "\"" + String(key) + "\"";
  int keyPos = body.indexOf(needle);
  if (keyPos < 0) return false;

  int colonPos = body.indexOf(':', keyPos + needle.length());
  if (colonPos < 0) return false;

  index = colonPos + 1;
  while (index < body.length() && (body[index] == ' ' || body[index] == '\n' || body[index] == '\r' || body[index] == '\t')) {
    index++;
  }

  return index < body.length();
}

bool jsonGetString(const String& body, const char* key, String& out) {
  int index = 0;
  if (!findJsonValueStart(body, key, index) || body[index] != '"') {
    return false;
  }

  index++;
  int end = body.indexOf('"', index);
  if (end < 0) return false;

  out = body.substring(index, end);
  return true;
}

bool jsonGetUInt32(const String& body, const char* key, uint32_t& out) {
  int index = 0;
  if (!findJsonValueStart(body, key, index)) {
    return false;
  }

  uint64_t value = 0;
  int end = index;
  while (end < body.length() && (body[end] >= '0' && body[end] <= '9')) {
    value = (value * 10ULL) + static_cast<uint32_t>(body[end] - '0');
    if (value > 0xFFFFFFFFULL) {
      return false;
    }
    end++;
  }

  if (end == index) return false;
  out = static_cast<uint32_t>(value);
  return true;
}

bool jsonGetInt(const String& body, const char* key, int& out) {
  uint32_t tmp = 0;
  if (!jsonGetUInt32(body, key, tmp)) {
    int index = 0;
    if (!findJsonValueStart(body, key, index)) return false;

    int end = index;
    while (end < body.length() && ((body[end] >= '0' && body[end] <= '9') || body[end] == '-')) {
      end++;
    }
    if (end == index) return false;
    out = body.substring(index, end).toInt();
    return true;
  }
  out = static_cast<int>(tmp);
  return true;
}

bool jsonGetBool(const String& body, const char* key, bool& out) {
  int index = 0;
  if (!findJsonValueStart(body, key, index)) {
    return false;
  }

  if (body.startsWith("true", index)) {
    out = true;
    return true;
  }

  if (body.startsWith("false", index)) {
    out = false;
    return true;
  }

  return false;
}

String buildStatusJson() {
  IPAddress ip = network_manager_primary_ip();
  bool reverseRequested = (driveInput.flags & CONTROL_FLAG_REVERSE_REQ) != 0;
  bool estopRequested = (driveInput.flags & CONTROL_FLAG_ESTOP) != 0;

  return
      "{"
      "\"vehicle_name\":\"" + escapeJson(networkConfig.vehicleName) + "\","
      "\"mode\":\"" + network_manager_connection_mode() + "\","
      "\"hostname\":\"" + escapeJson(network_manager_hostname()) + "\","
      "\"station_mac\":\"" + network_manager_station_mac() + "\","
      "\"wifi_connected\":" + String(diagnosticsState.wifiConnected ? "true" : "false") + ","
      "\"fallback_ap_active\":" + String(diagnosticsState.fallbackApActive ? "true" : "false") + ","
      "\"ip\":\"" + ip.toString() + "\","
      "\"rssi\":" + String(diagnosticsState.wifiRssi) + ","
      "\"session_valid\":" + String(controlSession.valid ? "true" : "false") + ","
      "\"session_id\":" + String(controlSession.sessionId) + ","
      "\"controller_ip\":\"" + controlSession.controllerIp.toString() + "\","
      "\"controller_port\":" + String(controlSession.controllerPort) + ","
      "\"packet_age_ms\":" + String(diagnosticsState.lastPacketAgeMs) + ","
      "\"failsafe_count\":" + String(diagnosticsState.failsafeCount) + ","
      "\"dropped_packets\":" + String(diagnosticsState.droppedPacketCount) + ","
      "\"auth_failures\":" + String(diagnosticsState.authFailureCount) + ","
      "\"estop_latched\":" + String(diagnosticsState.estopLatched ? "true" : "false") + ","
      "\"reverse_drive\":" + String(vehicleState.reverseDrive ? "true" : "false") + ","
      "\"reverse_requested\":" + String(reverseRequested ? "true" : "false") + ","
      "\"estop_requested\":" + String(estopRequested ? "true" : "false") + ","
      "\"manual_override\":" + String(manualOverrideActive ? "true" : "false") + ","
      "\"input_steer_cmd_pct\":" + String(static_cast<float>(driveInput.steeringCmd) / 10.0f, 1) + ","
      "\"input_throttle_cmd_pct\":" + String(static_cast<float>(driveInput.throttleCmd) / 10.0f, 1) + ","
      "\"input_brake_cmd_pct\":" + String(static_cast<float>(driveInput.brakeCmd) / 10.0f, 1) + ","
      "\"virtual_speed_pct\":" + String(vehicleState.virtualSpeedPct, 1) + ","
      "\"current_steer_pct\":" + String(vehicleState.currentSteerPct, 1) + ","
      "\"current_throttle_pct\":" + String(vehicleState.currentThrottlePct) +
      "}";
}

String buildConfigJson() {
  return
      "{"
      "\"vehicle_name\":\"" + escapeJson(networkConfig.vehicleName) + "\","
      "\"ssid\":\"" + escapeJson(networkConfig.staSsid) + "\","
      "\"has_password\":" + String(networkConfig.staPassword.length() ? "true" : "false") + ","
      "\"shared_key_set\":" + String(networkConfig.sharedKey.length() ? "true" : "false") + ","
      "\"shared_key\":\"" + escapeJson(networkConfig.sharedKey) + "\","
      "\"fallback_ap_enabled\":" + String(networkConfig.fallbackApEnabled ? "true" : "false") + ","
      "\"steering_trim\":" + String(steeringTrim) + ","
      "\"steer_center_us\":" + String(steerCenterUs) + ","
      "\"steer_left_range_pct\":" + String(steerLeftRangePct) + ","
      "\"steer_right_range_pct\":" + String(steerRightRangePct) +
      "}";
}

void handleRoot() {
  server.send_P(200, "text/html", MAIN_PAGE);
}

void handleCmd() {
  if (controlSession.valid) {
    sendJson(409, "{\"ok\":false,\"error\":\"control_session_active\"}");
    return;
  }

  int throttlePct = manualThrottlePct;
  int steerPct = manualSteerPct;

  if (server.hasArg("th")) {
    throttlePct = clampInt(server.arg("th").toInt(), -100, 100);
  }

  if (server.hasArg("st")) {
    steerPct = clampInt(server.arg("st").toInt(), -100, 100);
  }

  if (server.hasArg("trim")) {
    steeringTrim = clampInt(server.arg("trim").toInt(), -100, 100);
  }

  if (server.hasArg("center")) {
    steerCenterUs = clampInt(server.arg("center").toInt(), 1200, 2400);
  }

  if (server.hasArg("recenter")) {
    if (server.arg("recenter").toInt() == 1) {
      steerCenterUs = clampInt(steerCenterUs + steeringTrim * STEER_TRIM_US_PER_UNIT, 1200, 2400);
      steeringTrim = 0;
    }
  }

  drive_control_set_manual(throttlePct, steerPct);
  sendJson(200, "{\"ok\":true}");
}

void handleStatus() {
  sendJson(200, buildStatusJson());
}

void handleGetConfig() {
  sendJson(200, buildConfigJson());
}

void handlePostConfig() {
  String body = requestBody();
  int value = 0;
  bool clearEstop = false;

  if (jsonGetInt(body, "steering_trim", value)) {
    steeringTrim = clampInt(value, -100, 100);
  }

  if (jsonGetInt(body, "steer_center_us", value)) {
    steerCenterUs = clampInt(value, 1200, 2400);
  }

  if (jsonGetInt(body, "steer_left_range_pct", value)) {
    steerLeftRangePct = clampInt(value, STEER_RANGE_PCT_MIN, STEER_RANGE_PCT_MAX);
  }

  if (jsonGetInt(body, "steer_right_range_pct", value)) {
    steerRightRangePct = clampInt(value, STEER_RANGE_PCT_MIN, STEER_RANGE_PCT_MAX);
  }

  if (jsonGetBool(body, "clear_estop", clearEstop) && clearEstop) {
    drive_control_clear_estop();
  }

  network_manager_save_calibration();
  sendJson(200, buildConfigJson());
}

void handlePostCenterSteering() {
  center_steering_now();
  sendJson(200, "{\"ok\":true,\"steer_us\":" + String(appliedSteerUs) + "}");
}

void handlePostEstop() {
  String body = requestBody();
  bool clearEstop = false;

  if (jsonGetBool(body, "clear", clearEstop) && clearEstop) {
    drive_control_clear_estop();
  } else {
    drive_control_trigger_estop();
  }

  sendJson(200, "{\"ok\":true,\"estop_latched\":" + String(diagnosticsState.estopLatched ? "true" : "false") + "}");
}

void handlePostNetwork() {
  String body = requestBody();
  String ssid = networkConfig.staSsid;
  String password = networkConfig.staPassword;
  String vehicleName = networkConfig.vehicleName;
  String sharedKey = networkConfig.sharedKey;
  String incoming;
  bool fallbackEnabled = networkConfig.fallbackApEnabled;

  if (jsonGetString(body, "ssid", incoming) && incoming.length()) {
    ssid = incoming;
  }
  if (jsonGetString(body, "password", incoming) && incoming.length()) {
    password = incoming;
  }
  if (jsonGetString(body, "vehicle_name", incoming) && incoming.length()) {
    vehicleName = incoming;
  }
  if (jsonGetString(body, "shared_key", incoming)) {
    sharedKey = incoming;
  }
  jsonGetBool(body, "fallback_ap_enabled", fallbackEnabled);

  network_manager_update_network(ssid, password, vehicleName, sharedKey, fallbackEnabled);
  control_link_reset_session();

  sendJson(200, "{\"ok\":true,\"mode\":\"" + network_manager_connection_mode() + "\"}");
}

void handlePostPair() {
  String body = requestBody();
  String vehicleName;
  uint32_t timestampMs = 0;
  uint32_t controllerNonce = 0;
  uint32_t providedTag = 0;
  uint32_t controllerPortValue = 0;

  if (!jsonGetString(body, "vehicle_name", vehicleName) ||
      !jsonGetUInt32(body, "timestamp", timestampMs) ||
      !jsonGetUInt32(body, "controller_nonce", controllerNonce) ||
      !jsonGetUInt32(body, "controller_port", controllerPortValue) ||
      !jsonGetUInt32(body, "mac_tag", providedTag)) {
    sendJson(400, "{\"ok\":false,\"error\":\"invalid_json\"}");
    return;
  }

  if (controllerPortValue == 0 || controllerPortValue > 65535UL) {
    sendJson(400, "{\"ok\":false,\"error\":\"invalid_port\"}");
    return;
  }

  String responseJson;
  bool ok = control_link_pair(
      vehicleName,
      timestampMs,
      controllerNonce,
      static_cast<uint16_t>(controllerPortValue),
      providedTag,
      server.client().remoteIP(),
      responseJson);

  sendJson(ok ? 200 : 403, responseJson);
}

void handleNotFound() {
  server.send(404, "text/plain", "Not found");
}

}  // namespace

void web_handlers_init() {
  server.on("/", HTTP_GET, handleRoot);
  server.on("/cmd", HTTP_GET, handleCmd);
  server.on("/api/status", HTTP_GET, handleStatus);
  server.on("/api/config", HTTP_GET, handleGetConfig);
  server.on("/api/config", HTTP_POST, handlePostConfig);
  server.on("/api/network", HTTP_POST, handlePostNetwork);
  server.on("/api/pair", HTTP_POST, handlePostPair);
  server.on("/api/estop", HTTP_POST, handlePostEstop);
  server.on("/api/center_steering", HTTP_POST, handlePostCenterSteering);
  server.onNotFound(handleNotFound);
  server.begin();
}
