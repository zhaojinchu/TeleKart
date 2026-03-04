#include "network_manager.h"

#include <ESPmDNS.h>
#include <Preferences.h>
#include <WiFi.h>

#include "app_state.h"
#include "config.h"

namespace {

Preferences prefs;
bool prefsReady = false;
bool connectInProgress = false;
bool mdnsStarted = false;
String cachedHostname;

String sanitizeHostname(const String& input) {
  String out;
  out.reserve(input.length());

  for (size_t i = 0; i < input.length(); ++i) {
    char c = input.charAt(i);
    if ((c >= 'a' && c <= 'z') || (c >= '0' && c <= '9')) {
      out += c;
    } else if (c >= 'A' && c <= 'Z') {
      out += static_cast<char>(c - 'A' + 'a');
    } else if (c == '-' || c == '_') {
      out += '-';
    }
  }

  if (out.length() == 0) {
    out = DEFAULT_VEHICLE_NAME;
  }

  return out;
}

void saveConfig() {
  if (!prefsReady) return;

  prefs.putString("ssid", networkConfig.staSsid);
  prefs.putString("pass", networkConfig.staPassword);
  prefs.putString("vehicle", networkConfig.vehicleName);
  prefs.putString("shared", networkConfig.sharedKey);
  prefs.putBool("fallback", networkConfig.fallbackApEnabled);
  prefs.putInt("trim", steeringTrim);
  prefs.putInt("center", steerCenterUs);
  prefs.putInt("range_l", steerLeftRangePct);
  prefs.putInt("range_r", steerRightRangePct);
}

void loadConfig() {
  prefsReady = prefs.begin("telekart", false);
  if (!prefsReady) {
    return;
  }

  networkConfig.staSsid = prefs.getString("ssid", DEFAULT_STA_SSID);
  networkConfig.staPassword = prefs.getString("pass", DEFAULT_STA_PASSWORD);
  networkConfig.vehicleName = prefs.getString("vehicle", DEFAULT_VEHICLE_NAME);
  networkConfig.sharedKey = prefs.getString("shared", DEFAULT_SHARED_KEY);
  networkConfig.fallbackApEnabled = prefs.getBool("fallback", DEFAULT_FALLBACK_AP_ON);

  steeringTrim = prefs.getInt("trim", steeringTrim);
  if (steeringTrim < -100) steeringTrim = -100;
  if (steeringTrim > 100) steeringTrim = 100;

  steerCenterUs = prefs.getInt("center", steerCenterUs);
  if (steerCenterUs < 1200) steerCenterUs = 1200;
  if (steerCenterUs > 2400) steerCenterUs = 2400;

  steerLeftRangePct = prefs.getInt("range_l", DEFAULT_STEER_LEFT_RANGE_PCT);
  if (steerLeftRangePct < STEER_RANGE_PCT_MIN) steerLeftRangePct = STEER_RANGE_PCT_MIN;
  if (steerLeftRangePct > STEER_RANGE_PCT_MAX) steerLeftRangePct = STEER_RANGE_PCT_MAX;

  steerRightRangePct = prefs.getInt("range_r", DEFAULT_STEER_RIGHT_RANGE_PCT);
  if (steerRightRangePct < STEER_RANGE_PCT_MIN) steerRightRangePct = STEER_RANGE_PCT_MIN;
  if (steerRightRangePct > STEER_RANGE_PCT_MAX) steerRightRangePct = STEER_RANGE_PCT_MAX;
}

void stopMdns() {
  if (mdnsStarted) {
    MDNS.end();
    mdnsStarted = false;
  }
}

void startMdnsIfPossible() {
  if (mdnsStarted || !network_manager_is_station_connected()) {
    return;
  }

  cachedHostname = sanitizeHostname(networkConfig.vehicleName);
  if (MDNS.begin(cachedHostname.c_str())) {
    MDNS.addService("http", "tcp", HTTP_PORT);
    mdnsStarted = true;
  }
}

void beginStationConnect() {
  stopMdns();
  diagnosticsState.fallbackApActive = false;

  cachedHostname = sanitizeHostname(networkConfig.vehicleName);

  WiFi.mode(WIFI_STA);
  WiFi.disconnect(true, true);
  delay(50);
  WiFi.setHostname(cachedHostname.c_str());
  WiFi.begin(networkConfig.staSsid.c_str(), networkConfig.staPassword.c_str());

  lastNetworkAttemptMs = millis();
  connectInProgress = true;
}

void startFallbackApInternal() {
  stopMdns();

  WiFi.disconnect(true, true);
  delay(50);
  WiFi.mode(WIFI_AP);
  WiFi.softAP(FALLBACK_AP_SSID, FALLBACK_AP_PASS);

  diagnosticsState.fallbackApActive = true;
  connectInProgress = false;
  udpListening = false;
}

bool setupButtonPressed() {
  if (SETUP_BUTTON_PIN < 0) {
    return false;
  }

  return digitalRead(SETUP_BUTTON_PIN) == LOW;
}

}  // namespace

void network_manager_init() {
  if (SETUP_BUTTON_PIN >= 0) {
    pinMode(SETUP_BUTTON_PIN, INPUT_PULLUP);
  }

  loadConfig();

  if (setupButtonPressed()) {
    network_manager_force_fallback_ap();
    return;
  }

  if (networkConfig.staSsid.length() == 0) {
    network_manager_force_fallback_ap();
    return;
  }

  beginStationConnect();
}

void network_manager_poll(uint32_t now) {
  wl_status_t wifiStatus = WiFi.status();
  bool connected = (wifiStatus == WL_CONNECTED);

  diagnosticsState.wifiConnected = connected;
  diagnosticsState.wifiRssi = connected ? WiFi.RSSI() : 0;

  if (connected) {
    diagnosticsState.fallbackApActive = false;
    connectInProgress = false;
    startMdnsIfPossible();
    return;
  }

  if (diagnosticsState.fallbackApActive) {
    return;
  }

  if (connectInProgress) {
    if ((now - lastNetworkAttemptMs) >= NETWORK_CONNECT_TIMEOUT_MS) {
      if (networkConfig.fallbackApEnabled) {
        startFallbackApInternal();
      } else {
        connectInProgress = false;
      }
    }
    return;
  }

  if ((now - lastNetworkAttemptMs) >= NETWORK_RETRY_INTERVAL_MS) {
    beginStationConnect();
  }
}

void network_manager_force_fallback_ap() {
  startFallbackApInternal();
}

bool network_manager_update_network(
    const String& ssid,
    const String& password,
    const String& vehicleName,
    const String& sharedKey,
    bool fallbackApEnabled) {
  networkConfig.staSsid = ssid;
  networkConfig.staPassword = password;
  networkConfig.vehicleName = vehicleName.length() ? vehicleName : String(DEFAULT_VEHICLE_NAME);
  networkConfig.sharedKey = sharedKey.length() ? sharedKey : String(DEFAULT_SHARED_KEY);
  networkConfig.fallbackApEnabled = fallbackApEnabled;

  saveConfig();
  beginStationConnect();
  return true;
}

void network_manager_save_calibration() {
  saveConfig();
}

bool network_manager_is_station_connected() {
  return WiFi.status() == WL_CONNECTED;
}

IPAddress network_manager_primary_ip() {
  if (network_manager_is_station_connected()) {
    return WiFi.localIP();
  }

  if (diagnosticsState.fallbackApActive) {
    return WiFi.softAPIP();
  }

  return IPAddress(0, 0, 0, 0);
}

String network_manager_connection_mode() {
  if (network_manager_is_station_connected()) {
    return "station";
  }

  if (diagnosticsState.fallbackApActive) {
    return "fallback_ap";
  }

  return "connecting";
}

String network_manager_hostname() {
  if (cachedHostname.length() == 0) {
    cachedHostname = sanitizeHostname(networkConfig.vehicleName);
  }
  return cachedHostname;
}

String network_manager_station_mac() {
  return WiFi.macAddress();
}
