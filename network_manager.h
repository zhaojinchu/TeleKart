#pragma once

#include <Arduino.h>
#include <IPAddress.h>

void network_manager_init();
void network_manager_poll(uint32_t now);
void network_manager_force_fallback_ap();
bool network_manager_update_network(
    const String& ssid,
    const String& password,
    const String& vehicleName,
    const String& sharedKey,
    bool fallbackApEnabled);
void network_manager_save_calibration();

bool network_manager_is_station_connected();
IPAddress network_manager_primary_ip();
String network_manager_connection_mode();
String network_manager_hostname();
String network_manager_station_mac();
