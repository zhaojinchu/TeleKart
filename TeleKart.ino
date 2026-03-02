#include <WiFi.h>

#include "app_state.h"
#include "config.h"
#include "control_link.h"
#include "drive_control.h"
#include "network_manager.h"
#include "rc_output.h"
#include "web_handlers.h"

void setup() {
  Serial.begin(115200);

  rc_output_init();

  Serial.println("Arming ESC...");
  delay(3000);

  network_manager_init();
  web_handlers_init();
  control_link_init();
  drive_control_init();

  Serial.print("Vehicle name: ");
  Serial.println(networkConfig.vehicleName);
  Serial.print("Station MAC: ");
  Serial.println(network_manager_station_mac());
}

void loop() {
  const uint32_t now = millis();

  network_manager_poll(now);
  control_link_poll(now);
  drive_control_poll(now);
  server.handleClient();
}
